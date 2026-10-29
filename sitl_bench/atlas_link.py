"""Bridge between the SITL bench and the ATLAS repository.

Builds the simulated airframe and tune from a stored ATLAS aircraft, checks
every proposed tune change with ATLAS's own safety layer, scores each run
0..1, and files the run into ATLAS through ATLAS's own functions
(record_sim_run, ingest_sim_flight). Nothing here writes ATLAS tables
directly, so ATLAS's rules still hold: simulated runs are tagged
source='sitl', never change an aircraft's validation status, and only enter
training when "Include SITL runs in training" is ticked.
"""
import json
import math
import os
import sqlite3
import sys

# Score definition. Bump the version whenever a constant below changes, so
# scores from different definitions are never compared or trained on together.
SCORE_VERSION = "bench-score-v1"
SCORE_WEIGHTS = {"tracking": 0.35, "step": 0.25, "oscillation": 0.25, "navigation": 0.15}

# A run is scored only if the log kept this much of the attitude/rate data
# and never lost more than this in one gap (a step response takes ~150 ms).
MIN_LOG_COVERAGE = 0.97
MAX_LOG_GAP_MS = 60

# Aircraft parameters copied from the hardware snapshot into the simulation:
# the tune and the filters/mixer that shape it. Calibration, I/O, GPS,
# failsafe and frame-wiring parameters are deliberately left out. FRAME_TYPE
# only changes motor order, which matters for wiring, not tuning, and the SITL
# frame model must keep its own motor order.
COPY_PREFIXES = ("ATC_", "ACRO_", "MOT_", "INS_HNTCH_", "INS_HNTC2_", "INS_HNTC3_")
COPY_NAMES = {"INS_GYRO_FILTER", "PILOT_Y_RATE"}
COPY_EXCLUDE = {"MOT_PWM_TYPE"}

# Config overrides with these prefixes configure the simulator or logging,
# not the aircraft, so they bypass the aircraft safety check.
SIM_ONLY_PREFIXES = ("SIM_", "LOG_")


def import_atlas(atlas_dir):
    atlas_dir = os.path.abspath(atlas_dir)
    if atlas_dir not in sys.path:
        sys.path.insert(0, atlas_dir)
    import acro_bridge
    import safety
    import sim_logs
    return acro_bridge, safety, sim_logs


def open_db(path, bridge):
    if not os.path.exists(path):
        raise FileNotFoundError(f"ATLAS database not found: {path}")
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    bridge.migrate(conn)
    return conn


# ---------------------------------------------------------------- airframe

def load_aircraft(conn, bridge, aircraft_id, weight_g=None):
    """Features, hardware tune and copied hardware parameters for one aircraft."""
    row = conn.execute("SELECT name FROM aircraft WHERE id=?", (aircraft_id,)).fetchone()
    if row is None:
        raise ValueError(f"no aircraft with id {aircraft_id} in the ATLAS database")
    feats, provenance = bridge.extract_features(conn, aircraft_id)
    weight_from_cli = feats.get("weight_g") is None and weight_g is not None
    if weight_from_cli:
        feats["weight_g"] = float(weight_g)
        provenance.append(f"weight_g: {weight_g} from --weight-g (not stored in ATLAS)")
    if feats.get("weight_g") is None:
        raise ValueError(
            f"'{row['name']}' has no all-up weight recorded. Record it in ATLAS "
            "(the simulated airframe and ATLAS training both need it), or pass "
            "--weight-g to simulate without storing it.")

    targets, missing = bridge.extract_targets(conn, aircraft_id)
    if missing:
        raise ValueError("hardware snapshot is missing tune parameters: " + ", ".join(missing))

    pf = bridge.latest_param_file(conn, aircraft_id)
    hardware = {}
    for r in conn.execute("SELECT name, value FROM parameters WHERE parameter_file_id=?", (pf["id"],)):
        name = r["name"]
        if name in COPY_EXCLUDE:
            continue
        if name in COPY_NAMES or name.startswith(COPY_PREFIXES):
            try:
                hardware[name] = float(r["value"])
            except (TypeError, ValueError):
                pass
    battery_mah = conn.execute(
        "SELECT value FROM parameters WHERE parameter_file_id=? AND name='BATT_CAPACITY'",
        (pf["id"],)).fetchone()
    return {
        "id": aircraft_id, "name": row["name"], "features": feats,
        "provenance": provenance, "weight_from_cli": weight_from_cli,
        "targets": targets, "hardware": hardware,
        "parameter_file_id": pf["id"],
        "battery_mah": float(battery_mah["value"]) if battery_mah else None,
    }


def build_model(template_path, aircraft, out_path):
    """SITL physics model scaled from the template to the aircraft's measurements."""
    with open(template_path) as f:
        model = json.load(f)
    feats, hw = aircraft["features"], aircraft["hardware"]
    mass = feats["weight_g"] / 1000.0
    diag = feats["wheelbase_mm"] / 1000.0
    motors = int(feats.get("motor_qty") or model["num_motors"])
    prop_r = feats["prop_size_in"] * 0.0254 / 2
    cells = feats["cell_count"]

    # inertia scales with mass x arm length squared
    scale = (mass / model["mass"]) * (diag / model["diagonal_size"]) ** 2
    model.update({
        "mass": round(mass, 4),
        "diagonal_size": round(diag, 4),
        "num_motors": motors,
        "disc_area": round(motors * math.pi * prop_r ** 2, 5),
        "maxVoltage": round(4.2 * cells, 2),
        "refVoltage": round(3.83 * cells, 2),
        "moment_inertia": [round(v * scale, 6) for v in model["moment_inertia"]],
    })
    # The real aircraft's learned hover throttle and thrust curve
    if "MOT_THST_HOVER" in hw:
        model["hoverThrOut"] = round(hw["MOT_THST_HOVER"], 4)
    if "MOT_THST_EXPO" in hw:
        model["propExpo"] = hw["MOT_THST_EXPO"]
    if aircraft["battery_mah"]:
        model["battCapacityAh"] = aircraft["battery_mah"] / 1000.0
    with open(out_path, "w") as f:
        json.dump(model, f, indent=2)
    return model


# ---------------------------------------------------------------- tunes

def plan_config(bridge, safety, aircraft, overrides):
    """Turn one bench config into the full parameter set to fly.

    Returns (sitl_params, tune, report). `tune` is the TARGET_PARAMS tune plus
    derived values, which is what ATLAS stores and trains on. Aircraft changes
    go through ATLAS's safety check; if anything is blocked the config must
    not be flown.
    """
    tune = dict(aircraft["targets"])
    aircraft_changes, sim_settings = {}, {}
    for k, v in overrides.items():
        if k.startswith(SIM_ONLY_PREFIXES):
            sim_settings[k] = float(v)
        elif k in bridge.TARGET_PARAMS:
            tune[k] = float(v)
        else:
            aircraft_changes[k] = float(v)

    report = bridge.validate_for_output(tune)   # expands derived params too
    tune = report["expanded"]
    # Changes outside the predicted tune still have to pass the fail-closed check
    extra = safety.check_param_set(aircraft_changes) if aircraft_changes else None
    blocked = report["blocked"] + (extra["blocked"] if extra else [])

    sitl_params = dict(aircraft["hardware"])
    sitl_params.update(tune)
    sitl_params.update(aircraft_changes)
    sitl_params.update(sim_settings)
    return sitl_params, tune, {"all_valid": not blocked, "blocked": blocked}


# ---------------------------------------------------------------- scoring

def score_run(record):
    """One 0..1 outcome score from the bench metrics (higher is better).

    Weighted geometric mean, so one bad term pulls the whole score down:
      tracking    exp(-rate RMS / 20 deg/s), roll and pitch averaged
      step        exp(-overshoot / 25 %) * min(1, 0.3 s / rise time)
      oscillation exp(-hover motor spread / 250 us) * (1 - motor saturation)
      navigation  exp(-box cross-track RMS / 1 m)
    The constants are judgment calls, not derived values; scores are only
    comparable within one SCORE_VERSION.
    """
    def aborted(reason):
        return {"outcome_score": 0.0, "aborted": True, "abort_reason": reason,
                "score_version": SCORE_VERSION}

    if record.get("error"):
        return aborted(record["error"])
    m = record.get("metrics") or {}
    if m.get("log_coverage", 0) < MIN_LOG_COVERAGE or (m.get("log_max_gap_ms") or 1e9) > MAX_LOG_GAP_MS:
        return aborted(f"log incomplete (coverage {m.get('log_coverage')}, longest gap "
                       f"{m.get('log_max_gap_ms')} ms); run not trustworthy")
    try:
        rate_rms = (m["rate_rms_roll_dps"] + m["rate_rms_pitch_dps"]) / 2
        overshoot = (m["roll_step_overshoot_pct"] + m["pitch_step_overshoot_pct"]) / 2
        rise = (m["roll_step_rise_s"] + m["pitch_step_rise_s"]) / 2
        terms = {
            "tracking": math.exp(-rate_rms / 20.0),
            "step": math.exp(-overshoot / 25.0) * min(1.0, 0.3 / rise),
            "oscillation": math.exp(-m["hover_motor_spread_us"] / 250.0)
                           * (1.0 - m.get("motor_sat_pct", 0.0) / 100.0),
            "navigation": math.exp(-m["box_xtrack_rms_m"] / 1.0),
        }
    except (KeyError, TypeError, ZeroDivisionError) as e:
        return aborted(f"metrics incomplete for scoring ({type(e).__name__}: {e})")

    score = math.prod(max(t, 1e-6) ** SCORE_WEIGHTS[k] for k, t in terms.items())
    return {
        "outcome_score": round(score, 4),
        "aborted": False,
        "abort_reason": "",
        "score_version": SCORE_VERSION,
        "terms": {k: round(v, 4) for k, v in terms.items()},
        "rate_rms_dps": round(rate_rms, 3),
        "step_overshoot_pct": round(overshoot, 2),
        "step_rise_s": round(rise, 3),
    }


# ---------------------------------------------------------------- filing

def file_runs(conn, bridge, sim_logs, aircraft, records, files_dir, run_group_prefix):
    """File each run into ATLAS. Returns a list of what was created per run."""
    filed = []
    for r in records:
        if r.get("blocked"):
            continue   # never flown, nothing to file
        score = dict(r["score"])
        score["bench_metrics"] = r.get("metrics", {})
        run_group = f"{run_group_prefix}_{r['config']}"
        sim_id = bridge.record_sim_run(conn, aircraft["id"], run_group, r["tune"], score)
        entry = {"config": r["config"], "repeat": r["repeat"], "sim_run_id": sim_id,
                 "outcome_score": score["outcome_score"], "aborted": score["aborted"]}
        if r.get("log"):
            entry.update(sim_logs.ingest_sim_flight(
                conn, aircraft["id"], r["tune"], score,
                files_dir=files_dir, bin_path=r["log"], run_group=run_group,
                firmware=r.get("firmware") or "SITL", sim_run_id=sim_id))
        filed.append(entry)
    return filed
