"""Compute standard metrics from a dataflash .BIN log of the standard profile.

All metrics come from the onboard log (full rate, simulated time), never from
MAVLink telemetry, so they're comparable across runs.

Usage:  python3 analyze.py path/to/00000001.BIN
"""
import json
import math
import sys

from pymavlink import mavutil

import flight_profile as fp

LOG_TYPES = ["CMD", "MISE", "MODE", "ARM", "DSF", "ANG", "ATT", "RATE",
             "CTUN", "POS", "BAT", "RCOU", "GPS", "MSG"]
GUIDED, LAND = 4, 9


def load(path):
    log = mavutil.mavlink_connection(path)
    data = {t: [] for t in LOG_TYPES}
    while True:
        msg = log.recv_match(type=LOG_TYPES)
        if msg is None:
            break
        row = msg.to_dict()
        row["t"] = msg.TimeUS / 1e6
        data[msg.get_type()].append(row)
    # ArduPilot 4.6+ logs controller attitude as ANG; ATT there is low-rate AHRS only
    if data["ANG"]:
        data["ATT"] = data["ANG"]
    return data


# ---------------------------------------------------------------- helpers

def window(rows, t0, t1):
    return [r for r in rows if t0 <= r["t"] < t1]


def rms(values):
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else None


def angdiff(a, b):
    return (a - b + 180) % 360 - 180


def first_within(rows, err, tol, t0):
    """Seconds after t0 until |err| first drops inside tol."""
    for r in rows:
        if abs(err(r)) <= tol:
            return r["t"] - t0
    return None


def settle_time(rows, err, tol, t0):
    """Seconds after t0 until |err| enters tol and stays there to the end of rows."""
    settled_at = None
    for r in rows:
        if abs(err(r)) > tol:
            settled_at = None
        elif settled_at is None:
            settled_at = r["t"]
    return settled_at - t0 if settled_at is not None else None


def dist_to_path(n, e, path):
    best = float("inf")
    for (n1, e1), (n2, e2) in zip(path, path[1:]):
        dn, de = n2 - n1, e2 - e1
        seg2 = dn * dn + de * de
        k = max(0.0, min(1.0, ((n - n1) * dn + (e - e1) * de) / seg2)) if seg2 else 0.0
        best = min(best, math.hypot(n - (n1 + k * dn), e - (e1 + k * de)))
    return best


def rnd(v, nd=3):
    return round(v, nd) if isinstance(v, float) else v


# ---------------------------------------------------------------- segmentation

def segments(data):
    """{name: (t_start, t_end)} for each mission segment plus 'att_steps'."""
    names = fp.segment_names()
    if data["MISE"]:
        # MISE = mission item executed (newer ArduPilot)
        cmds = data["MISE"]
    else:
        # Older versions log execution as CMD, after a dump of the whole
        # mission at log start; execution begins at the last item-1 entry.
        cmds = data["CMD"]
        starts = [i for i, c in enumerate(cmds) if c["CNum"] == 1]
        cmds = cmds[starts[-1]:] if starts else []

    # AUTO can start the mission before the vehicle is actually armed
    armed = [a["t"] for a in data["ARM"] if a.get("ArmState") == 1]
    arm_t = armed[0] if armed else 0.0

    spans = []
    for c in cmds:
        name = names.get(c["CNum"])
        if name is None or (spans and spans[-1][0] == name):
            continue
        t = max(c["t"], arm_t)
        if spans:
            spans[-1][2] = t
        spans.append([name, t, None])
    if not spans:
        raise ValueError("no mission commands in log - was the standard profile flown?")

    mission_start = spans[0][1]
    modes = [(m["t"], m["ModeNum"] if "ModeNum" in m else m["Mode"]) for m in data["MODE"]]
    guided_t = next((t for t, n in modes if t > mission_start and n == GUIDED), None)
    land_t = next((t for t, n in modes if guided_t and t > guided_t and n == LAND), None)
    end_t = data["ATT"][-1]["t"]

    spans[-1][2] = guided_t or end_t
    segs = {name: (t0, t1) for name, t0, t1 in spans}
    if guided_t:
        segs["att_steps"] = (guided_t, land_t or end_t)
    segs["_mission"] = (mission_start, guided_t or end_t)
    segs["_flight"] = (mission_start, end_t)
    return segs


# ---------------------------------------------------------------- metrics

def ne(row):
    return fp.latlon_to_ne(row["Lat"], row["Lng"])


def global_metrics(d, segs):
    t0, t1 = segs["_mission"]
    out = {}
    rate = window(d["RATE"], t0, t1)
    for ax, a, des in (("roll", "R", "RDes"), ("pitch", "P", "PDes"), ("yaw", "Y", "YDes")):
        out[f"rate_rms_{ax}_dps"] = rms(r[a] - r[des] for r in rate)
    att = window(d["ATT"], t0, t1)
    out["att_rms_roll_deg"] = rms(r["Roll"] - r["DesRoll"] for r in att)
    out["att_rms_pitch_deg"] = rms(r["Pitch"] - r["DesPitch"] for r in att)
    ctun = window(d["CTUN"], segs["hover"][0], t1)   # skip takeoff transient
    out["alt_rms_m"] = rms(r["Alt"] - r["DAlt"] for r in ctun)

    rcou = window(d["RCOU"], t0, t1)
    if rcou:
        sat = sum(1 for r in rcou if max(r["C1"], r["C2"], r["C3"], r["C4"]) >= 1950)
        out["motor_sat_pct"] = 100.0 * sat / len(rcou)

    ft0, ft1 = segs["_flight"]
    out["flight_time_s"] = ft1 - ft0
    bat = window(d["BAT"], ft0, ft1 + 1)
    if bat:
        out["mah_total"] = bat[-1]["CurrTot"] - bat[0]["CurrTot"]
        out["min_voltage"] = min(r["Volt"] for r in bat)
    return out


def hover_metrics(d, t0, t1):
    pos = window(d["POS"], t0, t1)
    ctun = window(d["CTUN"], t0, t1)
    rcou = window(d["RCOU"], t0, t1)
    bat = window(d["BAT"], t0, t1)
    out = {
        "hover_pos_rms_m": rms(math.hypot(*ne(r)) for r in pos),
        "hover_throttle": mean(r["ThO"] for r in ctun),
        "hover_amps": mean(r["Curr"] for r in bat),
    }
    if rcou:  # motor imbalance: spread between highest and lowest motor
        out["hover_motor_spread_us"] = mean(
            max(r["C1"], r["C2"], r["C3"], r["C4"]) - min(r["C1"], r["C2"], r["C3"], r["C4"])
            for r in rcou)
    return out


def step_north_metrics(d, t0, t1, target=30.0):
    pos = window(d["POS"], t0, t1)
    err = lambda r: ne(r)[0] - target
    return {
        "step_overshoot_m": max(0.0, max(ne(r)[0] for r in pos) - target),
        "step_time_to_1m_s": first_within(pos, err, 1.0, t0),
        "step_settle_0p3m_s": settle_time(pos, err, 0.3, t0),
    }


def box_metrics(d, t0, t1):
    pos = window(d["POS"], t0, t1)
    xt = [dist_to_path(*ne(r), fp.BOX_PATH) for r in pos]
    return {"box_xtrack_rms_m": rms(xt), "box_xtrack_max_m": max(xt), "box_time_s": t1 - t0}


def alt_step_metrics(d, t0, t1, target, prefix, up):
    ctun = window(d["CTUN"], t0, t1)
    err = lambda r: r["Alt"] - target
    peak = max(r["Alt"] for r in ctun) - target if up else target - min(r["Alt"] for r in ctun)
    return {
        f"{prefix}_overshoot_m": max(0.0, peak),
        f"{prefix}_time_to_0p3m_s": first_within(ctun, err, 0.3, t0),
    }


def yaw_metrics(d, t0, t1, target, prefix):
    att = window(d["ATT"], t0, t1)
    start = att[0]["Yaw"]
    sign = 1 if angdiff(target, start) > 0 else -1
    err = lambda r: angdiff(r["Yaw"], target)
    return {
        f"{prefix}_overshoot_deg": max(0.0, max(sign * err(r) for r in att)),
        f"{prefix}_time_to_3deg_s": first_within(att, err, 3.0, t0),
        f"{prefix}_track_rms_deg": rms(angdiff(r["Yaw"], r["DesYaw"]) for r in att),
    }


def speed_run_metrics(d, t0, t1):
    gps = window(d["GPS"], t0, t1)
    att = window(d["ATT"], t0, t1)
    bat = window(d["BAT"], t0, t1)
    return {
        "speed_max_mps": max(r["Spd"] for r in gps) if gps else None,
        "speed_max_tilt_deg": max(math.hypot(r["Roll"], r["Pitch"]) for r in att),
        "speed_peak_amps": max(r["Curr"] for r in bat) if bat else None,
        "speed_mah": bat[-1]["CurrTot"] - bat[0]["CurrTot"] if bat else None,
    }


def att_step_metrics(d, t0, t1):
    """Rise time / overshoot / tracking for each commanded attitude step."""
    att = window(d["ATT"], t0, t1)
    out = {}
    for axis, act_f, des_f in (("roll", "Roll", "DesRoll"), ("pitch", "Pitch", "DesPitch")):
        cmds = [deg for ax, deg in fp.ATTITUDE_STEPS if ax == axis]
        if not cmds:
            continue
        mag = abs(cmds[0])
        rises, overs, tracks = [], [], []
        i = 0
        while i < len(att):
            # a step is the desired angle passing half the command; it starts
            # where the desired angle last left level
            if abs(att[i][des_f]) > 0.5 * mag and (i == 0 or abs(att[i - 1][des_f]) <= 0.5 * mag):
                j = i
                while j > 0 and abs(att[j - 1][des_f]) > 1.0:
                    j -= 1
                ts = att[j]["t"]
                sign = 1 if att[i][des_f] > 0 else -1
                w = [r for r in att if ts <= r["t"] < ts + fp.STEP_HOLD_S]
                rise = next((r["t"] - ts for r in w if sign * r[act_f] >= 0.9 * mag), None)
                if rise is not None:
                    rises.append(rise)
                overs.append(100.0 * max(0.0, max(sign * r[act_f] for r in w) - mag) / mag)
                tracks.append(rms(r[act_f] - r[des_f] for r in w))
                i += len(w)
            else:
                i += 1
        out[f"{axis}_step_rise_s"] = mean(rises)
        out[f"{axis}_step_overshoot_pct"] = mean(overs)
        out[f"{axis}_step_track_rms_deg"] = mean(t for t in tracks if t is not None)
        out[f"{axis}_steps_detected"] = len(overs)
    return out


def log_coverage(d, t0, t1):
    """How much of the attitude/rate data survived logging during the flight.

    Expected sample count comes from the median sample interval, so this works
    at any logging rate. A few dropped messages spread evenly don't matter;
    long gaps do, because they can swallow a whole step response.
    """
    worst_cov, worst_gap = 1.0, 0.0
    for t in ("ATT", "RATE"):
        ts = [r["t"] for r in window(d[t], t0, t1)]
        if len(ts) < 2:
            return {"log_coverage": 0.0, "log_max_gap_ms": None}
        dts = sorted(b - a for a, b in zip(ts, ts[1:]))
        median = dts[len(dts) // 2]
        worst_cov = min(worst_cov, len(ts) / ((t1 - t0) / median + 1))
        worst_gap = max(worst_gap, dts[-1])
    return {"log_coverage": min(1.0, worst_cov), "log_max_gap_ms": worst_gap * 1000}


def analyze(path):
    d = load(path)
    segs = segments(d)
    m = global_metrics(d, segs)
    # Dropped log messages make runs incomparable; anything above 0 is a warning
    m["log_dropped"] = d["DSF"][-1]["Dp"] if d["DSF"] else 0
    m.update(log_coverage(d, *segs["_flight"]))
    m["firmware"] = next((r["Message"] for r in d["MSG"]
                          if r["Message"].startswith(("ArduCopter", "ArduPilot"))), None)

    per_segment = [
        ("hover", hover_metrics, {}),
        ("step_north", step_north_metrics, {}),
        ("box", box_metrics, {}),
        ("climb", alt_step_metrics, dict(target=20.0, prefix="climb", up=True)),
        ("descend", alt_step_metrics, dict(target=fp.ALT, prefix="descend", up=False)),
        ("yaw_90", yaw_metrics, dict(target=90, prefix="yaw90")),
        ("yaw_0", yaw_metrics, dict(target=0, prefix="yaw0")),
        ("speed_run", speed_run_metrics, {}),
        ("att_steps", att_step_metrics, {}),
    ]
    for name, func, kwargs in per_segment:
        if name not in segs:
            continue
        try:
            m.update(func(d, *segs[name], **kwargs))
        except (ValueError, IndexError, ZeroDivisionError) as e:
            m[f"{name}_error"] = f"{type(e).__name__}: {e}"
    m["segments_found"] = ",".join(k for k in segs if not k.startswith("_"))
    return {k: rnd(v) for k, v in m.items()}


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    print(json.dumps(analyze(sys.argv[1]), indent=2))
