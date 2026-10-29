"""
acro_bridge.py — the translation layer between the ATLAS repository schema and
the Acro tuning model.

WHY THIS FILE EXISTS
--------------------
ATLAS and the Acro tuner were built against different data models and neither
can read the other's:

    ATLAS stores                        the model needs
    ---------------------------------   ---------------------------------
    components.specs = "Stator 2207,    motor_kv = 1700.0  (a number)
      1700 KV, Qty 4"  (free text)      motor_qty = 4
    aircraft.frame_size = "5 inch"      prop_size_in = 5.0, wheelbase_mm = 220
    (nothing)                           weight_g, cell_count
    parameters(name TEXT, value TEXT)   ATC_RAT_RLL_P = 0.03205 (a number)
    validation_status = 'UNTESTED'      outcome label in 0..1, or EXCLUDED

Everything that converts between those two columns lives here and nowhere
else. The GUI reads rows, the model reads vectors, and this module is the only
thing that knows how one becomes the other.

THE ONE RULE IT ENFORCES
------------------------
UNTESTED aircraft are EXCLUDED from training, never scored 0. An unflown
configuration is absence of evidence, not evidence of failure. Scoring it zero
teaches the model to avoid configurations nobody has tried yet, which is the
opposite of useful.
"""

from __future__ import annotations

import json
import re
import sqlite3
import time
from typing import Any

import safety

# --------------------------------------------------------------------------
# What the model predicts
#
# Only the FREE parameters. The real TBS Lucid H7 parameter file shows roll
# P == roll I exactly, and yaw I == yaw P / 10 — these are conventions, not
# independent choices. Predicting them separately would let the model emit a
# combination no human would ever write, so they are derived instead.
# --------------------------------------------------------------------------
TARGET_PARAMS = [
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_D",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_D",
    "ATC_RAT_YAW_P",
    "ACRO_RP_RATE",
    "ACRO_Y_RATE",
    "INS_GYRO_FILTER",
]

#: derived_name -> (source_name, multiplier)
DERIVED_PARAMS = {
    "ATC_RAT_RLL_I": ("ATC_RAT_RLL_P", 1.0),
    "ATC_RAT_PIT_I": ("ATC_RAT_PIT_P", 1.0),
    "ATC_RAT_YAW_I": ("ATC_RAT_YAW_P", 0.1),
}

FEATURE_FIELDS = [
    "motor_kv",
    "motor_qty",
    "prop_size_in",
    "weight_g",
    "cell_count",
    "wheelbase_mm",
]

#: Human labels for the GUI.
FEATURE_LABELS = {
    "motor_kv": "Motor KV",
    "motor_qty": "Motor count",
    "prop_size_in": "Prop size (in)",
    "weight_g": "All-up weight (g)",
    "cell_count": "Battery cells (S)",
    "wheelbase_mm": "Wheelbase (mm)",
}

#: Features the model cannot sensibly guess around. Without these two a
#: prediction is not worth making.
REQUIRED_FEATURES = ["motor_kv", "prop_size_in"]

FRAME_MOTOR_COUNT = {
    "quadcopter": 4, "quad": 4,
    "hexacopter": 6, "hex": 6,
    "octocopter": 8, "octo": 8,
}

#: Rough prop-size -> wheelbase mapping, used only when wheelbase was not
#: recorded. These are typical values for common freestyle/cinematic builds,
#: not a specification. A recorded wheelbase always wins.
PROP_TO_WHEELBASE_MM = {
    2.0: 85, 2.5: 100, 3.0: 140, 3.5: 155, 4.0: 175,
    5.0: 220, 6.0: 260, 7.0: 295, 8.0: 350, 10.0: 450,
}

VALIDATION_LABEL = {
    "FULLY_VALIDATED": 1.0,
    "PARTIALLY_VALIDATED": 0.7,
    "REJECTED": 0.0,
    # UNTESTED is deliberately absent — see the module docstring.
}

SOURCE_HARDWARE = "hardware"
SOURCE_SIM = "sitl"


# ==========================================================================
# Schema migration
# ==========================================================================
MIGRATION_SQL = """
CREATE TABLE IF NOT EXISTS acro_outcomes(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    aircraft_id INTEGER NOT NULL,
    flight_test_id INTEGER,
    outcome_score REAL,
    tracking_error REAL,
    vibration_max REAL,
    oscillation_detected INTEGER DEFAULT 0,
    source TEXT NOT NULL DEFAULT 'hardware',
    notes TEXT,
    recorded_at TEXT,
    FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE,
    FOREIGN KEY(flight_test_id) REFERENCES flight_tests(id) ON DELETE SET NULL
);
CREATE INDEX IF NOT EXISTS idx_acro_outcomes_aircraft
    ON acro_outcomes(aircraft_id);

CREATE TABLE IF NOT EXISTS sim_runs(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    aircraft_id INTEGER,
    run_group TEXT NOT NULL,
    started_utc TEXT NOT NULL,
    params_json TEXT NOT NULL,
    score_json TEXT NOT NULL,
    outcome_score REAL,
    aborted INTEGER NOT NULL DEFAULT 0,
    abort_reason TEXT,
    FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE SET NULL
);
CREATE INDEX IF NOT EXISTS idx_sim_runs_group ON sim_runs(run_group);

CREATE TABLE IF NOT EXISTS applied_params(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    aircraft_id INTEGER,
    applied_utc TEXT NOT NULL,
    connection TEXT,
    params_json TEXT NOT NULL,
    confirmed_json TEXT,
    origin TEXT,
    operator_note TEXT,
    FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE SET NULL
);
"""

#: New nullable columns on `aircraft`. ATLAS v8 captured none of these, and
#: four of the six model features cannot be derived without them.
NEW_AIRCRAFT_COLUMNS = [
    ("weight_g", "REAL"),
    ("prop_size_in", "REAL"),
    ("cell_count", "INTEGER"),
    ("wheelbase_mm", "REAL"),
]

#: New columns on `flight_tests`. Without `source`, a simulated flight filed as
#: evidence is indistinguishable from a real one in every query and every view.
NEW_FLIGHT_TEST_COLUMNS = [
    ("source", "TEXT NOT NULL DEFAULT 'hardware'"),
]

#: Same for parameter_files. A simulated run files the tune it flew, which
#: becomes that aircraft's newest snapshot — and "newest snapshot" is what case
#: export and target extraction both reach for.
NEW_PARAM_FILE_COLUMNS = [
    ("source", "TEXT NOT NULL DEFAULT 'hardware'"),
]


#: Child tables that must follow their aircraft into the grave.
CASCADE_TABLES = {
    "components": """CREATE TABLE components(
        id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
        category TEXT NOT NULL, manufacturer TEXT, model TEXT, specs TEXT,
        FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE)""",
    "parameter_files": """CREATE TABLE parameter_files(
        id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
        original_name TEXT, stored_path TEXT, ardupilot_version TEXT, imported_at TEXT,
        source TEXT NOT NULL DEFAULT 'hardware',
        FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE)""",
    "parameters": """CREATE TABLE parameters(
        id INTEGER PRIMARY KEY AUTOINCREMENT, parameter_file_id INTEGER NOT NULL,
        name TEXT NOT NULL, value TEXT,
        FOREIGN KEY(parameter_file_id) REFERENCES parameter_files(id) ON DELETE CASCADE)""",
    "flight_tests": """CREATE TABLE flight_tests(
        id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
        log_name TEXT, stored_path TEXT, test_date TEXT, result TEXT, notes TEXT,
        parameter_file_id INTEGER REFERENCES parameter_files(id) ON DELETE SET NULL,
        source TEXT NOT NULL DEFAULT 'hardware',
        FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE)""",
}


#: Columns to carry across during a rebuild, per table.
CASCADE_COLUMNS = {
    "components": ["id", "aircraft_id", "category", "manufacturer", "model", "specs"],
    "parameter_files": ["id", "aircraft_id", "original_name", "stored_path",
                        "ardupilot_version", "imported_at", "source"],
    "parameters": ["id", "parameter_file_id", "name", "value"],
    "flight_tests": ["id", "aircraft_id", "log_name", "stored_path", "test_date",
                     "result", "notes", "parameter_file_id", "source"],
}


def _missing_cascades(conn: sqlite3.Connection) -> list[str]:
    """Which child tables were created without ON DELETE CASCADE.

    CREATE TABLE IF NOT EXISTS does not alter an existing table, so a database
    created by an earlier build keeps its original constraints no matter what
    the current source declares. The shipped atlas_repository.db is in exactly
    that state: atlas_v8.py says CASCADE, the file says NO ACTION, and
    deleting an aircraft fails with a foreign-key error.
    """
    broken = []
    for table, parent, column in (("components", "aircraft", "aircraft_id"),
                                  ("parameter_files", "aircraft", "aircraft_id"),
                                  ("flight_tests", "aircraft", "aircraft_id"),
                                  ("parameters", "parameter_files", "parameter_file_id")):
        try:
            fks = list(conn.execute(f"PRAGMA foreign_key_list({table})"))
        except sqlite3.OperationalError:
            continue                      # table not created yet
        for fk in fks:
            if fk[2] == parent and fk[3] == column:
                if (fk[6] or "").upper() != "CASCADE":
                    broken.append(table)
    return broken


def repair_cascades(conn: sqlite3.Connection) -> list[str]:
    """Rebuild child tables that are missing their delete cascade.

    Uses SQLite's documented table-rebuild sequence. Foreign keys must be off
    for it, and that pragma is a no-op inside a transaction, so this commits
    first and runs the rebuild outside one.
    """
    broken = _missing_cascades(conn)
    if not broken:
        return []
    conn.commit()
    conn.execute("PRAGMA foreign_keys=OFF")
    # Without legacy_alter_table, RENAME TO rewrites references to this table
    # in OTHER tables' definitions — so renaming flight_tests out of the way
    # silently repoints acro_outcomes at _old_flight_tests, which is then
    # dropped, leaving a dangling reference that breaks every later delete.
    # SQLite's own table-rebuild procedure turns this pragma on for exactly
    # this reason.
    conn.execute("PRAGMA legacy_alter_table=ON")
    try:
        # Rebuild parents before children so each child's new definition points
        # at a table that already exists.
        for table in ("components", "parameter_files", "parameters", "flight_tests"):
            if table not in broken:
                continue
            # Copy only the columns both the old and new definitions have, so
            # a database with extra legacy columns still migrates.
            have = {r[1] for r in conn.execute(f"PRAGMA table_info({table})")}
            kept = [c for c in CASCADE_COLUMNS[table] if c in have]
            collist = ", ".join(kept)
            conn.execute(f"ALTER TABLE {table} RENAME TO _old_{table}")
            conn.execute(CASCADE_TABLES[table])
            conn.execute(f"INSERT INTO {table} ({collist}) "
                         f"SELECT {collist} FROM _old_{table}")
            conn.execute(f"DROP TABLE _old_{table}")
        violations = list(conn.execute("PRAGMA foreign_key_check"))
        if violations:
            raise sqlite3.IntegrityError(
                f"cascade repair left {len(violations)} foreign-key violation(s); "
                "the database was not modified")
        conn.commit()
    finally:
        conn.execute("PRAGMA legacy_alter_table=OFF")
        conn.execute("PRAGMA foreign_keys=ON")
    return broken


def migrate(conn: sqlite3.Connection) -> list[str]:
    """Bring a v8 database up to v9. Safe to run on every launch."""
    applied = []
    conn.executescript(MIGRATION_SQL)
    existing = {r[1] for r in conn.execute("PRAGMA table_info(aircraft)")}
    for col, coltype in NEW_AIRCRAFT_COLUMNS:
        if col not in existing:
            conn.execute(f"ALTER TABLE aircraft ADD COLUMN {col} {coltype}")
            applied.append(f"aircraft.{col}")
    ft_existing = {r[1] for r in conn.execute("PRAGMA table_info(flight_tests)")}
    for col, coltype in NEW_FLIGHT_TEST_COLUMNS:
        if col not in ft_existing:
            conn.execute(f"ALTER TABLE flight_tests ADD COLUMN {col} {coltype}")
            applied.append(f"flight_tests.{col}")
    pf_existing = {r[1] for r in conn.execute("PRAGMA table_info(parameter_files)")}
    for col, coltype in NEW_PARAM_FILE_COLUMNS:
        if col not in pf_existing:
            conn.execute(f"ALTER TABLE parameter_files ADD COLUMN {col} {coltype}")
            applied.append(f"parameter_files.{col}")
    conn.commit()
    for table in repair_cascades(conn):
        applied.append(f"{table}: restored ON DELETE CASCADE")
    return applied


# ==========================================================================
# Text -> number extraction
# ==========================================================================
def _num(text: Any) -> float | None:
    if text is None:
        return None
    try:
        v = float(str(text).strip())
        return v
    except (TypeError, ValueError):
        return None


def parse_kv(specs: str) -> float | None:
    """'Stator 2207, 1700 KV, Qty 4' -> 1700.0"""
    if not specs:
        return None
    m = re.search(r"([\d.]+)\s*KV\b", specs, re.I) or \
        re.search(r"\bKV\s*[:=]?\s*([\d.]+)", specs, re.I)
    return _num(m.group(1)) if m else None


def parse_qty(specs: str) -> int | None:
    if not specs:
        return None
    m = re.search(r"\bQty\s*[:=]?\s*(\d+)", specs, re.I)
    return int(m.group(1)) if m else None


def parse_cells(text: str) -> int | None:
    """Find a '6S' style cell count in free text."""
    if not text:
        return None
    m = re.search(r"\b(\d{1,2})\s*S\b", text, re.I)
    if m:
        cells = int(m.group(1))
        if 1 <= cells <= 14:
            return cells
    return None


def parse_frame_size(frame_size: str) -> tuple[float | None, float | None]:
    """Read ATLAS's free-text frame_size as (prop_size_in, wheelbase_mm).

    Handles '5 inch', '5"', '5in', '220mm', '220 mm', and bare '5' / '220'
    (disambiguated by magnitude, since no prop is 220 inches and no useful
    multirotor has a 5 mm wheelbase).
    """
    if not frame_size:
        return None, None
    s = str(frame_size).strip().lower()

    mm = re.search(r"([\d.]+)\s*mm\b", s)
    if mm:
        return None, _num(mm.group(1))

    inch = re.search(r'([\d.]+)\s*(?:inch|inches|in\b|")', s)
    if inch:
        return _num(inch.group(1)), None

    bare = _num(re.sub(r"[^\d.]", "", s) or None)
    if bare is None:
        return None, None
    if bare <= 30:          # inches
        return bare, None
    return None, bare       # millimetres


# ==========================================================================
# Feature extraction
# ==========================================================================
def extract_features(conn: sqlite3.Connection,
                     aircraft_id: int) -> tuple[dict[str, float], list[str]]:
    """Build the model's feature vector for one aircraft.

    Returns (features, provenance) where provenance explains where each value
    came from — recorded, parsed from text, or inferred. The GUI shows this so
    nobody mistakes an inferred wheelbase for a measured one.
    """
    a = conn.execute("SELECT * FROM aircraft WHERE id=?", (aircraft_id,)).fetchone()
    if a is None:
        raise ValueError(f"no aircraft with id {aircraft_id}")
    comps = conn.execute(
        "SELECT * FROM components WHERE aircraft_id=?", (aircraft_id,)).fetchall()

    keys = a.keys() if hasattr(a, "keys") else []
    feats: dict[str, float] = {}
    prov: list[str] = []

    # --- recorded columns (authoritative) --------------------------------
    for col in ("weight_g", "prop_size_in", "cell_count", "wheelbase_mm"):
        if col in keys:
            v = _num(a[col])
            if v is not None:
                feats[col] = v
                prov.append(f"{col}: recorded")

    # --- motor specs ------------------------------------------------------
    motor_specs = " ".join(
        str(x["specs"] or "") for x in comps if x["category"] == "Motor")
    kv = parse_kv(motor_specs)
    if kv is not None:
        feats["motor_kv"] = kv
        prov.append("motor_kv: parsed from motor specs")

    qty = parse_qty(motor_specs)
    if qty is not None:
        feats["motor_qty"] = float(qty)
        prov.append("motor_qty: parsed from motor specs")
    else:
        frame = str(a["frame_type"] or "").strip().lower()
        if frame in FRAME_MOTOR_COUNT:
            feats["motor_qty"] = float(FRAME_MOTOR_COUNT[frame])
            prov.append(f"motor_qty: inferred from frame type ({frame})")

    # --- frame size -------------------------------------------------------
    prop_fs, wheel_fs = parse_frame_size(a["frame_size"])
    if "prop_size_in" not in feats and prop_fs is not None:
        feats["prop_size_in"] = prop_fs
        prov.append("prop_size_in: parsed from frame size")
    if "wheelbase_mm" not in feats and wheel_fs is not None:
        feats["wheelbase_mm"] = wheel_fs
        prov.append("wheelbase_mm: parsed from frame size")

    # --- cells from any free text ----------------------------------------
    if "cell_count" not in feats:
        blob = " ".join([str(a["notes"] or "")] +
                        [str(x["specs"] or "") for x in comps])
        cells = parse_cells(blob)
        if cells is not None:
            feats["cell_count"] = float(cells)
            prov.append("cell_count: parsed from notes/specs")

    # --- last resort: wheelbase from prop size ----------------------------
    if "wheelbase_mm" not in feats and "prop_size_in" in feats:
        p = feats["prop_size_in"]
        nearest = min(PROP_TO_WHEELBASE_MM, key=lambda k: abs(k - p))
        if abs(nearest - p) <= 0.75:
            feats["wheelbase_mm"] = float(PROP_TO_WHEELBASE_MM[nearest])
            prov.append(f"wheelbase_mm: INFERRED from {p:g}in prop "
                        "(typical value, not measured)")

    return feats, prov


def missing_features(feats: dict[str, float]) -> list[str]:
    return [f for f in FEATURE_FIELDS if f not in feats]


def feature_vector(feats: dict[str, float]) -> list[float] | None:
    """Dense vector in FEATURE_FIELDS order, or None if anything is missing."""
    if missing_features(feats):
        return None
    return [float(feats[f]) for f in FEATURE_FIELDS]


# ==========================================================================
# Target extraction
# ==========================================================================
def latest_param_file(conn: sqlite3.Connection, aircraft_id: int,
                      source: str | None = SOURCE_HARDWARE):
    """The most recent parameter snapshot for an aircraft.

    Defaults to hardware snapshots only. A simulated run files the tune it
    flew, so without this filter "the newest snapshot" silently becomes a
    SITL tune — which then gets exported as a validated candidate and used as
    a training target under a hardware label.
    """
    if source is None:
        return conn.execute(
            "SELECT * FROM parameter_files WHERE aircraft_id=? "
            "ORDER BY id DESC LIMIT 1", (aircraft_id,)).fetchone()
    return conn.execute(
        "SELECT * FROM parameter_files WHERE aircraft_id=? AND source=? "
        "ORDER BY id DESC LIMIT 1", (aircraft_id, source)).fetchone()


def extract_targets(conn: sqlite3.Connection, aircraft_id: int,
                    parameter_file_id: int | None = None
                    ) -> tuple[dict[str, float], list[str]]:
    """Pull the Acro tuning parameters out of a stored snapshot.

    Returns (targets, missing_names). ATLAS stores every parameter as text;
    this converts the handful that matter into numbers and reports which of
    them the snapshot did not contain.
    """
    if parameter_file_id is None:
        pf = latest_param_file(conn, aircraft_id)
        if pf is None:
            return {}, list(TARGET_PARAMS)
        parameter_file_id = pf["id"]

    rows = conn.execute(
        "SELECT name, value FROM parameters WHERE parameter_file_id=?",
        (parameter_file_id,)).fetchall()
    stored = {str(r["name"]).strip(): r["value"] for r in rows}

    targets, missing = {}, []
    for name in TARGET_PARAMS:
        v = _num(stored.get(name))
        if v is None:
            missing.append(name)
        else:
            targets[name] = v
    return targets, missing


def expand_derived(params: dict[str, float]) -> dict[str, float]:
    """Add the parameters that are conventionally tied to a predicted one."""
    out = dict(params)
    for derived, (source, mult) in DERIVED_PARAMS.items():
        if source in out:
            out[derived] = round(out[source] * mult, 8)
    return out


# ==========================================================================
# Outcome labels
# ==========================================================================
def outcome_label(conn: sqlite3.Connection,
                  aircraft_id: int) -> tuple[float | None, str]:
    """The training label for one aircraft, or None to EXCLUDE it.

    Precedence:
      1. an explicitly recorded acro_outcomes score (most trustworthy)
      2. the aircraft's validation status
      3. flight-test results, if the status is UNTESTED but flights exist
    Returns (label, explanation).
    """
    row = conn.execute(
        "SELECT AVG(outcome_score) AS s, COUNT(*) AS n FROM acro_outcomes "
        "WHERE aircraft_id=? AND outcome_score IS NOT NULL AND source=?",
        (aircraft_id, SOURCE_HARDWARE)).fetchone()
    if row and row["n"]:
        return float(row["s"]), f"mean of {row['n']} recorded flight score(s)"

    a = conn.execute("SELECT validation_status FROM aircraft WHERE id=?",
                     (aircraft_id,)).fetchone()
    status = (a["validation_status"] if a else "") or "UNTESTED"
    if status in VALIDATION_LABEL:
        return VALIDATION_LABEL[status], f"validation status {status}"

    # source='hardware' is load bearing. Without it a simulated flight filed as
    # evidence counts as a hardware pass, and an UNTESTED airframe that has only
    # ever flown in SITL enters the hardware training set with a perfect label.
    tests = conn.execute(
        "SELECT result, COUNT(*) AS n FROM flight_tests WHERE aircraft_id=? "
        "AND source=? GROUP BY result",
        (aircraft_id, SOURCE_HARDWARE)).fetchall()
    counts = {(r["result"] or "Not evaluated"): r["n"] for r in tests}
    passes, fails, mixed = counts.get("Pass", 0), counts.get("Fail", 0), counts.get("Mixed", 0)
    if passes or fails or mixed:
        total = passes + fails + mixed
        label = (1.0 * passes + 0.5 * mixed) / total
        return label, f"{passes} pass / {mixed} mixed / {fails} fail"

    return None, "UNTESTED with no evaluated flights — excluded from training"


# ==========================================================================
# Training set assembly
# ==========================================================================
def training_rows(conn: sqlite3.Connection,
                  min_label: float = 0.6,
                  include_sim: bool = False,
                  sim_min_score: float = 0.7) -> dict[str, Any]:
    """Assemble everything the model can learn from, and say what was skipped.

    Only aircraft that clear `min_label` contribute: the model learns what a
    GOOD configuration looks like for given hardware, so feeding it known-bad
    tunes as positive examples would be actively harmful.
    """
    X: list[list[float]] = []
    Y: list[list[float]] = []
    meta: list[dict[str, Any]] = []
    skipped: list[dict[str, str]] = []

    for a in conn.execute("SELECT id, name FROM aircraft ORDER BY id"):
        aid, name = a["id"], a["name"]

        label, why = outcome_label(conn, aid)
        if label is None:
            skipped.append({"name": name, "reason": why})
            continue
        if label < min_label:
            skipped.append({"name": name,
                            "reason": f"outcome {label:.2f} below {min_label:.2f} ({why})"})
            continue

        feats, _ = extract_features(conn, aid)
        vec = feature_vector(feats)
        if vec is None:
            skipped.append({"name": name,
                            "reason": "missing features: " +
                                      ", ".join(missing_features(feats))})
            continue

        targets, missing = extract_targets(conn, aid)
        if missing:
            skipped.append({"name": name,
                            "reason": "snapshot missing parameters: " +
                                      ", ".join(missing)})
            continue

        X.append(vec)
        Y.append([targets[p] for p in TARGET_PARAMS])
        meta.append({"aircraft_id": aid, "name": name, "label": label,
                     "label_reason": why, "source": SOURCE_HARDWARE})

    if include_sim:
        X2, Y2, meta2, skipped2 = _sim_training_rows(conn, sim_min_score)
        X += X2
        Y += Y2
        meta += meta2
        skipped += skipped2

    return {"X": X, "Y": Y, "meta": meta, "skipped": skipped,
            "features": list(FEATURE_FIELDS), "targets": list(TARGET_PARAMS)}


def _sim_training_rows(conn, sim_min_score):
    """Simulated runs, kept separate so their provenance is never lost.

    SITL results are not hardware results. They are included only on request
    and are tagged, so a model trained with them can be compared against one
    trained without.
    """
    X, Y, meta, skipped = [], [], [], []
    rows = conn.execute(
        "SELECT * FROM sim_runs WHERE aborted=0 AND aircraft_id IS NOT NULL "
        "AND outcome_score >= ? ORDER BY id", (sim_min_score,)).fetchall()
    for r in rows:
        feats, _ = extract_features(conn, r["aircraft_id"])
        vec = feature_vector(feats)
        if vec is None:
            skipped.append({"name": f"sim run #{r['id']}",
                            "reason": "aircraft missing features"})
            continue
        try:
            params = json.loads(r["params_json"])
        except (TypeError, ValueError):
            skipped.append({"name": f"sim run #{r['id']}",
                            "reason": "unreadable params_json"})
            continue
        missing = [p for p in TARGET_PARAMS if _num(params.get(p)) is None]
        if missing:
            skipped.append({"name": f"sim run #{r['id']}",
                            "reason": "missing parameters: " + ", ".join(missing)})
            continue
        X.append(vec)
        Y.append([float(params[p]) for p in TARGET_PARAMS])
        meta.append({"aircraft_id": r["aircraft_id"],
                     "name": f"sim run #{r['id']} ({r['run_group']})",
                     "label": r["outcome_score"],
                     "label_reason": "SITL score", "source": SOURCE_SIM})
    return X, Y, meta, skipped


def census(conn: sqlite3.Connection, min_label: float = 0.6) -> dict[str, Any]:
    """Counts for the GUI: how close is the repository to being trainable?"""
    total = conn.execute("SELECT COUNT(*) FROM aircraft").fetchone()[0]
    with_params = conn.execute(
        "SELECT COUNT(DISTINCT aircraft_id) FROM parameter_files").fetchone()[0]
    sim_ok = conn.execute(
        "SELECT COUNT(*) FROM sim_runs WHERE aborted=0").fetchone()[0]
    data = training_rows(conn, min_label=min_label)
    return {
        "aircraft_total": total,
        "aircraft_with_params": with_params,
        "trainable": len(data["X"]),
        "skipped": data["skipped"],
        "sim_runs": sim_ok,
    }


# ==========================================================================
# Write-back
# ==========================================================================
def record_outcome(conn, aircraft_id, outcome_score, *, flight_test_id=None,
                   tracking_error=None, vibration_max=None,
                   oscillation_detected=False, source=SOURCE_HARDWARE,
                   notes="") -> int:
    cur = conn.execute(
        "INSERT INTO acro_outcomes(aircraft_id, flight_test_id, outcome_score, "
        "tracking_error, vibration_max, oscillation_detected, source, notes, "
        "recorded_at) VALUES(?,?,?,?,?,?,?,?,?)",
        (aircraft_id, flight_test_id, outcome_score, tracking_error,
         vibration_max, int(bool(oscillation_detected)), source, notes,
         time.strftime("%Y-%m-%dT%H:%M:%S")))
    conn.commit()
    return cur.lastrowid


def record_application(conn, aircraft_id, connection, params, confirmed,
                       origin, operator_note="") -> int:
    """Audit trail: what was written to a flight controller, when, by whom."""
    cur = conn.execute(
        "INSERT INTO applied_params(aircraft_id, applied_utc, connection, "
        "params_json, confirmed_json, origin, operator_note) "
        "VALUES(?,?,?,?,?,?,?)",
        (aircraft_id, time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         connection, json.dumps(params, sort_keys=True),
         json.dumps(confirmed, sort_keys=True), origin, operator_note))
    conn.commit()
    return cur.lastrowid


def record_sim_run(conn, aircraft_id, run_group, params, score) -> int:
    cur = conn.execute(
        "INSERT INTO sim_runs(aircraft_id, run_group, started_utc, params_json, "
        "score_json, outcome_score, aborted, abort_reason) VALUES(?,?,?,?,?,?,?,?)",
        (aircraft_id, run_group,
         time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         json.dumps(params, sort_keys=True), json.dumps(score),
         score.get("outcome_score"), int(bool(score.get("aborted"))),
         score.get("abort_reason", "")))
    conn.commit()
    return cur.lastrowid


# ==========================================================================
# Validation helper used by every output path
# ==========================================================================
def validate_for_output(params: dict[str, float]) -> dict[str, Any]:
    """Run a proposed parameter set through safety.py and expand derivations.

    Every path that leaves this application — export, flight-controller write,
    simulation — goes through here, so there is exactly one definition of
    "allowed to leave".
    """
    expanded = expand_derived(params)
    report = safety.check_param_set(expanded)
    report["expanded"] = expanded
    report["derived_added"] = sorted(set(expanded) - set(params))
    return report
