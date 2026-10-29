#!/usr/bin/env python3
"""
import_build.py — add a build to the repository from a parameter file plus a
components list.

    python import_build.py --name "Lucid H7 QAV-R 5in" \
        --params  AP_FW_4_7_Lucid_H7.bin \
        --components 5in_Components.xlsx \
        --weight-g 680 --validation PARTIALLY_VALIDATED

It reads the parameter source, classifies the components, derives every
measurement it honestly can, and REFUSES to invent the ones it cannot. Run it
with --dry-run first: it prints exactly what it would store and what is still
missing, without touching the database.

PARAMETER SOURCES
-----------------
Handles both of the things people call a ".bin":

  * a CSV/text parameter export (NAME,value per line) — what Mission Planner
    writes when you save parameters, whatever extension it is given
  * a real ArduPilot dataflash log (binary, starts a3 95) — the PARM messages
    are read out of it with pymavlink

The distinction matters: parsing a binary log as text produces thousands of
plausible-looking garbage parameter names, and nothing downstream would catch
it. The file is sniffed rather than trusted by extension.

WHAT IT CANNOT DO
-----------------
A parameter file is not flight evidence. It records what a tune WAS, not how
it flew. This importer never sets a validation status on its own — you pass
one, based on what you know about the aircraft — and it never fabricates the
all-up weight, which no parameter file or parts list contains.
"""

from __future__ import annotations

import argparse
import csv
import io
import re
import shutil
import sqlite3
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))

import acro_bridge as bridge          # noqa: E402
import safety                        # noqa: E402

DB_PATH = APP_DIR / "atlas_repository.db"
FILES_DIR = APP_DIR / "repository_files"

#: ArduPilot FRAME_CLASS -> motor count.
FRAME_CLASS_MOTORS = {1: 4, 2: 6, 3: 8, 4: 8, 5: 6, 7: 3, 12: 12, 14: 10}
FRAME_CLASS_NAMES = {
    1: "Quadcopter", 2: "Hexacopter", 3: "Octocopter", 4: "Octocopter",
    5: "Hexacopter", 7: "Other", 12: "Other", 14: "Other",
}

#: Keyword -> ATLAS component category, checked in order.
CATEGORY_RULES = [
    (("flight controller", "flight-controller", " fc ", "h743", "pixhawk",
      "cube orange", "cube red", "lucid"), "Flight Controller"),
    (("esc", "4-in-1", "4in1"), "ESC"),
    (("motor",), "Motor"),
    (("frame kit", "frame", "chassis"), "Frame"),
    (("gnss", "gps", "compass", "m10q", "m8n", "m9n"), "GPS"),
    (("receiver", "elrs", "crossfire", "rx "), "Receiver"),
    (("vtx", "video", "avatar", "walksnail", "dji o3", "vista"), "VTX"),
]


# ==========================================================================
# Reading parameters
# ==========================================================================
def is_dataflash_log(path: Path) -> bool:
    """A real ArduPilot .bin begins with the log header bytes a3 95."""
    with open(path, "rb") as fh:
        return fh.read(2) == b"\xa3\x95"


def params_from_text(path: Path) -> dict[str, str]:
    raw = path.read_text(errors="ignore", newline="")
    out: dict[str, str] = {}
    # Try CSV first, then whitespace/tab separated, which Mission Planner and
    # MAVProxy both emit depending on how the file was saved.
    for row in csv.reader(io.StringIO(raw)):
        if len(row) >= 2 and row[0].strip():
            out[row[0].strip()] = row[1].strip()
    if len(out) > 5:
        return out
    out = {}
    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith(("#", "//")):
            continue
        parts = line.replace("\t", " ").split(None, 1)
        if len(parts) == 2:
            out[parts[0].strip()] = parts[1].strip()
    return out


def params_from_log(path: Path) -> dict[str, str]:
    """Pull PARM messages out of a binary dataflash log."""
    try:
        from pymavlink import mavutil
    except ImportError:
        raise RuntimeError(
            f"{path.name} is a binary dataflash log. Reading parameters out of "
            "it needs pymavlink:\n\n    pip install pymavlink\n\n"
            "Alternatively, open the log in Mission Planner and save the "
            "parameters to a .param file, then import that.")
    log = mavutil.mavlink_connection(str(path))
    out: dict[str, str] = {}
    while True:
        msg = log.recv_match(type="PARM")
        if msg is None:
            break
        name = getattr(msg, "Name", None)
        if name:
            out[str(name).strip()] = str(getattr(msg, "Value", ""))
    return out


def read_params(path: Path) -> tuple[dict[str, str], str]:
    """Returns (params, how_it_was_read)."""
    if is_dataflash_log(path):
        return params_from_log(path), "binary dataflash log (PARM messages)"
    params = params_from_text(path)
    if not params:
        raise ValueError(f"{path.name}: no parameters found. Expected either a "
                         "NAME,value text export or a binary dataflash log.")
    return params, "text/CSV parameter export"


# ==========================================================================
# Reading components
# ==========================================================================
def read_components(path: Path) -> list[dict[str, str]]:
    rows: list[list[str]] = []
    if path.suffix.lower() in (".xlsx", ".xlsm"):
        try:
            import openpyxl
        except ImportError:
            raise RuntimeError(
                "Reading .xlsx needs openpyxl:\n\n    pip install openpyxl\n\n"
                "Or save the sheet as .csv and pass that instead.")
        wb = openpyxl.load_workbook(path, data_only=True)
        for r in wb.active.iter_rows(values_only=True):
            rows.append([("" if c is None else str(c)).strip() for c in r])
    else:
        with open(path, newline="", errors="ignore") as fh:
            rows = [[c.strip() for c in r] for r in csv.reader(fh)]

    out = []
    for r in rows:
        if not r or not r[0]:
            continue
        name = r[0]
        if name.lower() in ("component", "part", "name", "item"):
            continue                      # header row
        link = next((c for c in r[1:] if c.startswith("http")), "")
        # Retailer links arrive carrying tracking parameters — gclid, utm_*,
        # campaign ids — that are longer than the product name and useless
        # here. Keep the product URL, drop the query string.
        if "?" in link:
            link = link.split("?", 1)[0]
        note = next((c for c in r[1:] if c and not c.startswith("http")), "")
        out.append({"name": name, "link": link, "note": note})
    return out


def classify(name: str) -> str:
    low = f" {name.lower()} "
    for keys, category in CATEGORY_RULES:
        if any(k in low for k in keys):
            return category
    return "Payload"


KNOWN_MAKES = ["TBS", "Team BlackSheep", "Hobbywing", "Lumenier", "T-Motor",
               "iFlight", "BrotherHobby", "Holybro", "CubePilot", "Matek",
               "MATEKSYS", "RadioMaster", "Walksnail", "Caddx", "Foxeer",
               "SpeedyBee", "Axisflying", "Diatone", "GEPRC", "Rush", "Emax",
               "ArduPilot", "mRo", "CUAV", "Gemfan", "HQProp", "Tattu"]

#: Retailer domains carry no brand information; manufacturer sites do.
DOMAIN_MAKES = {
    "lumenier.com": "Lumenier", "tmotor.com": "T-Motor",
    "holybro.com": "Holybro", "mateksys.com": "Matek",
    "team-blacksheep.com": "TBS", "radiomasterrc.com": "RadioMaster",
    "iflight-rc.com": "iFlight", "hobbywing.com": "Hobbywing",
    "caddxfpv.com": "Caddx", "foxeer.com": "Foxeer",
    "speedybee.com": "SpeedyBee", "emaxmodel.com": "Emax",
}
RETAILER_DOMAINS = ("getfpv.com", "amazon.", "ebay.", "aliexpress.",
                    "racedayquads.com", "pyrodrone.com")


def split_make_model(name: str, category: str = "",
                     link: str = "", note: str = "") -> tuple[str, str]:
    """Best-effort manufacturer/model split. The full name is kept regardless.

    Order of trust: an explicit brand prefix in the name, then the note column,
    then a manufacturer's own domain in the link. Retailer domains are ignored
    — getfpv.com tells you nothing about who made the part.
    """
    for mk in KNOWN_MAKES:
        if name.lower().startswith(mk.lower()):
            return mk, name[len(mk):].strip(" -\u2013")
    for mk in KNOWN_MAKES:
        if mk.lower() in (note or "").lower():
            return mk, name
    low = (link or "").lower()
    if link and not any(d in low for d in RETAILER_DOMAINS):
        for domain, mk in DOMAIN_MAKES.items():
            if domain in low:
                return mk, name
    # No brand identified. Leave the manufacturer blank rather than promoting
    # a model number into it — "M10Q-5883" is not a manufacturer.
    return "", name


# ==========================================================================
# Deriving measurements
# ==========================================================================
def derive(components: list[dict[str, str]],
           params: dict[str, str]) -> tuple[dict[str, Any], list[str]]:
    """Work out what can be known, and say where each value came from."""
    found: dict[str, Any] = {}
    prov: list[str] = []
    blob = " | ".join(c["name"] for c in components)

    # motor KV and stator, from the motor's own product name
    motor = next((c["name"] for c in components
                  if classify(c["name"]) == "Motor"), "")
    if motor:
        m = re.search(r"([\d.]+)\s*KV\b", motor, re.I)
        if m:
            found["motor_kv"] = float(m.group(1))
            prov.append(f"motor_kv = {m.group(1)} — from the motor name")
        st = re.search(r"\b(\d{4}(?:\.\d)?)\b", motor)
        if st:
            found["motor_stator"] = st.group(1)
            prov.append(f"motor stator {st.group(1)} — from the motor name")

    # motor count, from the autopilot's own frame class
    fc = params.get("FRAME_CLASS")
    if fc and fc.strip().replace(".0", "").isdigit():
        cls = int(float(fc))
        if cls in FRAME_CLASS_MOTORS:
            found["motor_qty"] = FRAME_CLASS_MOTORS[cls]
            found["frame_type"] = FRAME_CLASS_NAMES.get(cls, "Other")
            prov.append(f"motor_qty = {FRAME_CLASS_MOTORS[cls]} and frame type "
                        f"{found['frame_type']} — from FRAME_CLASS={cls}")

    # prop size, from the frame's product name
    frame = next((c["name"] for c in components
                  if classify(c["name"]) == "Frame"), "")
    m = re.search(r'([\d.]+)\s*(?:"|inch|in\b)', frame or blob)
    if m:
        found["prop_size_in"] = float(m.group(1))
        prov.append(f"prop_size_in = {m.group(1)} — from "
                    f"{'the frame name' if frame else 'the parts list'}")

    # cell count, from the ESC's rating, corroborated against the arm voltage
    esc = next((c["name"] for c in components
                if classify(c["name"]) == "ESC"), "")
    m = re.search(r"\b(\d{1,2})\s*S\b", esc or blob, re.I)
    if m:
        cells = int(m.group(1))
        if 1 <= cells <= 14:
            found["cell_count"] = cells
            note = f"cell_count = {cells} — from the ESC rating"
            av = params.get("BATT_ARM_VOLT")
            try:
                per = float(av) / cells
                if 2.8 <= per <= 4.3:
                    note += (f", consistent with BATT_ARM_VOLT {av} "
                             f"({per:.2f} V per cell)")
                else:
                    note += (f" — BUT BATT_ARM_VOLT {av} gives {per:.2f} V per "
                             "cell, which is out of range. Check this.")
            except (TypeError, ValueError, ZeroDivisionError):
                pass
            prov.append(note)
    return found, prov


# ==========================================================================
# Consistency checks worth surfacing
# ==========================================================================
def cross_check(components, params) -> list[str]:
    """Disagreements between the parts list and the configuration."""
    notes = []
    cats = {classify(c["name"]) for c in components}

    gps_listed = "GPS" in cats
    gps_type = (params.get("GPS1_TYPE") or params.get("GPS_TYPE") or "").strip()
    posxy = (params.get("EK3_SRC1_POSXY") or "").strip()
    if gps_listed and gps_type in ("0", "0.0"):
        notes.append(
            "The parts list includes a GPS/compass module, but this "
            "configuration has GPS1_TYPE=0"
            + (f" and EK3_SRC1_POSXY={posxy}" if posxy in ("0", "0.0") else "")
            + " — GPS is disabled in this tune. Either the module is fitted "
              "but unused, or the parts list and the configuration are from "
              "different builds.")

    if "Motor" not in cats:
        notes.append("No motor found in the parts list — motor KV cannot be "
                     "derived, and KV is one of the two features a prediction "
                     "cannot be made without.")
    if "Frame" not in cats:
        notes.append("No frame found in the parts list — prop size may not be "
                     "derivable.")

    ft = params.get("FRAME_TYPE", "").strip()
    if ft in ("18", "18.0"):
        notes.append("FRAME_TYPE=18 is BetaFlightX reversed — motor order and "
                     "direction differ from the ArduPilot default. Worth "
                     "recording, since it affects nothing in the tuning but "
                     "everything in the wiring.")
    return notes


# ==========================================================================
# The reusable hardware catalog
# ==========================================================================
def catalog_sync(conn: sqlite3.Connection,
                 components: list[dict[str, str]]) -> tuple[int, int]:
    """Add these parts to the reusable catalog if they are not already in it.

    The catalog is what the Add Aircraft dropdowns offer, so a part that was
    only ever written as a per-aircraft component has to be retyped by hand on
    the next build. UNIQUE(category, manufacturer, model) makes this safe to
    run repeatedly.
    """
    added = already = 0
    now = datetime.now().isoformat(timespec="seconds")
    for c in components:
        cat = classify(c["name"])
        make, model = split_make_model(c["name"], cat, c["link"], c["note"])
        if not model:
            continue
        exists = conn.execute(
            "SELECT 1 FROM hardware_catalog WHERE category=? AND manufacturer=? "
            "AND model=?", (cat, make, model)).fetchone()
        if exists:
            already += 1
            continue
        conn.execute(
            "INSERT INTO hardware_catalog(category, manufacturer, model, specs, "
            "created_at) VALUES(?,?,?,?,?)",
            (cat, make, model, c["link"] or c["note"] or "", now))
        added += 1
    conn.commit()
    return added, already


def catalog_sync_from_aircraft(conn: sqlite3.Connection) -> tuple[int, int]:
    """Catalog every part already recorded against an aircraft.

    Covers builds added before the catalog existed, or typed in by hand.
    """
    rows = conn.execute(
        "SELECT DISTINCT category, manufacturer, model, specs FROM components "
        "WHERE model IS NOT NULL AND TRIM(model) <> ''").fetchall()
    added = already = 0
    now = datetime.now().isoformat(timespec="seconds")
    for r in rows:
        cat = r["category"] or "Payload"
        make = (r["manufacturer"] or "").strip()
        model = (r["model"] or "").strip()
        if conn.execute("SELECT 1 FROM hardware_catalog WHERE category=? AND "
                        "manufacturer=? AND model=?", (cat, make, model)).fetchone():
            already += 1
            continue
        # Carry only a link across, never the derived specs — "Qty 4" belongs
        # to one build, not to the part.
        specs = ""
        for piece in (r["specs"] or "").split(", "):
            if piece.startswith("http"):
                specs = piece
                break
        conn.execute(
            "INSERT INTO hardware_catalog(category, manufacturer, model, specs, "
            "created_at) VALUES(?,?,?,?,?)", (cat, make, model, specs, now))
        added += 1
    conn.commit()
    return added, already


# ==========================================================================
# Writing to the repository
# ==========================================================================
def import_build(args) -> int:
    pp, cp = Path(args.params), Path(args.components) if args.components else None
    if not pp.exists():
        print(f"error: {pp} not found", file=sys.stderr)
        return 2

    params, how = read_params(pp)
    print(f"Parameters: {len(params)} read from {pp.name}")
    print(f"  source type: {how}")

    components = read_components(cp) if cp else []
    if cp:
        print(f"\nComponents: {len(components)} read from {cp.name}")
        for c in components:
            cat = classify(c["name"])
            print(f"  [{cat:<18}] {c['name']}")

    found, prov = derive(components, params)
    print("\nDerived from the files:")
    for line in prov:
        print(f"  {line}")

    # Command-line values override anything derived.
    supplied = {}
    for key, val in (("weight_g", args.weight_g), ("prop_size_in", args.prop_size_in),
                     ("cell_count", args.cell_count), ("wheelbase_mm", args.wheelbase_mm)):
        if val is not None:
            supplied[key] = val
            found[key] = val
            prov.append(f"{key} = {val} — supplied on the command line")
    if supplied:
        print("\nSupplied by you:")
        for k, v in supplied.items():
            print(f"  {k} = {v}")

    notes = cross_check(components, params)
    if notes:
        print("\nWorth checking:")
        for n in notes:
            print(f"  ! {n}")

    # Which Acro targets are present
    targets = {n: params[n] for n in bridge.TARGET_PARAMS if n in params}
    missing_targets = [n for n in bridge.TARGET_PARAMS if n not in params]
    print(f"\nAcro tuning parameters: {len(targets)}/{len(bridge.TARGET_PARAMS)} present")
    for n, v in targets.items():
        chk = safety.check_param(n, v)
        flag = "" if chk["verdict"] == safety.OK else f"   <-- {chk['message']}"
        print(f"  {n:<18} {v}{flag}")
    if missing_targets:
        print(f"  missing: {', '.join(missing_targets)}")
        print("  This build cannot be a training example without all of them.")

    # Which model features are still unknown
    feat_keys = ["motor_kv", "motor_qty", "prop_size_in", "weight_g",
                 "cell_count", "wheelbase_mm"]
    have = {k: found.get(k) for k in feat_keys}
    still_missing = [k for k in feat_keys if have[k] is None]
    print("\nModel features:")
    for k in feat_keys:
        v = have[k]
        print(f"  {bridge.FEATURE_LABELS[k]:<20} "
              f"{(f'{v:g}' if isinstance(v, (int, float)) else 'NOT KNOWN')}")
    if still_missing:
        print(f"\n  {len(still_missing)} feature(s) missing: "
              + ", ".join(bridge.FEATURE_LABELS[k] for k in still_missing))
        print("  The aircraft will store and will work for case retrieval, but")
        print("  cannot train or receive a prediction until these are filled in.")
        for k in still_missing:
            print(f"    --{k.replace('_', '-')} <value>")

    if args.dry_run:
        print("\n(dry run — nothing was written)")
        return 0

    FILES_DIR.mkdir(exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    bridge.migrate(conn)

    frame_type = args.frame_type or found.get("frame_type") or "Quadcopter"
    frame_size = args.frame_size or (
        f"{found['prop_size_in']:g} inch" if found.get("prop_size_in") else "")
    try:
        aid = conn.execute(
            "INSERT INTO aircraft(name, frame_type, frame_size, validation_status, "
            "notes, created_at, weight_g, prop_size_in, cell_count, wheelbase_mm) "
            "VALUES(?,?,?,?,?,?,?,?,?,?)",
            (args.name, frame_type, frame_size, args.validation,
             (args.notes or "") + ("\n\n" if args.notes else "")
             + "Imported by import_build.py\n"
             + f"Parameter source: {pp.name} ({how})\n"
             + (f"Components: {cp.name}\n" if cp else "")
             + ("\n".join("! " + n for n in notes) if notes else ""),
             datetime.now().isoformat(timespec="seconds"),
             found.get("weight_g"), found.get("prop_size_in"),
             int(found["cell_count"]) if found.get("cell_count") else None,
             found.get("wheelbase_mm"))).lastrowid
    except sqlite3.IntegrityError:
        print(f"\nerror: an aircraft named {args.name!r} already exists. "
              "Use a different --name.", file=sys.stderr)
        return 3

    for c in components:
        cat = classify(c["name"])
        make, model = split_make_model(c["name"], cat, c["link"], c["note"])
        specs = []
        if cat == "Motor":
            if found.get("motor_kv"):
                specs.append(f"{found['motor_kv']:g} KV")
            if found.get("motor_stator"):
                specs.append(f"Stator {found['motor_stator']}")
            if found.get("motor_qty"):
                specs.append(f"Qty {found['motor_qty']}")
        if c["link"]:
            specs.append(c["link"])
        conn.execute(
            "INSERT INTO components(aircraft_id, category, manufacturer, model, specs)"
            " VALUES(?,?,?,?,?)",
            (aid, cat, make, model, ", ".join(specs)))

    if not args.no_catalog:
        added, already = catalog_sync(conn, components)
        print(f"\nHardware catalog: {added} new entry(s)"
              + (f", {already} already present" if already else ""))

    stored = FILES_DIR / f"{aid}_{pp.name}"
    shutil.copy2(pp, stored)
    pf = conn.execute(
        "INSERT INTO parameter_files(aircraft_id, original_name, stored_path, "
        "ardupilot_version, imported_at, source) VALUES(?,?,?,?,?,'hardware')",
        (aid, pp.name, str(stored),
         args.firmware or params.get("FORMAT_VERSION", ""),
         datetime.now().isoformat(timespec="seconds"))).lastrowid
    conn.executemany(
        "INSERT INTO parameters(parameter_file_id, name, value) VALUES(?,?,?)",
        [(pf, k, v) for k, v in params.items()])
    conn.commit()

    feats, fprov = bridge.extract_features(conn, aid)
    label, why = bridge.outcome_label(conn, aid)
    cen = bridge.census(conn)
    print(f"\nStored aircraft #{aid} — {args.name}")
    print(f"  components: {len(components)}   parameters: {len(params)}")
    print(f"  features read back: {len(feats)}/6"
          + (f"   missing {', '.join(bridge.missing_features(feats))}"
             if bridge.missing_features(feats) else ""))
    for line in fprov:
        if "INFERRED" in line:
            print(f"  ! {line}")
    print(f"  training label: {'excluded' if label is None else f'{label:.2f}'} — {why}")
    print(f"\nRepository now holds {cen['aircraft_total']} aircraft, "
          f"{cen['trainable']} usable for training "
          f"(prediction needs {12}).")
    conn.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="aircraft name (must be unique)")
    ap.add_argument("--params", required=True,
                    help="parameter export or dataflash log")
    ap.add_argument("--components", help="components list (.xlsx or .csv)")
    ap.add_argument("--validation", default="UNTESTED",
                    choices=["UNTESTED", "PARTIALLY_VALIDATED",
                             "FULLY_VALIDATED", "REJECTED"],
                    help="what YOU know about how this build flies "
                         "(default: %(default)s). A parameter file is not "
                         "evidence, so this is never inferred.")
    ap.add_argument("--weight-g", type=float, help="all-up weight in grams")
    ap.add_argument("--prop-size-in", type=float)
    ap.add_argument("--cell-count", type=int)
    ap.add_argument("--wheelbase-mm", type=float)
    ap.add_argument("--frame-type", help="override the frame type")
    ap.add_argument("--frame-size", help="override the frame size text")
    ap.add_argument("--firmware", help="ArduPilot version string")
    ap.add_argument("--notes", help="free text stored with the aircraft")
    ap.add_argument("--no-catalog", action="store_true",
                    help="do not add these parts to the reusable hardware catalog")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be stored and change nothing")
    args = ap.parse_args(argv)
    try:
        return import_build(args)
    except (RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
