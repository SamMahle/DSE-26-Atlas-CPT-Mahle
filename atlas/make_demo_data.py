#!/usr/bin/env python3
"""
make_demo_data.py — fill the repository with synthetic builds so the Predict
tab can be demonstrated before real flight data exists.

THE DATA THIS WRITES IS FAKE. Parameters come from a made-up formula plus
noise, not from any aircraft that ever flew. It exists so you can see the
prediction path work end to end and so the interface is not empty in a
walkthrough. Delete these aircraft before drawing any conclusion from a
prediction.

    python make_demo_data.py            # add 15 synthetic builds
    python make_demo_data.py --remove   # delete them again
"""
import argparse
import random
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import acro_bridge as bridge  # noqa: E402

DB = Path(__file__).resolve().parent / "atlas_repository.db"
PREFIX = "DEMO-"


def connect():
    conn = sqlite3.connect(DB)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    bridge.migrate(conn)
    return conn


def synth_params(prop_in, weight_g, rng):
    """A plausible-looking relationship, NOT a physical model."""
    return {
        "ATC_RAT_RLL_P": round(0.02 + 0.00022 * weight_g + rng.gauss(0, 0.004), 6),
        "ATC_RAT_RLL_D": round(0.0008 + 0.0000055 * weight_g + rng.gauss(0, 0.0002), 7),
        "ATC_RAT_PIT_P": round(0.02 + 0.00022 * weight_g + rng.gauss(0, 0.004), 6),
        "ATC_RAT_PIT_D": round(0.0008 + 0.0000055 * weight_g + rng.gauss(0, 0.0002), 7),
        "ATC_RAT_YAW_P": round(0.18 + 0.00008 * weight_g + rng.gauss(0, 0.01), 6),
        "ACRO_RP_RATE": round(560 - prop_in * 40 + rng.gauss(0, 15), 1),
        "ACRO_Y_RATE": round(300 + rng.gauss(0, 20), 1),
        "INS_GYRO_FILTER": round(max(20.0, 120 - prop_in * 6), 1),
    }


def add_aircraft(conn, name, *, prop_in, kv, cells, weight_g, wheelbase_mm,
                 status, params, motor_qty=4):
    aid = conn.execute(
        "INSERT INTO aircraft(name,frame_type,frame_size,validation_status,notes,"
        "created_at,weight_g,prop_size_in,cell_count,wheelbase_mm) "
        "VALUES(?,?,?,?,?,datetime('now'),?,?,?,?)",
        (name, "Quadcopter", f"{prop_in:g} inch", status,
         "Synthetic demonstration record — not a real aircraft.",
         weight_g, prop_in, cells, wheelbase_mm)).lastrowid
    for cat, make, model, specs in (
        ("Flight Controller", "Holybro", "Pixhawk 6C", ""),
        ("ESC", "T-Motor", "F55A Pro II", ""),
        ("Motor", "T-Motor", "F60 Pro V", f"Stator 2207, {kv:g} KV, Qty {motor_qty}"),
    ):
        conn.execute("INSERT INTO components(aircraft_id,category,manufacturer,model,specs)"
                     " VALUES(?,?,?,?,?)", (aid, cat, make, model, specs))
    pf = conn.execute(
        "INSERT INTO parameter_files(aircraft_id,original_name,stored_path,"
        "ardupilot_version,imported_at) VALUES(?,?,?,?,datetime('now'))",
        (aid, f"{name}.param", "", "4.7.0-beta7")).lastrowid
    conn.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",
                     [(pf, k, str(v)) for k, v in params.items()])
    return aid


def populate(n=15, seed=11):
    rng = random.Random(seed)
    conn = connect()
    existing = conn.execute("SELECT COUNT(*) FROM aircraft WHERE name LIKE ?",
                            (PREFIX + "%",)).fetchone()[0]
    if existing:
        print(f"{existing} demo aircraft already present. Run with --remove first.")
        return
    for i in range(n):
        prop = rng.choice([3.0, 4.0, 5.0, 6.0, 7.0])
        weight = round(prop * 120 + rng.uniform(-50, 50))
        kv = round(3200 - prop * 280 + rng.uniform(-120, 120))
        add_aircraft(conn, f"{PREFIX}Build-{i:02d}",
                     prop_in=prop, kv=kv, cells=4 if prop <= 4 else 6,
                     weight_g=weight,
                     wheelbase_mm=bridge.PROP_TO_WHEELBASE_MM.get(prop),
                     status="FULLY_VALIDATED",
                     params=synth_params(prop, weight, rng))

    # One real configuration, from the TBS Lucid H7 parameter file. Marked
    # PARTIALLY_VALIDATED because that is what it actually is.
    add_aircraft(conn, f"{PREFIX}Lucid-H7-QAVR", prop_in=5.0, kv=1900, cells=6,
                 weight_g=640, wheelbase_mm=220, status="PARTIALLY_VALIDATED",
                 params={"ATC_RAT_RLL_P": 0.03205, "ATC_RAT_RLL_D": 0.001392785,
                         "ATC_RAT_PIT_P": 0.03205, "ATC_RAT_PIT_D": 0.00146504,
                         "ATC_RAT_YAW_P": 0.2290468, "ACRO_RP_RATE": 400,
                         "ACRO_Y_RATE": 300, "INS_GYRO_FILTER": 100})

    # One unflown rig, to show that UNTESTED is excluded rather than scored 0.
    add_aircraft(conn, f"{PREFIX}Untested-bench", prop_in=5.0, kv=1800, cells=6,
                 weight_g=600, wheelbase_mm=220, status="UNTESTED",
                 params={"ATC_RAT_RLL_P": 0.05})

    conn.commit()
    cen = bridge.census(conn)
    print(f"Added {n + 2} demo aircraft.")
    print(f"  usable for training: {cen['trainable']}")
    for s in cen["skipped"]:
        print(f"  excluded — {s['name']}: {s['reason']}")
    conn.close()


def remove():
    conn = connect()
    n = conn.execute("SELECT COUNT(*) FROM aircraft WHERE name LIKE ?",
                     (PREFIX + "%",)).fetchone()[0]
    conn.execute("DELETE FROM aircraft WHERE name LIKE ?", (PREFIX + "%",))
    conn.execute("DELETE FROM sim_runs WHERE run_group LIKE 'demo-%'")
    conn.commit()
    print(f"Removed {n} demo aircraft (components, parameter files and "
          "parameters cascaded).")
    conn.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--remove", action="store_true", help="delete the demo aircraft")
    ap.add_argument("--count", type=int, default=15, help="how many to add")
    args = ap.parse_args()
    remove() if args.remove else populate(args.count)
