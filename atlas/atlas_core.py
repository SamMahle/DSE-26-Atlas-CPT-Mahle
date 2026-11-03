"""
atlas_core.py — the parts of ATLAS that have nothing to do with a window.

Paths, constants, the repository schema and the .param reader. Both
interfaces import this module: the NiceGUI interface (atlas_gui.py) and the
classic Tkinter one (atlas_v9.py). Keeping it free of any GUI import means the
new interface runs on a Python without Tkinter, and the two can never disagree
about where the repository lives or what its schema is.
"""

from __future__ import annotations

import os
import sqlite3
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import acro_bridge as bridge

APP_VERSION = "9.1 ACRO"

APP_DIR = Path(__file__).resolve().parent
DB_PATH = APP_DIR / "atlas_repository.db"
FILES_DIR = APP_DIR / "repository_files"
FILES_DIR.mkdir(exist_ok=True)
LOG_DIR = APP_DIR / "sim_logs"              # raw flight logs from simulated runs
LOG_DIR.mkdir(exist_ok=True)
BENCH_DIR = APP_DIR.parent / "sitl_bench"   # the standardized SITL test bench
BENCH_GROUP_PREFIX = "bench_"               # run_group prefix of bench runs in sim_runs

VALIDATION = ["UNTESTED", "PARTIALLY_VALIDATED", "FULLY_VALIDATED", "REJECTED"]
# Frame and GPS were added in v9: import_build.py classifies them, so the
# catalog has to be able to hold and offer them too.
CATEGORIES = ["Flight Controller", "ESC", "Motor", "Frame", "GPS", "Receiver", "VTX", "Payload"]
FRAME_TYPES = ["Quadcopter", "Hexacopter", "Octocopter", "Fixed Wing", "VTOL", "Other"]
TEST_RESULTS = ["Not evaluated", "Pass", "Fail", "Mixed"]
STARTER_HARDWARE = {
    "Flight Controller": {"Holybro": ["Pixhawk 6C", "Pixhawk 6X", "Durandal"],
                          "CubePilot": ["Cube Orange+", "Cube Red", "Cube Yellow"],
                          "Matek": ["H743-WING V3", "H743-SLIM", "F405-WTE"]},
    "ESC": {"Holybro": ["Tekko32 F4 45A", "Tekko32 F4 65A"],
            "T-Motor": ["F55A Pro II", "F60A Mini"],
            "Hobbywing": ["XRotor Micro 60A", "XRotor 40A"]},
    "Motor": {"T-Motor": ["F60 Pro V", "F90", "MN3110"],
              "BrotherHobby": ["Avenger 2806.5", "VY 2004"],
              "iFlight": ["XING2 2806.5", "XING2 2207"]},
}

PARAM_PATTERNS = ("*.param", "*.parm", "*.txt")
LOG_PATTERNS = ("*.bin", "*.log", "*.tlog")


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def db() -> sqlite3.Connection:
    """A new connection with named rows and foreign keys enforced.

    Use it as ``with db() as c:`` — the block commits on success and rolls
    back on an exception. Open a fresh one in every worker thread.
    """
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def init_db() -> None:
    with db() as c:
        c.executescript("""
        CREATE TABLE IF NOT EXISTS aircraft(
            id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT UNIQUE NOT NULL,
            frame_type TEXT, frame_size TEXT, validation_status TEXT DEFAULT 'UNTESTED',
            notes TEXT, created_at TEXT
        );
        CREATE TABLE IF NOT EXISTS components(
            id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
            category TEXT NOT NULL, manufacturer TEXT, model TEXT, specs TEXT,
            FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE
        );
        CREATE TABLE IF NOT EXISTS parameter_files(
            id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
            original_name TEXT, stored_path TEXT, ardupilot_version TEXT, imported_at TEXT,
            FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE
        );
        CREATE TABLE IF NOT EXISTS parameters(
            id INTEGER PRIMARY KEY AUTOINCREMENT, parameter_file_id INTEGER NOT NULL,
            name TEXT NOT NULL, value TEXT,
            FOREIGN KEY(parameter_file_id) REFERENCES parameter_files(id) ON DELETE CASCADE
        );
        CREATE TABLE IF NOT EXISTS flight_tests(
            id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER NOT NULL,
            log_name TEXT, stored_path TEXT, test_date TEXT, result TEXT, notes TEXT,
            FOREIGN KEY(aircraft_id) REFERENCES aircraft(id) ON DELETE CASCADE
        );
        CREATE TABLE IF NOT EXISTS hardware_catalog(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            category TEXT NOT NULL, manufacturer TEXT NOT NULL, model TEXT NOT NULL,
            specs TEXT DEFAULT '', active INTEGER NOT NULL DEFAULT 1,
            created_at TEXT, UNIQUE(category, manufacturer, model)
        );
        """)
        # v7 migration: tie each flight test to the exact parameter file used.
        flight_cols = {r[1] for r in c.execute("PRAGMA table_info(flight_tests)")}
        if "parameter_file_id" not in flight_cols:
            c.execute("ALTER TABLE flight_tests ADD COLUMN parameter_file_id INTEGER REFERENCES parameter_files(id)")
        # v9 migration: airframe measurements, outcome scores, sim runs, and
        # the audit trail of what was written to a flight controller.
        bridge.migrate(c)
        n = c.execute("SELECT COUNT(*) FROM hardware_catalog").fetchone()[0]
        if n == 0:
            now = now_iso()
            for cat, makes in STARTER_HARDWARE.items():
                for make, models in makes.items():
                    for model in models:
                        c.execute("INSERT OR IGNORE INTO hardware_catalog(category,manufacturer,model,created_at) "
                                  "VALUES(?,?,?,?)", (cat, make, model, now))


def parse_param(path) -> list[tuple[str, str]]:
    """(name, value) pairs from a text parameter file: comma, tab or space separated."""
    out = []
    for raw in Path(path).read_text(errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith(("#", "//")):
            continue
        if "," in line:
            a, b = line.split(",", 1)
        elif "\t" in line:
            a, b = line.split("\t", 1)
        elif " " in line:
            a, b = line.split(None, 1)
        else:
            continue
        a, b = a.strip(), b.strip()
        if a:
            out.append((a, b))
    return out


def open_in_file_browser(path) -> None:
    """Show a folder (or file) in the operating system's file browser. Raises OSError."""
    path = str(path)
    if sys.platform.startswith("win"):
        os.startfile(path)  # noqa: S606
    elif sys.platform == "darwin":
        subprocess.Popen(["open", path])
    else:
        subprocess.Popen(["xdg-open", path])
