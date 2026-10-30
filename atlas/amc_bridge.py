#!/usr/bin/env python3
"""
amc_bridge.py — hand ATLAS builds to ArduPilot Methodic Configurator (AMC),
and seed ATLAS from AMC's published vehicle templates.

WHY THIS EXISTS
---------------
ATLAS is the library: it stores builds, finds similar ones, and predicts Acro
tunes. AMC is the ArduPilot developers' own configurator: it walks a vehicle
through ~70 ordered configuration steps, derives parameters from the vehicle's
components, and checks every upload. Rebuilding AMC inside ATLAS would be
slower and less safe than using it, so ATLAS hands builds to it instead.

THE BOUNDARY
------------
The two programs exchange FILES ONLY. This module never imports AMC's Python
code. It reads AMC's published template folders (JSON and .param text) and
writes a project folder in AMC's own format:

    <project>/
        00_default.param … 70_everyday_use.param   copied from the AMC template
        vehicle_components.json                    built from the ATLAS record
        NN_imported_atlas_parameters.param         the ATLAS tune, as a delta
        atlas_handoff.json                         provenance and review report

`NN_imported_<source>_parameters.param` is the same convention AMC uses when it
imports a flight controller or a .bin log, so AMC treats the ATLAS tune exactly
as it treats its own imports. AMC is GPL-3; keeping to a file interface keeps
ATLAS a separate program. Importing AMC modules would change that.

WHAT IT REFUSES TO DO
---------------------
* Invent a component value. Anything ATLAS cannot source from its own record
  or the parameter snapshot keeps the template value and is listed for review.
* Carry one flight controller's sensor calibration onto another. Accelerometer,
  gyro, compass, barometer, level and radio calibrations are stripped unless
  the operator states the project is for the SAME flight controller.
* Export an Acro parameter outside ATLAS's approved range. The same
  fail-closed check in safety.py that guards every other exit applies here.
* Overwrite an existing project folder.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import re
import shutil
import sqlite3
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))

import acro_bridge as bridge  # noqa: E402
import safety  # noqa: E402

SETTINGS_PATH = APP_DIR / "atlas_settings.json"
FILES_DIR = APP_DIR / "repository_files"
HANDOFF_FORMAT = 1
IMPORT_SOURCE = "atlas"
TEMPLATE_ORIGIN_TAG = "Origin: AMC vehicle template"

# ---------------------------------------------------------------------------
# ArduPilot enumerations. These are ArduPilot's own parameter-value names, the
# same strings AMC's component editor offers, so the exported file validates
# without edits. Values outside these tables are never guessed: the field keeps
# its template value and is reported for review.
# ---------------------------------------------------------------------------

SERIAL_PORTS = tuple(f"SERIAL{i}" for i in range(1, 9))

#: SERIALn_PROTOCOL value -> (AMC component it belongs to, protocol name)
SERIAL_PROTOCOLS = {
    1: ("Telemetry", "MAVLink1"),
    2: ("Telemetry", "MAVLink2"),
    5: ("GNSS Receiver", "GPS"),
    16: ("ESC", "ESC Telemetry"),
    23: ("RC Receiver", "RCIN"),
    28: ("ESC", "Scripting"),
    38: ("ESC", "FETtecOneWire"),
    39: ("ESC", "Torqeedo"),
    41: ("ESC", "CoDevESC"),
    43: ("Telemetry", "MAVLink High Latency"),
    48: ("Telemetry", "PPP"),
}
ESC_TELEMETRY_ONLY = {"ESC Telemetry", "Scripting"}

#: MOT_PWM_TYPE value -> protocol name, per vehicle type (Rover numbers differ)
MOT_PWM_TYPES = {
    "ArduCopter": {0: "Normal", 1: "OneShot", 2: "OneShot125", 3: "Brushed",
                   4: "DShot150", 5: "DShot300", 6: "DShot600", 7: "DShot1200",
                   8: "PWMRange", 9: "PWMAngle"},
    "Rover": {0: "Normal", 1: "OneShot", 2: "OneShot125", 3: "BrushedWithRelay",
              4: "BrushedBiPolar", 5: "DShot150", 6: "DShot300", 7: "DShot600",
              8: "DShot1200", 9: "PWMRange", 10: "PWMAngle"},
}
MOT_PWM_TYPES["Heli"] = MOT_PWM_TYPES["ArduCopter"]
MOT_PWM_TYPES["ArduPlane"] = MOT_PWM_TYPES["ArduCopter"]
#: SERVOn_FUNCTION values that mean "motor output" on the AUX/AIO pins
ESC_SERVO_FUNCTIONS = {33, 34, 35, 36, 73, 74}

#: RC_PROTOCOLS bit value -> protocol name (1 means "All")
RC_PROTOCOLS = {1: "All", 2: "PPM", 4: "IBUS", 8: "SBUS", 16: "SBUS_NI", 32: "DSM",
                64: "SUMD", 128: "SRXL", 256: "SRXL2", 512: "CRSF", 1024: "ST24",
                2048: "FPORT", 4096: "FPORT2", 8192: "FastSBUS", 32768: "Ghost",
                65536: "MAVRadio"}

#: BATT_MONITOR value -> (connection type, protocol). I2C types resolve the bus
#: from BATT_I2C_BUS; anything not listed is left for review.
BATT_MONITORS = {
    0: ("None", "Disabled"),
    3: ("Analog", "Analog Voltage Only"),
    4: ("Analog", "Analog Voltage and Current"),
    7: ("I2C", "SMBus-Generic"),
    8: ("CAN1", "DroneCAN-BatteryInfo"),
    9: ("other", "ESC"),
    10: ("other", "Sum Of Selected Monitors"),
    21: ("I2C", "INA2XX"),
    25: ("Analog", "Synthetic Current and Analog Voltage"),
    29: ("other", "Scripting"),
}

#: GPS1_TYPE / GPS_TYPE value -> (bus kind, protocol)
GNSS_TYPES = {
    0: ("None", "None"), 1: ("serial", "AUTO"), 2: ("serial", "uBlox"),
    5: ("serial", "NMEA"), 9: ("can", "DroneCAN"), 10: ("serial", "Septentrio(SBF)"),
    11: ("serial", "Trimble(GSOF)"), 14: ("serial", "MAVLink"), 15: ("serial", "NOVA"),
    19: ("serial", "MSP"), 24: ("serial", "UnicoreNMEA"),
}

FRAME_CLASSES = {
    "ArduCopter": {0: "Undefined", 1: "Quad", 2: "Hexa", 3: "Octa", 4: "OctaQuad",
                   5: "Y6", 6: "Heli", 7: "Tri", 8: "SingleCopter", 9: "CoaxCopter",
                   10: "BiCopter", 11: "Heli_Dual", 12: "DodecaHexa", 13: "HeliQuad",
                   14: "Deca", 15: "Scripting Matrix"},
}
#: ATLAS frame_type -> AMC frame class, used only when FRAME_CLASS is absent
FRAME_TYPE_TO_CLASS = {"Quadcopter": "Quad", "Hexacopter": "Hexa", "Octocopter": "Octa"}
#: AMC frame class -> ATLAS frame_type, for template import
CLASS_TO_FRAME_TYPE = {"Quad": "Quadcopter", "Hexa": "Hexacopter", "Octa": "Octocopter",
                       "OctaQuad": "Octocopter", "Y6": "Hexacopter"}
#: ATLAS frame_type -> AMC firmware/vehicle type
FRAME_TYPE_TO_VEHICLE = {"Quadcopter": "ArduCopter", "Hexacopter": "ArduCopter",
                         "Octocopter": "ArduCopter", "Fixed Wing": "ArduPlane",
                         "VTOL": "ArduPlane"}

#: Conservative per-cell voltages, used only when neither the build nor the
#: template states them. Always reported as inferred.
STANDARD_CELL_VOLTS = {
    "Lipo": {"Volt per cell max": 4.2, "Volt per cell arm": 3.8, "Volt per cell low": 3.6,
             "Volt per cell crit": 3.3, "Volt per cell min": 3.2},
    "LipoHV": {"Volt per cell max": 4.35, "Volt per cell arm": 3.9, "Volt per cell low": 3.6,
               "Volt per cell crit": 3.3, "Volt per cell min": 3.2},
    "LiIon": {"Volt per cell max": 4.2, "Volt per cell arm": 3.6, "Volt per cell low": 3.2,
              "Volt per cell crit": 3.0, "Volt per cell min": 2.9},
}

#: Battery chemistry inferred from the configured full-charge voltage per cell
CHEMISTRY_BY_MAX_CELL = [(4.10, "LiIon"), (4.20, "Lipo"), (4.35, "LipoHV"), (4.45, "LipoHVSS")]

# ---------------------------------------------------------------------------
# Parameters that belong to one physical flight controller, not to a design.
# Copying them to a different board is wrong in a way no check downstream will
# catch: the aircraft arms with another board's sensor offsets.
# ---------------------------------------------------------------------------
CALIBRATION_PATTERNS = [
    r"^INS_ACC\d?OFFS_[XYZ]$", r"^INS_ACC\d?SCAL_[XYZ]$", r"^INS_GYR\d?OFFS_[XYZ]$",
    r"^INS_TCAL\d_(?!ENABLE$).+",                       # temperature-cal results
    r"^COMPASS_(OFS|DIA|ODI|MOT|SCALE)\d?_?[XYZ]?$",    # compass offsets / iron / motor comp
    r"^COMPASS_(OFS|DIA|ODI|MOT)\d_[XYZ]$",
    r"^AHRS_TRIM_[XY]$",                                  # level calibration
    r"^BARO\d?_GND_PRESS$", r"^GND_ABS_PRESS\d?$",
    r"^RC\d{1,2}_(MIN|MAX|TRIM)$",                        # radio calibration
]
#: Never transfer, whatever the target: identifiers and counters.
NEVER_TRANSFER_PATTERNS = [
    r"^STAT_", r"^FORMAT_VERSION$", r"_DEV_?ID\d*$", r"^INS_(ACC|GYR)\d?_ID$",
    r"^COMPASS_PRIO\d_ID$", r"^BARO\d_DEVID$", r"^SYSID_SW_", r"^BRD_SERIAL_NUM$",
]
_CAL_RE = [re.compile(p) for p in CALIBRATION_PATTERNS]
_NEVER_RE = [re.compile(p) for p in NEVER_TRANSFER_PATTERNS]

#: Component fields AMC derives parameters from. Left at a template default,
#: they would overwrite the ATLAS tune with someone else's hardware choice.
CRITICAL_FIELDS = [
    ("Frame", "Specifications", "Frame class"),
    ("Battery", "Specifications", "Number of cells"),
    ("Battery", "Specifications", "Capacity mAh"),
    ("Battery Monitor", "FC Connection", "Protocol"),
    ("ESC", "FC->ESC Connection", "Protocol"),
    ("ESC", "ESC->FC Telemetry", "Protocol"),
    ("Motors", "Specifications", "Poles"),
    ("Propellers", "Specifications", "Diameter_inches"),
    ("RC Receiver", "FC Connection", "Protocol"),
    ("GNSS Receiver", "FC Connection", "Protocol"),
]

# ---------------------------------------------------------------------------
# Where AMC and the ATLAS tune disagree. Each disagreement is a "conflict" the
# operator can settle either way; the default rule is: a mandatory AMC value
# that is a SAFETY decision (failsafes, arming, fences, return-to-launch) goes
# to AMC; everything else goes to the build's validated tune. Every conflict
# and who settled it is recorded in the report.
# ---------------------------------------------------------------------------
AMC, ATLAS = "amc", "atlas"
SAFETY_PATTERNS = [r"^BATT\d?_FS_", r"^BATT\d?_(ARM|LOW|CRT)_", r"^FS_", r"^ARMING_", r"^FENCE_",
                   r"^RC_FS", r"^THR_FS", r"^GPS\d?_?FS", r"^EK[23]_.*FS", r"^DISARM_DELAY$",
                   r"^MOT_SAFE_", r"^BRD_SAFETY", r"^RTL_", r"^LAND_"]
_SAFETY_RE = [re.compile(x) for x in SAFETY_PATTERNS]


def is_safety_param(name: str) -> bool:
    return any(r.search(name) for r in _SAFETY_RE)


def make_conflict(name: str, atlas_value: Any, amc_value: Any, step: str, reason: str, kind: str,
                  overrides: dict[str, str] | None = None) -> dict[str, Any]:
    """One disagreement between AMC and the tune, settled by the operator or the default rule."""
    safety_ = is_safety_param(name)
    default = AMC if (kind == "forced" and safety_) else ATLAS
    choice = (overrides or {}).get(name)
    return {"parameter": name, "atlas": _fmt(atlas_value), "amc": _fmt(amc_value), "step": step,
            "reason": reason, "kind": kind, "safety": safety_, "default": default,
            "winner": choice if choice in (AMC, ATLAS) else default,
            "decided_by": "operator" if choice in (AMC, ATLAS) else "default rule"}


def conflict_warnings(conflicts: list[dict[str, Any]]) -> list[str]:
    """The conflicts an operator must not miss: AMC's value used, or a safety value overruled."""
    out = []
    for c in conflicts:
        label = "mandatory" if c["kind"] == "forced" else "calculated"
        if c["winner"] == AMC:
            out.append(f"AMC value used: {c['parameter']} = {c['amc']} (AMC {label}, step {c['step']}: "
                       f"{c['reason'] or 'no reason given'}). The tune had {c['atlas']}.")
        elif c["safety"]:
            out.append(f"SAFETY VALUE OVERRULED: {c['parameter']} = {c['atlas']} from the tune, instead of AMC's "
                       f"{label} {c['amc']} (step {c['step']}: {c['reason'] or 'no reason given'}). "
                       f"Decided by {c['decided_by']}.")
    return out


# Provenance labels shown in the GUI and written to atlas_handoff.json
SRC_RECORD = "ATLAS record"
SRC_PARAMS = "parameter snapshot"
SRC_INFERRED = "inferred"
SRC_TEMPLATE = "template default"


class BridgeError(RuntimeError):
    """A handoff that cannot proceed, with a reason fit to show an operator."""


# ===========================================================================
# Settings and locating AMC
# ===========================================================================

def load_settings() -> dict[str, Any]:
    try:
        return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_settings(settings: dict[str, Any]) -> None:
    SETTINGS_PATH.write_text(json.dumps(settings, indent=2), encoding="utf-8")


def _is_templates_dir(path: Path) -> bool:
    return path.is_dir() and any((path / v).is_dir() for v in ("ArduCopter", "ArduPlane", "Rover", "Heli"))


def resolve_templates_dir(path: str | Path) -> Path | None:
    """Accept an AMC install, source tree, or templates folder; return the templates folder."""
    p = Path(path).expanduser()
    candidates = [p, p / "vehicle_templates", p / "ardupilot_methodic_configurator" / "vehicle_templates",
                  p / "_internal" / "ardupilot_methodic_configurator" / "vehicle_templates"]
    for c in candidates:
        if _is_templates_dir(c):
            return c.resolve()
    return None


def find_templates_dir(settings: dict[str, Any] | None = None) -> Path | None:
    """Find AMC's vehicle templates without importing AMC.

    Order: the folder the operator chose, the Windows installer location, then
    a pip-installed package found by its spec (which does not execute it).
    """
    settings = load_settings() if settings is None else settings
    if settings.get("amc_templates_dir"):
        found = resolve_templates_dir(settings["amc_templates_dir"])
        if found:
            return found
    program_data = os.environ.get("PROGRAMDATA")
    if program_data:
        found = resolve_templates_dir(Path(program_data) / ".ardupilot_methodic_configurator")
        if found:
            return found
    spec = importlib.util.find_spec("ardupilot_methodic_configurator")
    if spec and spec.submodule_search_locations:
        for loc in spec.submodule_search_locations:
            found = resolve_templates_dir(loc)
            if found:
                return found
    return None


def find_amc_command(settings: dict[str, Any] | None = None) -> list[str] | None:
    """The command that starts AMC, or None if it is not installed."""
    settings = load_settings() if settings is None else settings
    exe = settings.get("amc_executable")
    if exe and Path(exe).is_file():
        return [exe]
    on_path = shutil.which("ardupilot_methodic_configurator")
    if on_path:
        return [on_path]
    for root in (os.environ.get("ProgramW6432"), os.environ.get("PROGRAMFILES")):
        if root:
            p = Path(root) / "ardupilot_methodic_configurator" / "ardupilot_methodic_configurator.exe"
            if p.is_file():
                return [str(p)]
    if importlib.util.find_spec("ardupilot_methodic_configurator"):
        return [sys.executable, "-m", "ardupilot_methodic_configurator"]
    return None


def launch_amc(project_dir: Path, settings: dict[str, Any] | None = None) -> list[str]:
    """Open AMC on an exported project. Returns the command used."""
    cmd = find_amc_command(settings)
    if not cmd:
        raise BridgeError(
            "ArduPilot Methodic Configurator was not found. Install it, or set its "
            "executable with 'Locate AMC…', then try again. The exported project is "
            f"complete without it:\n{project_dir}")
    full = [*cmd, "--vehicle-dir", str(project_dir)]
    kwargs: dict[str, Any] = {"cwd": str(project_dir)}
    if os.name == "nt":
        kwargs["creationflags"] = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    subprocess.Popen(full, **kwargs)  # noqa: S603 - fixed argument list, no shell
    return full


# ===========================================================================
# Parameter files
# ===========================================================================

def read_param_file(path: Path) -> dict[str, str]:
    """Read a Mission Planner or MAVProxy style .param file. Comments are dropped."""
    out: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or line.startswith("//"):
            continue
        if "," in line:
            name, value = line.split(",", 1)
        elif "\t" in line:
            name, value = line.split("\t", 1)
        elif " " in line:
            name, value = line.split(None, 1)
        else:
            continue
        name, value = name.strip(), value.strip()
        if name:
            out[name] = value
    return out


def _num(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int(value: Any) -> int | None:
    v = _num(value)
    return int(v) if v is not None and float(v).is_integer() else None


def _fmt(value: Any) -> str:
    v = _num(value)
    if v is None:
        return str(value)
    if v.is_integer():
        return str(int(v))
    return f"{v:.7g}"


def _same(a: Any, b: Any) -> bool:
    x, y = _num(a), _num(b)
    if x is None or y is None:
        return str(a).strip() == str(b).strip()
    return abs(x - y) <= 1e-6 * max(1.0, abs(x), abs(y))


def numbered_param_files(folder: Path) -> list[Path]:
    return sorted(p for p in folder.glob("*.param") if p.name[:2].isdigit())


def merged_template_params(folder: Path) -> dict[str, str]:
    """The parameter state a template reaches after all of its steps, in order."""
    merged: dict[str, str] = {}
    for f in numbered_param_files(folder):
        merged.update(read_param_file(f))
    return merged


def _templates_root(path: Path) -> Path | None:
    for parent in [path, *path.parents]:
        if _is_templates_dir(parent):
            return parent
    return None


def load_configuration_steps(vehicle_type: str, template_path: Path,
                             settings: dict[str, Any] | None = None) -> dict[str, Any] | None:
    """Read AMC's step definitions (a JSON data file shipped beside its templates)."""
    settings = load_settings() if settings is None else settings
    name = f"configuration_steps_{vehicle_type}.json"
    roots: list[Path] = []
    root = _templates_root(template_path)
    if root:
        roots.append(root.parent)
    exe = settings.get("amc_executable")
    if exe:
        roots.append(Path(exe).parent / "_internal" / "ardupilot_methodic_configurator")
    for env in ("ProgramW6432", "PROGRAMFILES"):
        if os.environ.get(env):
            roots.append(Path(os.environ[env]) / "ardupilot_methodic_configurator" / "_internal"
                         / "ardupilot_methodic_configurator")
    spec = importlib.util.find_spec("ardupilot_methodic_configurator")
    if spec and spec.submodule_search_locations:
        roots.extend(Path(p) for p in spec.submodule_search_locations)
    for r in roots:
        f = r / name
        if f.is_file():
            try:
                return json.loads(f.read_text(encoding="utf-8")).get("steps", {})
            except (OSError, ValueError):
                continue
    return None


def _condition_holds(condition: str, fc_params: dict[str, str]) -> bool:
    """Evaluate the two condition forms AMC uses on forced values; unknown forms count as true."""
    m = re.fullmatch(r"\s*'(\w+)'\s+in\s+fc_parameters\s*", condition)
    if m:
        return m.group(1) in fc_params
    m = re.fullmatch(r"\s*(?:'\w+'\s+in\s+fc_parameters\s+and\s+)?fc_parameters\['(\w+)'\]\s*>\s*([\d.]+)\s*", condition)
    if m:
        v = _num(fc_params.get(m.group(1)))
        return v is not None and v > float(m.group(2))
    return True


def amc_end_state(template_path: Path, steps: dict[str, Any] | None,
                  fc_params: dict[str, str] | None = None
                  ) -> tuple[dict[str, str], dict[str, tuple[str, str, str]], set[str]]:
    """Simulate the parameter state AMC reaches after its last step.

    Returns (state, last_writer, computed): last_writer[name] is
    ("file" | "forced" | "derived", step file, reason); computed holds every
    parameter AMC calculates from components or the flight controller.
    """
    fc_params = fc_params or {}
    state: dict[str, str] = {}
    writer: dict[str, tuple[str, str, str]] = {}
    computed: set[str] = set()
    for f in numbered_param_files(template_path):
        for k, v in read_param_file(f).items():
            state[k] = v
            writer[k] = ("file", f.name, "")
        info = (steps or {}).get(f.name, {})
        for kind in ("forced", "derived"):
            for pname, pinfo in info.get(f"{kind}_parameters", {}).items():
                new = pinfo.get("New Value")
                static = isinstance(new, (int, float)) or _num(new) is not None
                if kind == "derived" or not static:
                    computed.add(pname)
                    if static and "if" not in pinfo:
                        state[pname] = _fmt(new)
                        writer[pname] = (kind, f.name, pinfo.get("Change Reason", ""))
                    continue
                if "if" in pinfo and not _condition_holds(str(pinfo["if"]), fc_params):
                    continue
                state[pname] = _fmt(new)
                writer[pname] = ("forced", f.name, pinfo.get("Change Reason", ""))
    return state, writer, computed


METADATA_DIR = APP_DIR / "amc_metadata"
_METADATA_FAMILY = {"ArduCopter": "Copter", "Heli": "Copter", "ArduPlane": "Plane", "Rover": "Rover"}


def find_param_metadata(vehicle_type: str, fw_version: str, *search_roots: Path) -> Path | None:
    """Find an apm.pdef.xml for this firmware so AMC can open the project offline.

    Searched: ATLAS's amc_metadata/ folder first, then AMC's templates. Only a
    file generated from the same major.minor release is accepted; parameter
    documentation changes between releases.
    """
    family = _METADATA_FAMILY.get(vehicle_type)
    want = parse_fw(fw_version)
    if not family or not want:
        return None
    same_minor = None
    for root in (METADATA_DIR, *search_roots):
        if not root or not Path(root).is_dir():
            continue
        for f in sorted(Path(root).rglob("apm.pdef.xml")):
            try:
                with f.open(encoding="utf-8", errors="ignore") as fh:
                    head = fh.read(4096)
            except OSError:
                continue
            m = re.search(rf"Generated from git tag {family}-(\d+\.\d+\.\d+)", head)
            if not m or parse_fw(m.group(1)) != want:
                continue
            if m.group(1) == amc_fw_version(fw_version).split(" ")[0]:
                return f
            same_minor = same_minor or f
    return same_minor


def classify_param(name: str) -> str:
    """'never', 'calibration', or 'design'."""
    if any(r.search(name) for r in _NEVER_RE):
        return "never"
    if any(r.search(name) for r in _CAL_RE):
        return "calibration"
    return "design"


# ===========================================================================
# AMC templates
# ===========================================================================

@dataclass
class Template:
    name: str                 # e.g. "ArduCopter/Holybro_X500_V2"
    path: Path
    vehicle_type: str
    fw_version: str
    frame_class: str
    prop_in: float | None
    cells: int | None
    is_empty: bool

    @property
    def fw_major_minor(self) -> tuple[int, int] | None:
        return parse_fw(self.fw_version)

    def components(self) -> dict[str, Any]:
        return json.loads((self.path / "vehicle_components.json").read_text(encoding="utf-8-sig"))

    def label(self) -> str:
        bits = [self.name]
        if self.fw_version:
            bits.append(f"fw {self.fw_version}")
        if not self.is_empty:
            if self.frame_class and self.frame_class != "Undefined":
                bits.append(self.frame_class)
            if self.prop_in:
                bits.append(f'{self.prop_in:g}" props')
            if self.cells:
                bits.append(f"{self.cells}S")
        return "  ·  ".join(bits)


def amc_fw_version(text: str) -> str:
    """AMC wants 'major.minor.patch', optionally followed by a space and a label."""
    m = re.match(r"\s*(\d+\.\d+\.\d+)[\s._-]*(.*)$", text or "")
    if not m:
        return text
    return f"{m.group(1)} {m.group(2)}".strip()


def parse_fw(text: str | None) -> tuple[int, int] | None:
    m = re.search(r"(\d+)\.(\d+)", text or "")
    return (int(m.group(1)), int(m.group(2))) if m else None


def list_templates(templates_dir: Path) -> list[Template]:
    out: list[Template] = []
    for comp_file in sorted(templates_dir.rglob("vehicle_components.json")):
        folder = comp_file.parent
        if not numbered_param_files(folder):
            continue
        try:
            data = json.loads(comp_file.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError):
            continue
        comps = data.get("Components", {})
        fw = comps.get("Flight Controller", {}).get("Firmware", {})
        rel = folder.relative_to(templates_dir).as_posix()
        vehicle = fw.get("Type") or rel.split("/", 1)[0]
        out.append(Template(
            name=rel, path=folder, vehicle_type=vehicle,
            fw_version=str(fw.get("Version", "")).strip(),
            frame_class=str(comps.get("Frame", {}).get("Specifications", {}).get("Frame class", "")),
            prop_in=_num(comps.get("Propellers", {}).get("Specifications", {}).get("Diameter_inches")) or None,
            cells=_int(comps.get("Battery", {}).get("Specifications", {}).get("Number of cells")) or None,
            is_empty=folder.name.startswith("empty_"),
        ))
    return out


# ===========================================================================
# Reading an ATLAS aircraft
# ===========================================================================

@dataclass
class AircraftRecord:
    id: int
    row: dict[str, Any]
    components: list[dict[str, Any]]
    params: dict[str, str]
    param_file: dict[str, Any] | None

    @property
    def name(self) -> str:
        return self.row["name"]

    @property
    def fw_version(self) -> str:
        return (self.param_file or {}).get("ardupilot_version") or ""

    def part(self, category: str) -> dict[str, Any] | None:
        for c in self.components:
            if c["category"] == category:
                return c
        return None


def load_aircraft(conn: sqlite3.Connection, aircraft_id: int) -> AircraftRecord:
    conn.row_factory = sqlite3.Row
    row = conn.execute("SELECT * FROM aircraft WHERE id=?", (aircraft_id,)).fetchone()
    if row is None:
        raise BridgeError(f"Aircraft #{aircraft_id} is not in the repository.")
    comps = [dict(r) for r in conn.execute(
        "SELECT * FROM components WHERE aircraft_id=? ORDER BY id", (aircraft_id,))]
    # Hardware snapshots first; a SITL tune is evidence about a tune, not a build.
    pf = bridge.latest_param_file(conn, aircraft_id)
    params: dict[str, str] = {}
    if pf is not None:
        # Read from the database, not the stored file path: stored paths are
        # absolute and break when the repository moves to another computer.
        params = {r["name"]: r["value"] for r in conn.execute(
            "SELECT name, value FROM parameters WHERE parameter_file_id=? ORDER BY id", (pf["id"],))}
    return AircraftRecord(aircraft_id, dict(row), comps, params, dict(pf) if pf else None)


def vehicle_type_for(rec: AircraftRecord) -> str | None:
    return FRAME_TYPE_TO_VEHICLE.get((rec.row.get("frame_type") or "").strip())


def suggest_template(rec: AircraftRecord, templates: list[Template]) -> tuple[Template | None, str]:
    """Pick the template an export should start from, and say why.

    With a parameter snapshot, the build's own tune is the content, so the
    baseline must be the EMPTY template for its firmware: starting from another
    vehicle's template would mix two vehicles' choices. Without a snapshot, the
    nearest real build is the most useful starting point.
    """
    vt = vehicle_type_for(rec)
    if vt is None:
        return None, (f"Frame type '{rec.row.get('frame_type') or 'not set'}' does not map to an "
                      "ArduPilot vehicle type. Choose a template manually.")
    pool = [t for t in templates if t.vehicle_type == vt]
    if not pool:
        return None, f"No {vt} templates were found in the AMC install."
    if rec.params:
        fw = parse_fw(rec.fw_version)
        if fw is None:
            return None, ("The parameter snapshot has no firmware version recorded. Choose the empty "
                          "template that matches the firmware on the flight controller.")
        for t in pool:
            if t.is_empty and t.fw_major_minor == fw:
                return t, (f"Snapshot is firmware {rec.fw_version}: start from the empty {fw[0]}.{fw[1]}.x "
                           "template and carry the ATLAS tune as an imported delta.")
        return None, (f"AMC has no empty template for firmware {fw[0]}.{fw[1]}. Update AMC, or choose a "
                      "template manually and check the firmware version in AMC.")
    # No snapshot: nearest real build
    want_class = _frame_class_from_record(rec)
    prop = _num(rec.row.get("prop_size_in"))
    cells = _int(rec.row.get("cell_count"))
    best, best_score, why = None, None, ""
    for t in pool:
        if t.is_empty:
            continue
        score = 0.0
        reasons = []
        if want_class and t.frame_class == want_class:
            score -= 10; reasons.append(f"same frame class ({want_class})")
        if prop and t.prop_in:
            score += abs(prop - t.prop_in); reasons.append(f'props {t.prop_in:g}" vs {prop:g}"')
        else:
            score += 5
        if cells and t.cells:
            score += abs(cells - t.cells) * 0.5; reasons.append(f"{t.cells}S vs {cells}S")
        if best_score is None or score < best_score:
            best, best_score, why = t, score, ", ".join(reasons)
    if best is None:
        return None, "No populated template fits. Choose an empty template manually."
    return best, f"No parameter snapshot; nearest published build: {why}."


def _frame_class_from_record(rec: AircraftRecord) -> str | None:
    fc = _int(rec.params.get("FRAME_CLASS"))
    if fc is not None:
        return FRAME_CLASSES["ArduCopter"].get(fc)
    return FRAME_TYPE_TO_CLASS.get(rec.row.get("frame_type") or "")


# ===========================================================================
# Building vehicle_components.json
# ===========================================================================

@dataclass
class Provenance:
    path: tuple[str, ...]
    value: Any
    source: str
    note: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {"field": " > ".join(self.path), "value": self.value, "source": self.source, "note": self.note}


class _Writer:
    """Sets component fields and records where each value came from."""

    def __init__(self, components: dict[str, Any]):
        self.c = components
        self.log: dict[tuple[str, ...], Provenance] = {}

    def set(self, path: tuple[str, ...], value: Any, source: str, note: str = "") -> None:
        node = self.c
        for key in path[:-1]:
            node = node.setdefault(key, {})
        node[path[-1]] = value
        self.log[path] = Provenance(path, value, source, note)

    def get(self, path: tuple[str, ...]) -> Any:
        node: Any = self.c
        for key in path:
            if not isinstance(node, dict) or key not in node:
                return None
            node = node[key]
        return node


def _product(w: _Writer, component: str, part: dict[str, Any] | None) -> None:
    if not part:
        return
    if (part.get("manufacturer") or "").strip():
        w.set((component, "Product", "Manufacturer"), part["manufacturer"].strip(), SRC_RECORD)
    if (part.get("model") or "").strip():
        w.set((component, "Product", "Model"), part["model"].strip(), SRC_RECORD)
    url = re.search(r"https?://\S+", part.get("specs") or "")
    if url:
        w.set((component, "Product", "URL"), url.group(0).rstrip(","), SRC_RECORD)


def _serial_ports_with(params: dict[str, str], wanted: set[int]) -> list[tuple[str, int]]:
    hits = []
    for port in SERIAL_PORTS:
        v = _int(params.get(f"{port}_PROTOCOL"))
        if v in wanted:
            hits.append((port, v))
    return hits


def build_components(rec: AircraftRecord, template: Template) -> tuple[dict[str, Any], list[Provenance], list[str]]:
    """Return (vehicle_components.json content, provenance, warnings)."""
    data = copy.deepcopy(template.components())
    comps = data.setdefault("Components", {})
    w = _Writer(comps)
    p = rec.params
    warnings: list[str] = []
    vt = template.vehicle_type

    # --- Flight controller --------------------------------------------------
    _product(w, "Flight Controller", rec.part("Flight Controller"))
    w.set(("Flight Controller", "Firmware", "Type"), vt, SRC_INFERRED, "from the chosen template")
    if rec.fw_version:
        w.set(("Flight Controller", "Firmware", "Version"), amc_fw_version(rec.fw_version), SRC_RECORD,
              "" if amc_fw_version(rec.fw_version) == rec.fw_version else f"recorded as '{rec.fw_version}'")
    fc_text = " ".join(str((rec.part("Flight Controller") or {}).get(k) or "") for k in ("model", "specs"))
    for pattern, mcu in ((r"\bH7|H74\d|H75\d", "STM32H7xx"), (r"\bF7|F7[0-9]{2}", "STM32F7xx"),
                         (r"\bF4|F4[0-9]{2}", "STM32F4xx")):
        if re.search(pattern, fc_text, re.I):
            w.set(("Flight Controller", "Specifications", "MCU Series"), mcu, SRC_INFERRED,
                  "from the flight controller model name")
            break

    # --- Frame --------------------------------------------------------------
    _product(w, "Frame", rec.part("Frame"))
    fc_class = _int(p.get("FRAME_CLASS"))
    if fc_class is not None and fc_class in FRAME_CLASSES.get("ArduCopter", {}) and vt in ("ArduCopter", "Heli"):
        w.set(("Frame", "Specifications", "Frame class"), FRAME_CLASSES["ArduCopter"][fc_class], SRC_PARAMS,
              f"FRAME_CLASS={fc_class}")
    elif rec.row.get("frame_type") in FRAME_TYPE_TO_CLASS:
        w.set(("Frame", "Specifications", "Frame class"), FRAME_TYPE_TO_CLASS[rec.row["frame_type"]],
              SRC_INFERRED, f"from frame type '{rec.row['frame_type']}'")
    weight = _num(rec.row.get("weight_g"))
    if weight:
        kg = round(weight / 1000.0, 3)
        w.set(("Frame", "Specifications", "TOW min Kg"), kg, SRC_RECORD, "all-up weight")
        w.set(("Frame", "Specifications", "TOW max Kg"), kg, SRC_RECORD, "all-up weight")

    # --- Battery ------------------------------------------------------------
    batt_part = rec.part("Battery") or {}
    batt_text = " ".join(str(batt_part.get(k) or "") for k in ("model", "specs"))
    _product(w, "Battery", batt_part or None)
    cells = _int(rec.row.get("cell_count"))
    cell_src = SRC_RECORD
    if not cells:
        m = re.search(r"\b(\d{1,2})\s*S(?:\d+P)?\b", batt_text)
        if m:
            cells = int(m.group(1))
    if not cells:
        vmax = _num(p.get("MOT_BAT_VOLT_MAX"))
        if vmax:
            for per in (4.2, 4.35, 4.1, 4.45):
                n = vmax / per
                if abs(n - round(n)) < 0.03 and round(n) > 0:
                    cells, cell_src = int(round(n)), SRC_INFERRED
                    break
    if cells:
        w.set(("Battery", "Specifications", "Number of cells"), cells, cell_src,
              "" if cell_src == SRC_RECORD else "from MOT_BAT_VOLT_MAX")
        per_cell = {
            "Volt per cell max": p.get("MOT_BAT_VOLT_MAX"),
            "Volt per cell arm": p.get("BATT_ARM_VOLT"),
            "Volt per cell low": p.get("BATT_LOW_VOLT"),
            "Volt per cell crit": p.get("BATT_CRT_VOLT"),
            "Volt per cell min": p.get("MOT_BAT_VOLT_MIN"),
        }
        for key, total in per_cell.items():
            v = _num(total)
            if v and v > 0:
                w.set(("Battery", "Specifications", key), round(v / cells, 3), SRC_PARAMS,
                      f"{_fmt(total)} V / {cells} cells")
        vmax_cell = _num(w.get(("Battery", "Specifications", "Volt per cell max")))
        if vmax_cell and ("Battery", "Specifications", "Volt per cell max") in w.log:
            chem = min(CHEMISTRY_BY_MAX_CELL, key=lambda t: abs(t[0] - vmax_cell))
            if abs(chem[0] - vmax_cell) <= 0.03:
                w.set(("Battery", "Specifications", "Chemistry"), chem[1], SRC_INFERRED,
                      f"{vmax_cell:g} V per cell at full charge")
        # No voltages in the snapshot and none in the template: standard values
        # for the recorded chemistry, marked as inferred. A zero here would
        # become BATT_ARM_VOLT=0, an aircraft that arms on a flat pack.
        chem_name = next((c for c in STANDARD_CELL_VOLTS if re.search(rf"\b{c}\b", batt_text, re.I)), None)
        chem_name = chem_name or str(w.get(("Battery", "Specifications", "Chemistry")) or "Lipo")
        standard = STANDARD_CELL_VOLTS.get(chem_name)
        if standard:
            filled = []
            for key, volts in standard.items():
                path = ("Battery", "Specifications", key)
                if path not in w.log and not (_num(w.get(path)) or 0) > 0:
                    w.set(path, volts, SRC_INFERRED, f"standard {chem_name} value")
                    filled.append(key)
            if filled:
                w.set(("Battery", "Specifications", "Chemistry"), chem_name, SRC_INFERRED,
                      "from the battery record" if re.search(chem_name, batt_text, re.I) else "assumed")
                warnings.append(f"Battery cell voltages are standard {chem_name} values, not this build's: "
                                f"{', '.join(k.replace('Volt per cell ', '') for k in filled)}. Confirm the pack "
                                "chemistry and failsafe voltages.")
    order =["Volt per cell max", "Volt per cell arm", "Volt per cell low", "Volt per cell crit", "Volt per cell min"]
    vals = [(k, _num(w.get(("Battery", "Specifications", k)))) for k in order]
    vals = [(k, v) for k, v in vals if v]
    for (hi_k, hi), (lo_k, lo) in zip(vals, vals[1:]):
        if lo > hi:
            warnings.append(f"Battery: {lo_k} ({lo:g} V) is above {hi_k} ({hi:g} V) in the snapshot. AMC requires "
                            "max ≥ arm ≥ low ≥ crit ≥ min and will not continue until one of them is corrected.")
    cap = _int(p.get("BATT_CAPACITY"))
    if cap:
        w.set(("Battery", "Specifications", "Capacity mAh"), cap, SRC_PARAMS, "BATT_CAPACITY")
    else:
        m = re.search(r"\b(\d{3,6})\s*mAh\b", batt_text, re.I)
        if m:
            w.set(("Battery", "Specifications", "Capacity mAh"), int(m.group(1)), SRC_RECORD, "battery record")

    # --- Battery monitor ----------------------------------------------------
    bm = _int(p.get("BATT_MONITOR"))
    if bm is not None:
        if bm in BATT_MONITORS:
            conn_type, proto = BATT_MONITORS[bm]
            if conn_type == "I2C":
                bus = _int(p.get("BATT_I2C_BUS")) or 0
                conn_type = f"I2C{min(max(bus, 0), 3) + 1}"
            w.set(("Battery Monitor", "FC Connection", "Type"), conn_type, SRC_PARAMS, f"BATT_MONITOR={bm}")
            w.set(("Battery Monitor", "FC Connection", "Protocol"), proto, SRC_PARAMS, f"BATT_MONITOR={bm}")
        else:
            warnings.append(f"BATT_MONITOR={bm} is not a monitor type this bridge maps. Set the battery "
                            "monitor in AMC's component editor.")

    # --- ESC ----------------------------------------------------------------
    _product(w, "ESC", rec.part("ESC"))
    serial_esc = [(port, v) for port, v in _serial_ports_with(p, {38, 39, 41})]
    if serial_esc:
        port, v = serial_esc[0]
        proto = SERIAL_PROTOCOLS[v][1]
        for sec in ("FC->ESC Connection", "ESC->FC Telemetry"):
            w.set(("ESC", sec, "Type"), port, SRC_PARAMS, f"{port}_PROTOCOL={v}")
            w.set(("ESC", sec, "Protocol"), proto, SRC_PARAMS, f"{port}_PROTOCOL={v}")
    else:
        mpt = _int(p.get("MOT_PWM_TYPE"))
        names = MOT_PWM_TYPES.get(vt, MOT_PWM_TYPES["ArduCopter"])
        if mpt is not None and mpt in names:
            aio = any(_int(p.get(f"SERVO{i}_FUNCTION")) in ESC_SERVO_FUNCTIONS for i in range(9, 15))
            out_type = "AIO" if aio else "Main Out"
            w.set(("ESC", "FC->ESC Connection", "Type"), out_type, SRC_PARAMS, "SERVO9-14_FUNCTION")
            w.set(("ESC", "FC->ESC Connection", "Protocol"), names[mpt], SRC_PARAMS, f"MOT_PWM_TYPE={mpt}")
            telem = _serial_ports_with(p, {16, 28})
            if telem:
                port, v = telem[0]
                allowed = ("Scripting" if names[mpt] == "Normal" else
                           "ESC Telemetry" if names[mpt].startswith(("DShot", "OneShot")) else None)
                if allowed != SERIAL_PROTOCOLS[v][1]:
                    warnings.append(
                        f"ESC: {port}_PROTOCOL={v} ({SERIAL_PROTOCOLS[v][1]}) with {names[mpt]} is a combination "
                        f"AMC's component editor does not offer (it expects {allowed or 'no serial telemetry'}). "
                        "The value is carried over unchanged; confirm the ESC telemetry wiring in AMC.")
                w.set(("ESC", "ESC->FC Telemetry", "Type"), port, SRC_PARAMS, f"{port}_PROTOCOL={v}")
                w.set(("ESC", "ESC->FC Telemetry", "Protocol"), SERIAL_PROTOCOLS[v][1], SRC_PARAMS,
                      f"{port}_PROTOCOL={v}")
            elif names[mpt].startswith("DShot") and _int(p.get("SERVO_BLH_BDMASK")):
                w.set(("ESC", "ESC->FC Telemetry", "Type"), out_type, SRC_PARAMS, "SERVO_BLH_BDMASK")
                w.set(("ESC", "ESC->FC Telemetry", "Protocol"), "BDShotOnly", SRC_PARAMS, "SERVO_BLH_BDMASK")
            elif "MOT_PWM_TYPE" in p:
                w.set(("ESC", "ESC->FC Telemetry", "Type"), "None", SRC_PARAMS, "no ESC telemetry configured")
                w.set(("ESC", "ESC->FC Telemetry", "Protocol"), "None", SRC_PARAMS, "no ESC telemetry configured")
        elif mpt is not None:
            warnings.append(f"MOT_PWM_TYPE={mpt} is not a protocol this bridge maps. Set the ESC protocol "
                            "in AMC's component editor before writing anything.")

    # --- Motors / propellers -----------------------------------------------
    _product(w, "Motors", rec.part("Motor"))
    poles = None
    for name in ("SERVO_BLH_POLES", "ESC_HW_POLES", "SERVO_FTW_POLES"):
        v = _int(p.get(name))
        if v and v > 1:
            poles = (v, SRC_PARAMS, name)
            break
    if poles is None:
        m = re.search(r"\b\d{1,2}N(\d{1,2})P\b|\b(\d{1,2})\s*poles?\b", (rec.part("Motor") or {}).get("specs") or "", re.I)
        if m:
            poles = (int(m.group(1) or m.group(2)), SRC_RECORD, "motor specs")
    if poles:
        w.set(("Motors", "Specifications", "Poles"), poles[0], poles[1], poles[2])
    prop = _num(rec.row.get("prop_size_in"))
    if prop:
        w.set(("Propellers", "Specifications", "Diameter_inches"), prop, SRC_RECORD)

    # --- GNSS ---------------------------------------------------------------
    _product(w, "GNSS Receiver", rec.part("GPS"))
    gname = "GPS1_TYPE" if "GPS1_TYPE" in p else ("GPS_TYPE" if "GPS_TYPE" in p else None)
    if gname:
        g = _int(p.get(gname))
        if g in GNSS_TYPES:
            bus, proto = GNSS_TYPES[g]
            if bus == "None":
                w.set(("GNSS Receiver", "FC Connection", "Type"), "None", SRC_PARAMS, f"{gname}={g}")
                w.set(("GNSS Receiver", "FC Connection", "Protocol"), "None", SRC_PARAMS, f"{gname}={g}")
                if rec.part("GPS"):
                    warnings.append(f"A GNSS module is recorded on this aircraft but {gname}=0 disables it. "
                                    "Confirm in AMC whether this build uses GNSS.")
            elif bus == "serial":
                ports = _serial_ports_with(p, {5})
                if ports:
                    w.set(("GNSS Receiver", "FC Connection", "Type"), ports[0][0], SRC_PARAMS,
                          f"{ports[0][0]}_PROTOCOL=5")
                else:
                    warnings.append(f"{gname}={g} but no SERIALn_PROTOCOL=5 port was found. Set the GNSS port in AMC.")
                w.set(("GNSS Receiver", "FC Connection", "Protocol"), proto, SRC_PARAMS, f"{gname}={g}")
            else:
                can_bus = "CAN2" if _int(p.get("CAN_D2_PROTOCOL")) == 1 and _int(p.get("CAN_D1_PROTOCOL")) != 1 else "CAN1"
                w.set(("GNSS Receiver", "FC Connection", "Type"), can_bus, SRC_PARAMS, "CAN_Dx_PROTOCOL")
                w.set(("GNSS Receiver", "FC Connection", "Protocol"), proto, SRC_PARAMS, f"{gname}={g}")
        elif g is not None:
            warnings.append(f"{gname}={g} is not a GNSS type this bridge maps. Set the GNSS receiver in AMC.")

    # --- RC receiver --------------------------------------------------------
    _product(w, "RC Receiver", rec.part("Receiver"))
    rc = _int(p.get("RC_PROTOCOLS"))
    if rc is not None:
        if rc in RC_PROTOCOLS:
            rc_ports = _serial_ports_with(p, {23})
            rc_type = rc_ports[0][0] if rc_ports else "RCin/SBUS"
            w.set(("RC Receiver", "FC Connection", "Type"), rc_type, SRC_PARAMS,
                  f"{rc_type}_PROTOCOL=23" if rc_ports else "no serial RCIN port")
            w.set(("RC Receiver", "FC Connection", "Protocol"), RC_PROTOCOLS[rc], SRC_PARAMS, f"RC_PROTOCOLS={rc}")
        else:
            warnings.append(f"RC_PROTOCOLS={rc} enables several protocols. AMC needs one: set it in AMC's "
                            "component editor.")

    # --- Telemetry ----------------------------------------------------------
    telem = _serial_ports_with(p, {1, 2, 43, 48})
    if telem:
        port, v = telem[0]
        w.set(("Telemetry", "FC Connection", "Type"), port, SRC_PARAMS, f"{port}_PROTOCOL={v}")
        w.set(("Telemetry", "FC Connection", "Protocol"), SERIAL_PROTOCOLS[v][1], SRC_PARAMS, f"{port}_PROTOCOL={v}")
    elif any(f"{port}_PROTOCOL" in p for port in SERIAL_PORTS):
        warnings.append("No serial port is configured for MAVLink telemetry. AMC's telemetry step expects "
                        "one; review it there, and leave it disabled if this build has no telemetry radio.")

    data["Configuration template"] = template.name
    # Every critical field the record did not supply is named, not hidden.
    for path in CRITICAL_FIELDS:
        if path not in w.log:
            warnings.append(f"{' > '.join(path)} kept the template value "
                            f"({w.get(path)!r}). AMC derives parameters from it: confirm it in AMC.")
    prov = list(w.log.values())
    for path in CRITICAL_FIELDS:
        if path not in w.log:
            prov.append(Provenance(path, w.get(path), SRC_TEMPLATE, "review in AMC"))
    return data, prov, warnings


# ===========================================================================
# Export plan (no files written) and export
# ===========================================================================

@dataclass
class ExportPlan:
    aircraft: AircraftRecord
    template: Template
    same_flight_controller: bool
    components: dict[str, Any]
    provenance: list[Provenance]
    delta: dict[str, str]
    stripped_calibration: list[str] = field(default_factory=list)
    never_transferred: list[str] = field(default_factory=list)
    safety_results: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    blockers: list[str] = field(default_factory=list)
    conflicts: list[dict[str, Any]] = field(default_factory=list)
    metadata: Path | None = None

    @property
    def ok(self) -> bool:
        return not self.blockers

    def project_name(self) -> str:
        base = re.sub(r"[^A-Za-z0-9._-]+", "_", self.aircraft.name).strip("_") or f"aircraft_{self.aircraft.id}"
        return f"{base}_ATLAS"


def plan_export(conn: sqlite3.Connection, aircraft_id: int, template: Template, *,
                same_flight_controller: bool = False, overrides: dict[str, str] | None = None) -> ExportPlan:
    """Everything an export would contain, written nowhere.

    overrides: {parameter: "amc" | "atlas"} — the operator's settlement of
    conflicts; parameters not named follow the default rule.
    """
    rec = load_aircraft(conn, aircraft_id)
    blockers: list[str] = []
    vt = vehicle_type_for(rec)
    if vt and vt != template.vehicle_type:
        blockers.append(f"The aircraft is a {rec.row.get('frame_type')} ({vt}) but the template is "
                        f"{template.vehicle_type}. Choose a {vt} template.")
    if rec.params and rec.fw_version:
        fw, tfw = parse_fw(rec.fw_version), template.fw_major_minor
        if template.is_empty and fw and tfw and fw != tfw:
            blockers.append(f"Snapshot firmware {rec.fw_version} does not match template firmware "
                            f"{template.fw_version}. Parameter names change between releases.")

    components, prov, warnings = build_components(rec, template)

    # What AMC will have done by its last step: every file in order, each with
    # the step's fixed forced/derived values applied on top.
    steps = load_configuration_steps(template.vehicle_type, template.path)
    baseline, last_writer, amc_computed = amc_end_state(template.path, steps, rec.params)
    if steps is None:
        warnings.append("AMC's step definitions were not found next to its templates, so conflicts with "
                        "AMC's mandatory values could not be listed here. AMC still applies them.")
    delta: dict[str, str] = {}
    stripped: list[str] = []
    never: list[str] = []
    conflicts: list[dict[str, Any]] = []
    for name, value in rec.params.items():
        kind = classify_param(name)
        if kind == "never":
            never.append(name)
            continue
        if kind == "calibration" and not same_flight_controller:
            stripped.append(name)
            continue
        writer = last_writer.get(name)
        if writer and writer[0] == "forced" and not _same(baseline[name], value):
            c = make_conflict(name, value, baseline[name], writer[1], writer[2], "forced", overrides)
            conflicts.append(c)
            if c["winner"] == ATLAS:
                delta[name] = value     # the imported file restores it after AMC's step
            continue
        # Values AMC computes from the components are always carried, so the
        # ATLAS tune is restored even where AMC's formula would differ.
        if name in amc_computed or name not in baseline or not _same(baseline[name], value):
            delta[name] = value

    # Fail closed on Acro parameters: the one definition of "allowed to leave".
    results = []
    for name, value in delta.items():
        if name in safety.ACRO_PARAM_RANGES:
            r = safety.check_param(name, value)
            results.append(r)
            if r["verdict"] != safety.OK:
                blockers.append(f"Safety check: {r['message']}")

    if not rec.params:
        warnings.insert(0, "No hardware parameter snapshot is stored for this aircraft. The project will "
                           "contain the template's parameters only, adjusted by AMC from the components.")
    if stripped:
        warnings.append(f"{len(stripped)} sensor and radio calibration values were left out. Run AMC's "
                        "calibration steps on the new flight controller.")
    fw_for_docs = rec.fw_version or template.fw_version
    metadata = find_param_metadata(template.vehicle_type, fw_for_docs, _templates_root(template.path))
    if metadata is None:
        warnings.append(
            f"No offline ArduPilot parameter documentation for {template.vehicle_type} {fw_for_docs}. AMC will try "
            "to download it when the project opens. For offline use, copy that version's apm.pdef.xml into "
            f"{METADATA_DIR.name}/ next to ATLAS and export again.")
    plan = ExportPlan(rec, template, same_flight_controller, components, prov, delta,
                      sorted(stripped), sorted(never), results, warnings, blockers)
    plan.conflicts = conflicts
    plan.metadata = metadata
    return plan


def write_export(plan: ExportPlan, parent_dir: Path, project_name: str | None = None) -> Path:
    """Write the AMC project. Never overwrites."""
    if not plan.ok:
        raise BridgeError("Export blocked:\n• " + "\n• ".join(plan.blockers))
    dest = Path(parent_dir) / (project_name or plan.project_name())
    if dest.exists():
        raise BridgeError(f"{dest} already exists. Choose another name or folder; exports never overwrite.")
    tmp = dest.with_name(dest.name + ".partial")
    if tmp.exists():
        shutil.rmtree(tmp)
    try:
        shutil.copytree(plan.template.path, tmp, ignore=shutil.ignore_patterns("*.bak", "__pycache__"))
        (tmp / "vehicle_components.json").write_text(
            json.dumps(plan.components, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
        import_file = None
        if plan.delta:
            highest = max((int(p.name[:2]) for p in numbered_param_files(tmp)), default=0)
            if highest >= 99:
                raise BridgeError("The template has no free numbered slot for the imported parameters.")
            import_file = f"{highest + 1:02d}_imported_{IMPORT_SOURCE}_parameters.param"
            lines = [f"{n},{_fmt(v)}  # ATLAS: {plan.aircraft.name}" for n, v in sorted(plan.delta.items())]
            (tmp / import_file).write_text("\n".join(lines) + "\n", encoding="utf-8")
        if plan.metadata is not None:
            shutil.copy2(plan.metadata, tmp / "apm.pdef.xml")
        report = {
            "format": HANDOFF_FORMAT,
            "created": datetime.now().isoformat(timespec="seconds"),
            "aircraft": {"id": plan.aircraft.id, "name": plan.aircraft.name,
                         "validation_status": plan.aircraft.row.get("validation_status")},
            "parameter_source": (plan.aircraft.param_file or {}).get("original_name"),
            "firmware": plan.aircraft.fw_version,
            "template": plan.template.name,
            "same_flight_controller": plan.same_flight_controller,
            "imported_parameter_file": import_file,
            "imported_parameter_count": len(plan.delta),
            "calibration_left_out": plan.stripped_calibration,
            "never_transferred": plan.never_transferred,
            "component_provenance": [p.as_dict() for p in plan.provenance],
            "safety_check": plan.safety_results,
            "amc_vs_atlas": plan.conflicts,
            "offline_parameter_documentation": str(plan.metadata) if plan.metadata else None,
            "review_before_flight": conflict_warnings(plan.conflicts) + plan.warnings,
            "note": ("ATLAS validation status belongs to the source aircraft and does not transfer. "
                     "Configure, calibrate and test this vehicle through AMC's steps before flight."),
        }
        (tmp / "atlas_handoff.json").write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
        tmp.rename(dest)
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return dest


# ===========================================================================
# Seeding ATLAS from AMC templates
# ===========================================================================

#: AMC component -> ATLAS component category
AMC_TO_ATLAS = {"Flight Controller": "Flight Controller", "Frame": "Frame", "ESC": "ESC",
                "Motors": "Motor", "GNSS Receiver": "GPS", "RC Receiver": "Receiver",
                "Telemetry": "Telemetry", "Battery": "Battery", "Propellers": "Propellers"}
CATALOG_CATEGORIES = {"Flight Controller", "ESC", "Motor", "Frame", "GPS", "Receiver"}


def template_aircraft_name(t: Template) -> str:
    return f"AMC · {t.name.split('/', 1)[-1]}"


def import_template(conn: sqlite3.Connection, t: Template, *, files_dir: Path = FILES_DIR,
                    add_to_catalog: bool = True) -> int | None:
    """Store one AMC template as an ATLAS build. Returns the aircraft id, or None if present.

    Stored UNTESTED, by the repository's own rule: a parameter file records
    what a tune was, not how it flew on this team's hardware. Weight is left
    blank because no template records all-up weight.
    """
    name = template_aircraft_name(t)
    if conn.execute("SELECT 1 FROM aircraft WHERE name=?", (name,)).fetchone():
        return None
    comps = t.components().get("Components", {})
    # The state AMC reaches after its last step, forced values included.
    params, _, _ = amc_end_state(t.path, load_configuration_steps(t.vehicle_type, t.path))
    params = {k: v for k, v in params.items() if classify_param(k) != "never"}
    now = datetime.now().isoformat(timespec="seconds")
    frame_type = CLASS_TO_FRAME_TYPE.get(t.frame_class) if t.vehicle_type == "ArduCopter" else \
        ("Fixed Wing" if t.vehicle_type == "ArduPlane" else "Other")
    frame_size = f"{t.prop_in:g} inch" if t.prop_in else ""
    notes = (f"{TEMPLATE_ORIGIN_TAG} {t.name} (firmware {t.fw_version or 'not stated'}).\n"
             "Published by the ArduPilot Methodic Configurator project. Not flown by this team: "
             "validation is UNTESTED until there is evidence on our hardware.")
    aid = conn.execute(
        "INSERT INTO aircraft(name,frame_type,frame_size,validation_status,notes,created_at,"
        "weight_g,prop_size_in,cell_count,wheelbase_mm) VALUES(?,?,?,?,?,?,?,?,?,?)",
        (name, frame_type or "Other", frame_size, "UNTESTED", notes, now, None, t.prop_in, t.cells, None)).lastrowid

    motor_qty = {"Quad": 4, "Hexa": 6, "Octa": 8, "OctaQuad": 8, "Y6": 6}.get(t.frame_class)
    for amc_name, cat in AMC_TO_ATLAS.items():
        c = comps.get(amc_name)
        if not isinstance(c, dict):
            continue
        prod = c.get("Product", {}) or {}
        make, model = str(prod.get("Manufacturer", "")).strip(), str(prod.get("Model", "")).strip()
        spec_bits = []
        specs = c.get("Specifications", {}) or {}
        if amc_name == "Motors":
            if specs.get("Poles"):
                spec_bits.append(f"{specs['Poles']} poles")
            if motor_qty:
                spec_bits.append(f"Qty {motor_qty}")
        elif amc_name == "Battery":
            if specs.get("Number of cells"):
                spec_bits.append(f"{specs['Number of cells']}S")
            if specs.get("Capacity mAh"):
                spec_bits.append(f"{specs['Capacity mAh']} mAh")
            if specs.get("Chemistry"):
                spec_bits.append(str(specs["Chemistry"]))
        elif amc_name == "Propellers" and specs.get("Diameter_inches"):
            spec_bits.append(f"{specs['Diameter_inches']} in")
        conn_ = c.get("FC Connection") or c.get("FC->ESC Connection")
        if isinstance(conn_, dict) and conn_.get("Protocol") not in (None, "", "None", "Disabled"):
            spec_bits.append(f"{conn_.get('Type')} {conn_.get('Protocol')}")
        if prod.get("URL"):
            spec_bits.append(str(prod["URL"]))
        if make or model or spec_bits:
            conn.execute("INSERT INTO components(aircraft_id,category,manufacturer,model,specs) VALUES(?,?,?,?,?)",
                         (aid, cat, make, model, ", ".join(spec_bits)))
            if add_to_catalog and cat in CATALOG_CATEGORIES and make and model:
                conn.execute("INSERT OR IGNORE INTO hardware_catalog(category,manufacturer,model,specs,created_at) "
                             "VALUES(?,?,?,?,?)", (cat, make, model, "", now))

    files_dir.mkdir(exist_ok=True)
    stored = files_dir / f"{aid}_AMC_{t.name.replace('/', '_')}.param"
    stored.write_text("\n".join(f"{k},{_fmt(v)}" for k, v in sorted(params.items())) + "\n", encoding="utf-8")
    pf = conn.execute(
        "INSERT INTO parameter_files(aircraft_id,original_name,stored_path,ardupilot_version,imported_at,source) "
        "VALUES(?,?,?,?,?,?)",
        (aid, f"AMC template {t.name} (merged steps)", str(stored), t.fw_version, now, bridge.SOURCE_HARDWARE)).lastrowid
    conn.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",
                     [(pf, k, v) for k, v in sorted(params.items())])
    return aid


def import_templates(conn: sqlite3.Connection, templates: list[Template], **kw) -> tuple[list[str], list[str]]:
    """Import every populated template. Returns (added names, skipped names)."""
    added, skipped = [], []
    for t in templates:
        if t.is_empty:
            continue
        aid = import_template(conn, t, **kw)
        (added if aid else skipped).append(template_aircraft_name(t))
    return added, skipped


# ===========================================================================
# Command line, for testing without the interface
# ===========================================================================

def _main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Hand an ATLAS aircraft to ArduPilot Methodic Configurator.")
    ap.add_argument("--db", default=str(APP_DIR / "atlas_repository.db"))
    ap.add_argument("--templates", help="AMC vehicle_templates folder (default: auto-detect)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("templates", help="list AMC templates")
    ex = sub.add_parser("export", help="export one aircraft as an AMC project")
    ex.add_argument("aircraft_id", type=int)
    ex.add_argument("--out", required=True)
    ex.add_argument("--template", help="template name, e.g. ArduCopter/empty_4.7.x")
    ex.add_argument("--same-fc", action="store_true", help="keep sensor calibrations (same physical board)")
    ex.add_argument("--dry-run", action="store_true")
    im = sub.add_parser("import-templates", help="add AMC's populated templates to the repository")
    im.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    tdir = resolve_templates_dir(a.templates) if a.templates else find_templates_dir()
    if not tdir:
        print("AMC templates not found. Pass --templates PATH.", file=sys.stderr)
        return 2
    templates = list_templates(tdir)
    if a.cmd == "templates":
        for t in templates:
            print(t.label())
        return 0
    conn = sqlite3.connect(a.db)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    if a.cmd == "import-templates":
        added, skipped = import_templates(conn, templates)
        if a.dry_run:
            conn.rollback()
        else:
            conn.commit()
        print(f"{'Would add' if a.dry_run else 'Added'} {len(added)}; already present {len(skipped)}")
        for n in added:
            print("  +", n)
        return 0
    rec = load_aircraft(conn, a.aircraft_id)
    if a.template:
        tpl = next((t for t in templates if t.name == a.template), None)
        why = "chosen on the command line"
    else:
        tpl, why = suggest_template(rec, templates)
    if tpl is None:
        print(f"No template: {why}", file=sys.stderr)
        return 2
    plan = plan_export(conn, a.aircraft_id, tpl, same_flight_controller=a.same_fc)
    print(f"Aircraft   {rec.name}\nTemplate   {tpl.name}  ({why})\nDelta      {len(plan.delta)} parameters"
          f"\nCal. out   {len(plan.stripped_calibration)}\nNever      {len(plan.never_transferred)}")
    for pr in plan.provenance:
        print(f"  {pr.source:<18} {' > '.join(pr.path):<45} {pr.value!r}  {pr.note}")
    for c in plan.conflicts:
        print(f"  {'SAFETY ' if c['safety'] else '       '}{c['kind']:<8} {c['parameter']:<18} tune={c['atlas']:<10} "
              f"amc={c['amc']:<10} -> {c['winner'].upper()} ({c['decided_by']})")
    for wmsg in conflict_warnings(plan.conflicts) + plan.warnings:
        print("  ! " + wmsg)
    for b in plan.blockers:
        print("  BLOCKED: " + b)
    if a.dry_run or not plan.ok:
        return 0 if plan.ok else 1
    print("Wrote", write_export(plan, Path(a.out)))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
