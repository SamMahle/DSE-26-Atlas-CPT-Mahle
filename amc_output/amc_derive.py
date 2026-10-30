"""Offline replica of AMC's per-step parameter derivation.

Walks an AMC vehicle project in step order, applies each step's
forced/derived/add parameters from configuration_steps_ArduCopter.json
(evaluated against vehicle_components.json), writes them back into the step
files, and emits the merged result as a single .param file.

fc_parameters is simulated as "what would be on the FC after uploading every
previous step", starting from 00_default.param.
"""
import json
import re
import sys
from math import log  # noqa: F401  (used by eval'd expressions)
from pathlib import Path


def Version(s):  # minimal stand-in for packaging.version.Version
    return tuple(int(x) for x in re.findall(r"\d+", s))


vehicle_dir = Path(sys.argv[1])
steps_json = Path(sys.argv[2])
out_path = Path(sys.argv[3])
last_step = int(sys.argv[4]) if len(sys.argv) > 4 else 99  # merge steps <= this

# Labels AMC resolves via the param docs; restricted to the ones the Lucid uses.
LABELS = {
    ("RC_PROTOCOLS", "CRSF"): 512,
    ("MOT_PWM_TYPE", "DShot600"): 6,
    ("BATT_MONITOR", "Analog Voltage and Current"): 4,
    ("FRAME_CLASS", "Quad"): 1,
    ("GPS1_TYPE", "None"): 0,
    ("GPS_TYPE", "None"): 0,
}


def load(path):
    out = {}
    for raw in path.read_text(errors="ignore").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        name, val = re.split(r"[,\t ]+", line, maxsplit=1)
        out[name] = float(val)
    return out


def write(path, params, reasons):
    old = {}
    for raw in path.read_text(errors="ignore").splitlines():
        if raw.strip() and not raw.startswith("#"):
            name = re.split(r"[,\t ]", raw.strip(), maxsplit=1)[0]
            old[name] = raw.split("#", 1)[1].strip() if "#" in raw else ""
    lines = []
    for name in sorted(params):
        v = params[name]
        txt = f"{v:g}" if v != int(v) else str(int(v))
        comment = reasons.get(name) or old.get(name, "")
        lines.append(f"{name},{txt}" + (f"  # {comment}" if comment else ""))
    path.write_text("\n".join(lines) + "\n")


def to_number(name, value):
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except ValueError:
        if (name, value) not in LABELS:
            raise SystemExit(f"No numeric mapping for {name}={value!r}")
        return float(LABELS[(name, value)])


steps = json.loads(steps_json.read_text())["steps"]
vehicle_components = json.loads((vehicle_dir / "vehicle_components.json").read_text())["Components"]

files = sorted((p for p in vehicle_dir.glob("*.param") if re.match(r"\d+_", p.name)), key=lambda p: int(re.match(r"(\d+)", p.name).group(1)))
fc_parameters = load(vehicle_dir / "00_default.param")
merged = {}
changed = []

for f in files:
    params = load(f)
    reasons = {}
    step = steps.get(f.name, {})
    for kind in ("forced_parameters", "derived_parameters", "add_parameters"):
        for name, spec in step.get(kind, {}).items():
            cond = spec.get("if")
            if cond and not eval(cond):  # noqa: S307  (AMC's own trusted expressions)
                continue
            if kind == "add_parameters" and name not in fc_parameters:
                continue
            if "New Value" not in spec:  # reason-only entry, keeps the file's value
                continue
            raw = spec["New Value"]
            if isinstance(raw, str) and kind != "forced_parameters":
                try:
                    raw = eval(raw)  # noqa: S307
                except NameError:
                    pass
            new = to_number(name, raw)
            if params.get(name) != new:
                changed.append((f.name, name, params.get(name), new))
            params[name] = new
            reasons[name] = spec.get("Change Reason", "")
    if f.name != "00_default.param":
        write(f, params, reasons)
    fc_parameters.update(params)
    if int(f.name[:2]) <= last_step:
        merged.update(params)

header = (
    "# Lucid H7 / QAV-R 5in - generated from AMC project "
    f"{vehicle_dir.name} (empty_4.7.x template + Lucid components)\n"
    f"# Steps 00-{last_step:02d} merged (AMC 'Basic mandatory configuration' = ready for first flight)\n"
    "# Hardware/config layer derived by AMC rules; tune = AMC pre-flight starting values, not flight-derived\n"
)
body = "\n".join(
    f"{k},{(f'{v:g}' if v != int(v) else str(int(v)))}" for k, v in sorted(merged.items())
)
out_path.write_text(header + body + "\n")

for fn, name, old, new in changed:
    print(f"{fn:<45}{name:<20}{old!s:>10} -> {new:g}")
print(f"\nmerged params: {len(merged)}  -> {out_path}")
