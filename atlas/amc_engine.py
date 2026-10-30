#!/usr/bin/env python3
"""
amc_engine.py — PROTOTYPE of option B: run ArduPilot Methodic Configurator's
calculation engine inside ATLAS, with no AMC window.

    build in ATLAS ──► amc_bridge.plan_export()   components + tune delta
                   ──► AMC engine (in process)    derives, forces, validates
                   ──► full .param file + report  one file, one review

LICENSE — READ BEFORE DISTRIBUTING
---------------------------------
This module IMPORTS ArduPilot Methodic Configurator, which is GPL-3.0-or-later.
Distributing ATLAS with this module makes the distributed program a combined
work under GPL-3. The sponsor decision is pending (ATLAS licensing decision
memo). Until it is made, keep this module out of any copy delivered outside
West Point. ATLAS runs without it: the import is lazy and the Configuration
tab reports the engine as unavailable.

Requires, in the same Python as ATLAS:
    pip install ardupilot_methodic_configurator==5.0.1
and an offline apm.pdef.xml for the build's firmware (see amc_metadata/).
"""

from __future__ import annotations

import json
import logging
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import amc_bridge as bridge

TESTED_AMC_VERSION = "5.0.1"
ATLAS_ORIGIN = "ATLAS tune"
_DEFAULT_CUTOFF = 27


def first_flight_last_step(vehicle_type: str) -> int:
    """Last step before AMC's first phase that needs flight data, read from AMC's own phase list."""
    try:
        import ardupilot_methodic_configurator as amc_pkg  # noqa: PLC0415
        f = Path(amc_pkg.__file__).parent / f"configuration_steps_{vehicle_type}.json"
        phases = json.loads(f.read_text(encoding="utf-8")).get("phases", {})
    except Exception:  # noqa: BLE001
        return _DEFAULT_CUTOFF
    starts = [p["start"] for name, p in phases.items()
              if "start" in p and ("tuning" in name.lower() or "flight data" in p.get("description", "").lower())]
    return min(starts) - 1 if starts else _DEFAULT_CUTOFF


def default_template(rec: bridge.AircraftRecord, templates: list[bridge.Template]) -> tuple[bridge.Template | None, str]:
    """The engine's starting point: always an EMPTY template, so no other airframe's choices ride along.

    The build's firmware picks the release when it is known. A build with no
    firmware recorded gets the newest release that has offline documentation.
    """
    vt = bridge.vehicle_type_for(rec)
    empties = sorted((t for t in templates if t.is_empty and t.vehicle_type == vt),
                     key=lambda t: t.fw_major_minor or (0, 0), reverse=True)
    if not empties:
        return None, f"No empty {vt or 'matching'} template was found."
    fw = bridge.parse_fw(rec.fw_version)
    if fw:
        match = next((t for t in empties if t.fw_major_minor == fw), None)
        return (match, f"Firmware {rec.fw_version}: empty {fw[0]}.{fw[1]}.x template.") if match else \
            (None, f"No empty template for firmware {fw[0]}.{fw[1]}.")
    for t in empties:
        if bridge.find_param_metadata(t.vehicle_type, t.fw_version, bridge._templates_root(t.path)):
            return t, (f"No firmware recorded: newest release with offline documentation ({t.fw_version}). "
                       "Confirm it matches the flight controller.")
    return empties[0], f"No firmware recorded: newest release ({empties[0].fw_version}). Confirm it matches."


class EngineError(RuntimeError):
    """The engine cannot produce a configuration, with a reason fit for an operator."""


def status() -> tuple[bool, str]:
    """(available, one-line description). Never raises."""
    try:
        import ardupilot_methodic_configurator as amc_pkg  # noqa: PLC0415
    except ImportError:
        return False, ("AMC engine not installed in this Python. "
                       f"pip install ardupilot_methodic_configurator=={TESTED_AMC_VERSION}")
    except Exception as exc:  # noqa: BLE001 - any import failure means unavailable
        return False, f"AMC engine failed to load: {exc}"
    version = getattr(amc_pkg, "__version__", "unknown")
    if version != TESTED_AMC_VERSION:
        return True, f"AMC engine {version} (tested with {TESTED_AMC_VERSION}: verify outputs)"
    return True, f"AMC engine {version}"


class _Capture(logging.Handler):
    """Collect AMC's warnings and errors so they reach the operator, not a console."""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.messages: list[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        msg = record.getMessage()
        if "update" in msg.lower() and ("check" in msg.lower() or "rate limit" in msg.lower()):
            return  # AMC's online update check; irrelevant offline
        self.messages.append(f"{record.levelname.title()}: {msg}")


@dataclass
class GeneratedConfig:
    plan: bridge.ExportPlan
    params: dict[str, float]
    origin: dict[str, str]                 # parameter -> the AMC step file that set it last
    component_errors: list[str]
    engine_messages: list[str]
    metadata: Path
    amc_version: str
    changed_from_snapshot: list[tuple[str, float, float]] = field(default_factory=list)

    @property
    def from_atlas(self) -> int:
        return sum(1 for f in self.origin.values() if f == ATLAS_ORIGIN)

    @property
    def ok(self) -> bool:
        return not self.component_errors


def generate(conn, aircraft_id: int, template: bridge.Template, *,
             same_flight_controller: bool = False, overrides: dict[str, str] | None = None) -> GeneratedConfig:
    """Compute the first-flight parameter set AMC would reach, without opening AMC.

    overrides: {parameter: "amc" | "atlas"} settles disagreements between AMC
    and the build's tune; anything not named follows bridge.make_conflict's
    default rule. Every disagreement is returned in cfg.plan.conflicts.
    """
    available, why = status()
    if not available:
        raise EngineError(why)
    # Imported here so ATLAS starts, and every other tab works, without AMC installed.
    import ardupilot_methodic_configurator as amc_pkg  # noqa: PLC0415
    from ardupilot_methodic_configurator.backend_filesystem import LocalFilesystem  # noqa: PLC0415
    from ardupilot_methodic_configurator.data_model_par_dict import ParDict  # noqa: PLC0415
    from ardupilot_methodic_configurator.data_model_vehicle_components import ComponentDataModel  # noqa: PLC0415
    from ardupilot_methodic_configurator.data_model_vehicle_components_json_schema import (  # noqa: PLC0415
        VehicleComponentsJsonSchema,
    )

    plan = bridge.plan_export(conn, aircraft_id, template, same_flight_controller=same_flight_controller,
                              overrides=overrides)
    if not plan.ok:
        raise EngineError("Blocked before the engine ran:\n• " + "\n• ".join(plan.blockers))
    if plan.metadata is None:
        fw = plan.aircraft.fw_version or template.fw_version
        raise EngineError(
            f"No offline ArduPilot parameter documentation for {template.vehicle_type} {fw}. The engine needs it "
            f"to convert protocol names into parameter values. Put that release's apm.pdef.xml in "
            f"{bridge.METADATA_DIR} and try again.")

    capture = _Capture()
    root = logging.getLogger()
    root.addHandler(capture)
    try:
        with tempfile.TemporaryDirectory(prefix="atlas_engine_") as tmp:
            project = bridge.write_export(plan, Path(tmp), "project")
            fs = LocalFilesystem(str(project), template.vehicle_type, "", allow_editing_template_files=False,
                                 save_component_to_system_templates=False)
            data = fs.load_vehicle_components_json_data(str(project))
            schema_ok, schema_msg = fs.validate_vehicle_components(data)
            errors: list[str] = [] if schema_ok else [f"Schema: {schema_msg}"]

            model = ComponentDataModel(data, fs.doc_dict, VehicleComponentsJsonSchema(fs.load_schema()))
            model.init_possible_choices(fs.doc_dict)
            entries: dict[tuple[str, ...], str] = {}

            def walk(node: dict[str, Any], path: tuple[str, ...]) -> None:
                for k, v in node.items():
                    if isinstance(v, dict):
                        walk(v, (*path, k))
                    else:
                        entries[(*path, k)] = "" if v is None else str(v)

            walk(data["Components"], ())
            valid, messages = model.validate_all_data(entries)
            if not valid:
                errors.extend(m.replace("\n", " ") for m in messages)

            # The firmware's parameter set stands in for a connected flight
            # controller, so formulas gated on "parameter exists" are computed.
            defaults = {n: p.value for n, p in
                        ParDict.load_param_file_into_dict(str(project / "00_default.param")).items()}
            fc_like = dict(defaults)
            for n, v in plan.aircraft.params.items():
                if bridge.classify_param(n) == "design" and bridge._num(v) is not None:
                    fc_like[n] = float(v)
            pending = fs.calculate_derived_and_forced_param_changes(fc_param_names=list(fc_like),
                                                                    fc_parameters=fc_like)
            fs.apply_computed_changes(pending)

            # First-flight configuration: every step before AMC's flight-data
            # tuning phase. Later steps record values measured in flight; their
            # template contents belong to another airframe or are placeholders.
            cutoff = first_flight_last_step(template.vehicle_type)
            params: dict[str, float] = dict(defaults)
            origin: dict[str, str] = {n: "00_default.param" for n in defaults}
            for fname in sorted(fs.file_parameters):
                if not fname[:2].isdigit() or int(fname[:2]) > cutoff:
                    continue
                for name, par in fs.file_parameters[fname].items():
                    params[name], origin[name] = par.value, fname
    finally:
        root.removeHandler(capture)

    # Where AMC set a value itself (mandatory or calculated) and the tune
    # disagrees, that is a conflict for the operator; read straight from what
    # AMC actually produced, so nothing is guessed.
    steps = bridge.load_configuration_steps(template.vehicle_type, template.path) or {}
    conflicts: list[dict[str, Any]] = []
    for name, value in plan.aircraft.params.items():
        v = bridge._num(value)
        if v is None or name not in params or bridge.classify_param(name) != "design":
            continue
        if bridge._same(v, params[name]):
            continue
        info = steps.get(origin.get(name, ""), {})
        for kind in ("forced", "derived"):
            entry = info.get(f"{kind}_parameters", {}).get(name)
            if entry is not None:
                conflicts.append(bridge.make_conflict(name, v, params[name], origin[name],
                                                      entry.get("Change Reason", ""), kind, overrides))
                break
    amc_wins = {c["parameter"] for c in conflicts if c["winner"] == bridge.AMC}

    # The build's own tune is evidence from flight; it is laid over AMC's
    # starting values, except where a conflict was settled for AMC.
    for name, value in plan.aircraft.params.items():
        v = bridge._num(value)
        if v is None or name in amc_wins:
            continue
        kind = bridge.classify_param(name)
        if kind == "never" or (kind == "calibration" and not same_flight_controller):
            continue
        params[name], origin[name] = v, ATLAS_ORIGIN
    plan.conflicts = conflicts
    if not params.get("BATT_MONITOR"):
        plan.warnings.insert(0, "BATTERY MONITORING IS DISABLED (BATT_MONITOR=0): this configuration has no voltage "
                                "or capacity failsafe. Record the power module in the build, or set the battery "
                                "monitor before flight.")

    changed = []
    for name, value in plan.aircraft.params.items():
        if name in params and bridge.classify_param(name) == "design":
            src = bridge._num(value)
            if src is not None and not bridge._same(src, params[name]):
                changed.append((name, src, params[name]))
    return GeneratedConfig(plan, params, origin, errors, capture.messages, plan.metadata,
                           getattr(amc_pkg, "__version__", "unknown"), sorted(changed))


def write_param_file(cfg: GeneratedConfig, path: Path) -> Path:
    """Write the full configuration and a JSON report beside it. Never overwrites."""
    if not cfg.ok:
        raise EngineError("AMC rejected the components, so no file was written:\n• "
                          + "\n• ".join(cfg.component_errors))
    path = Path(path)
    report_path = path.with_suffix(".report.json")
    for p in (path, report_path):
        if p.exists():
            raise EngineError(f"{p} already exists. Choose another name; ATLAS never overwrites.")
    rec = cfg.plan.aircraft
    header = [
        "# ATLAS generated FIRST-FLIGHT configuration - NOT FLIGHT VALIDATED",
        f"# Aircraft: {rec.name}   (source validation: {rec.row.get('validation_status')}; does not transfer)",
        f"# Template: {cfg.plan.template.name}   Engine: ArduPilot Methodic Configurator {cfg.amc_version}",
        f"# Generated: {datetime.now().isoformat(timespec='seconds')}",
        "# Sensor calibrations are not included. Calibrate on the aircraft before flight.",
    ]
    lines = [f"{n},{bridge._fmt(v)}" for n, v in sorted(cfg.params.items())]
    path.write_text("\n".join(header + lines) + "\n", encoding="utf-8")
    report = {
        "aircraft": rec.name,
        "template": cfg.plan.template.name,
        "amc_engine_version": cfg.amc_version,
        "parameter_count": len(cfg.params),
        "from_atlas_tune": cfg.from_atlas,
        "changed_from_snapshot": [{"parameter": n, "atlas": a, "generated": g} for n, a, g in cfg.changed_from_snapshot],
        "amc_vs_atlas": cfg.plan.conflicts,
        "calibration_left_out": cfg.plan.stripped_calibration,
        "component_errors": cfg.component_errors,
        "engine_messages": cfg.engine_messages,
        "review_before_flight": bridge.conflict_warnings(cfg.plan.conflicts) + cfg.plan.warnings,
        "parameter_documentation": str(cfg.metadata),
    }
    report_path.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    return path


if __name__ == "__main__":
    import argparse
    import sqlite3

    ap = argparse.ArgumentParser(description="Generate a full ArduPilot configuration with the embedded AMC engine.")
    ap.add_argument("aircraft_id", type=int)
    ap.add_argument("--db", default=str(bridge.APP_DIR / "atlas_repository.db"))
    ap.add_argument("--templates")
    ap.add_argument("--template")
    ap.add_argument("--out")
    ap.add_argument("--amc", action="append", default=[], metavar="PARAM", help="settle this conflict for AMC")
    ap.add_argument("--atlas", action="append", default=[], metavar="PARAM", help="settle this conflict for the tune")
    a = ap.parse_args()
    overrides = {**{n: bridge.AMC for n in a.amc}, **{n: bridge.ATLAS for n in a.atlas}}
    print(status()[1])
    tdir = bridge.resolve_templates_dir(a.templates) if a.templates else bridge.find_templates_dir()
    templates = bridge.list_templates(tdir)
    c = sqlite3.connect(a.db)
    c.row_factory = sqlite3.Row
    rec = bridge.load_aircraft(c, a.aircraft_id)
    tpl = next((t for t in templates if t.name == a.template), None) if a.template else default_template(rec, templates)[0]
    cfg = generate(c, a.aircraft_id, tpl, overrides=overrides)
    print(f"{rec.name} on {tpl.name} (steps 00-{first_flight_last_step(tpl.vehicle_type):02d}): "
          f"{len(cfg.params)} parameters, {cfg.from_atlas} from the ATLAS tune, "
          f"{len(cfg.changed_from_snapshot)} changed from the snapshot, {len(cfg.component_errors)} component errors")
    for c in cfg.plan.conflicts:
        print(f"  {'SAFETY ' if c['safety'] else '       '}{c['kind']:<8} {c['parameter']:<18} tune={c['atlas']:<10} "
              f"amc={c['amc']:<10} -> {c['winner'].upper()} ({c['decided_by']})")
    for e in cfg.component_errors + cfg.engine_messages:
        print("  !", e)
    if a.out:
        try:
            print("Wrote", write_param_file(cfg, Path(a.out)))
        except EngineError as exc:
            print("Not written:", str(exc).splitlines()[0])
