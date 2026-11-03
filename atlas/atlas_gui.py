"""
atlas_gui.py — ATLAS v9.1 with a NiceGUI interface.

    python atlas_gui.py              opens ATLAS in your web browser
    python atlas_gui.py --native     opens it in its own desktop window (needs pywebview)

ATLAS runs as a small web server bound to this computer only (127.0.0.1); the
window is a page served by it. Nothing is reachable from the network, and
every file path shown is a path on this computer.

Behaviour matches the classic Tkinter interface (atlas_v9.py) screen for
screen: the same tabs, the same fail-closed safety check before anything leaves
the application, the same confirmation before every write to an aircraft. Only
the presentation changed. The Configuration tab lives in gui_amc.py and the
shared widgets in gui_widgets.py.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import shutil
import sqlite3
import subprocess
import sys
from datetime import datetime
from pathlib import Path

try:
    from nicegui import app, run, ui
except ImportError:                         # say what to do, instead of a traceback
    sys.exit("ATLAS needs NiceGUI for its interface. Install it with:\n\n"
             "    pip install -r requirements.txt\n\n"
             "(or run the classic interface: python atlas_v9.py)")

import acro_bridge as bridge
import ardupilot_interface as fc
import cbr_engine as cbr
import check_link
import ml_model
import safety
import sim_logs
from atlas_core import (APP_DIR, APP_VERSION, BENCH_DIR, BENCH_GROUP_PREFIX, CATEGORIES, DB_PATH, FILES_DIR,
                        FRAME_TYPES, LOG_DIR, LOG_PATTERNS, PARAM_PATTERNS, TEST_RESULTS, VALIDATION, db,
                        init_db, now_iso, open_in_file_browser, parse_param)
import gui_widgets as w
from gui_amc import ConfigurationTab

READY = "SYSTEM READY   •   LOCAL REPOSITORY   •   TRACEABILITY ENABLED"
NOTHING_STAGED = "Nothing staged. Predict a tune, load a .param file, or read the current values from a connected aircraft."

# Tab names, so quick-action buttons never depend on counting tabs.
TAB_OVERVIEW, TAB_INGEST, TAB_CBR, TAB_PREDICT, TAB_FC, TAB_SIM, TAB_BENCH, TAB_REPO, TAB_HW, TAB_AMC = (
    "Overview", "Add Aircraft", "CBR Assistant", "Predict Tuning", "Flight Controller", "Simulation",
    "Standard Bench", "Repository", "Hardware Catalog", "Configuration")


class Session:
    """State that must survive a page reload: the one live aircraft link and the one bench run.

    The interface is rebuilt for every browser tab; these are not. A reload
    therefore never leaves a flight-controller link or a bench process
    orphaned and unreachable.
    """

    def __init__(self):
        self.status = READY
        self.vehicle = None              # the one live flight-controller link
        self.fc_confirmed = False        # set only after the operator confirms a write
        self.fc_pending: dict[str, float] = {}
        self.fc_origin = "manual"
        self.last_prediction = None
        self.bench_proc: subprocess.Popen | None = None
        self.bench_outdir: str | None = None
        self.bench_lines: queue.Queue[str] = queue.Queue()

    def close(self):
        if self.vehicle is not None:
            try:
                self.vehicle.close()
            except Exception:  # noqa: BLE001
                pass
            self.vehicle = None
        proc = self.bench_proc
        if proc is not None and proc.poll() is None:
            _kill_tree(proc)


SESSION = Session()


def _kill_tree(proc: subprocess.Popen) -> None:
    """Stop the bench and the worker processes and SITL instances it started."""
    if sys.platform.startswith("win"):
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True,
                       creationflags=subprocess.CREATE_NO_WINDOW)
    else:
        proc.kill()


class AtlasApp:
    """One ATLAS window: built once per browser tab, sharing SESSION."""

    def __init__(self):
        w.install_theme()
        self.weights = dict(cbr.WEIGHTS)
        self.matches: list[dict] = []
        self.edit_hw_id: int | None = None
        self._pred_sources: dict[str, int] = {}
        with ui.column().classes("w-full gap-0") as self.root:
            self._header()
            with ui.tabs().props("dense align=left no-caps active-color=white indicator-color=primary") \
                    .classes("atlas-tabs w-full px-6") as self.tabs:
                for name in (TAB_OVERVIEW, TAB_INGEST, TAB_CBR, TAB_PREDICT, TAB_FC, TAB_SIM, TAB_BENCH,
                             TAB_REPO, TAB_HW, TAB_AMC):
                    ui.tab(name)
            with ui.tab_panels(self.tabs, value=TAB_OVERVIEW).props("animated=false keep-alive") \
                    .classes("w-full bg-transparent px-4 pb-12"):
                with ui.tab_panel(TAB_OVERVIEW):
                    self.build_home()
                with ui.tab_panel(TAB_INGEST):
                    self.build_ingest()
                with ui.tab_panel(TAB_CBR):
                    self.build_search()
                with ui.tab_panel(TAB_PREDICT):
                    self.build_predict()
                with ui.tab_panel(TAB_FC):
                    self.build_fc()
                with ui.tab_panel(TAB_SIM):
                    self.build_sim()
                with ui.tab_panel(TAB_BENCH):
                    self.build_bench()
                with ui.tab_panel(TAB_REPO):
                    self.build_browser()
                with ui.tab_panel(TAB_HW):
                    self.build_hardware()
                with ui.tab_panel(TAB_AMC):
                    self.amc = ConfigurationTab(self)
        with ui.footer().classes("atlas-footer px-6 py-2"):
            ui.label().classes("atlas-status").bind_text_from(SESSION, "status")
        ui.timer(0.2, self._drain_bench_log)
        self.refresh_all()

    # ------------------------------------------------------------------ plumbing
    def go(self, tab: str) -> None:
        self.tabs.value = tab

    def set_status(self, text: str) -> None:
        SESSION.status = text

    async def run_bg(self, fn, label: str = "working"):
        """Run ``fn(say)`` on a worker thread and return its result; raises what it raises.

        ``say(message)`` updates the status bar. The worker never touches a
        widget; the status bar reads SESSION.status on its own refresh.
        """
        SESSION.status = label + " …"

        def say(message):
            SESSION.status = str(message)

        try:
            result = await run.io_bound(fn, say)
        except Exception as exc:
            SESSION.status = f"Failed: {exc}"
            raise
        SESSION.status = "Ready"
        return result

    def alert(self, heading, body, kind="info"):
        with self.root:
            w.alert(heading, body, kind)

    async def confirm(self, heading, body, **kw) -> bool:
        with self.root:
            return await w.confirm(heading, body, **kw)

    def _header(self):
        with ui.row().classes("w-full items-end justify-between px-7 pt-5 pb-2"):
            with ui.row().classes("items-end gap-3"):
                ui.label("ATLAS").style("font-size:30px;font-weight:600;line-height:1")
                ui.label("//  ACRO CONFIGURATION INTELLIGENCE").classes("atlas-eyebrow").style("padding-bottom:4px")
            with ui.row().classes("items-center gap-3"):
                with ui.row().classes("items-center gap-2 px-3 py-1").style(
                        f"background:{w.C['panel']};border:1px solid {w.C['line']};border-radius:4px"):
                    ui.label("●").style(f"color:{w.C['green']};font-size:11px")
                    ui.label("LOCAL / OFFLINE").style("font-size:11px;font-weight:600")
                ui.button(icon="power_settings_new", on_click=self.quit).props("flat round dense color=grey-5") \
                    .tooltip("Quit ATLAS")

    async def quit(self):
        busy = SESSION.bench_proc is not None and SESSION.bench_proc.poll() is None
        if not await self.confirm("Quit ATLAS?",
                                  "Close the flight-controller link and stop ATLAS?"
                                  + ("\n\nA bench run is in progress and will be stopped." if busy else ""),
                                  yes="Quit"):
            return
        SESSION.close()
        ui.run_javascript("window.close()")
        app.shutdown()

    # =====================================================================
    # OVERVIEW
    # =====================================================================
    def build_home(self):
        with ui.column().classes("w-full gap-2 p-2"):
            w.eyebrow("SYSTEM OVERVIEW")
            w.title("Configuration Repository")
            w.sub("Trace hardware, parameter snapshots, flight evidence, and validation from one local workspace.")
            self.stats = {}
            with ui.grid(columns=5).classes("w-full gap-3 pt-3"):
                for key, label in (("air", "AIRCRAFT"), ("comp", "INSTALLED COMPONENTS"), ("hw", "CATALOG HARDWARE"),
                                   ("param", "PARAMETERS"), ("train", "TRAINABLE CASES")):
                    with w.card():
                        self.stats[key] = ui.label("0").classes("atlas-metric")
                        ui.label(label).classes("atlas-metric-label")
            with w.card("QUICK ACTIONS").classes("mt-3"):
                with ui.row().classes("gap-2"):
                    w.primary("New Configuration", lambda: self.go(TAB_INGEST), icon="add")
                    w.secondary("Find Match", lambda: self.go(TAB_CBR), icon="search")
                    for label, tab in (("Predict Tuning", TAB_PREDICT), ("Flight Controller", TAB_FC),
                                       ("Repository", TAB_REPO), ("Hardware Catalog", TAB_HW),
                                       ("Configuration", TAB_AMC)):
                        w.secondary(label, lambda t=tab: self.go(t))
            with w.card("HOW A TUNE REACHES AN AIRCRAFT").classes("mt-1"):
                ui.label("RETRIEVE OR PREDICT     →     SAFETY BOUNDS     →     YOU CONFIRM     →     MAVLINK WRITE"
                         "     →     FLIGHT EVIDENCE").style("font-weight:600;white-space:pre")
                w.sub("No model output reaches an aircraft directly. Case retrieval and the prediction model both hand "
                      "their proposal to the same fail-closed check, which rejects anything outside its approved range "
                      "or not on its list at all — then a person confirms before a single parameter is written.")
                ui.label("FULL BUILD     →     AMC HANDOFF     →     AMC CONFIGURES, CALIBRATES, TUNES     →     AMC WRITE"
                         "     →     FLIGHT EVIDENCE").style("font-weight:600;white-space:pre").classes("pt-3")
                w.sub("A whole configuration — not just the Acro tune — goes to ArduPilot Methodic Configurator as a "
                      "project. ATLAS strips the source board's calibrations and runs the same Acro bounds check; AMC "
                      "then walks every configuration step, keeps its own mandatory safety values, and asks before "
                      "each write.")

    # =====================================================================
    # ADD AIRCRAFT
    # =====================================================================
    def catalog_makes(self, category):
        with db() as c:
            return [r[0] for r in c.execute("SELECT DISTINCT manufacturer FROM hardware_catalog "
                                            "WHERE category=? AND active=1 ORDER BY manufacturer", (category,))]

    def catalog_models(self, category, make):
        with db() as c:
            return [r[0] for r in c.execute("SELECT model FROM hardware_catalog WHERE category=? AND manufacturer=? "
                                            "AND active=1 ORDER BY model", (category, make))]

    def build_ingest(self):
        f = self.f = {}
        with ui.column().classes("w-full gap-3 p-2"):
            w.title("New Aircraft Configuration")
            w.sub("Create the configuration record, attach evidence, and assign its current validation state.")
            with w.card("1. Aircraft identity"), ui.grid(columns=3).classes("w-full gap-3"):
                f["name"] = w.text_input("Aircraft name", required=True)
                f["frame_type"] = w.Combo("Frame type", FRAME_TYPES, required=True)
                f["frame_size"] = w.text_input("Frame size")
            with w.card("2. Core hardware"):
                with ui.grid(columns=3).classes("w-full gap-3"):
                    self._make_model = {}
                    for cat, key, label in (("Flight Controller", "fc", "Flight controller"),
                                            ("ESC", "esc", "ESC"), ("Motor", "motor", "Motor")):
                        required = cat == "Flight Controller"
                        make = w.Combo(f"{label} manufacturer", self.catalog_makes(cat), required=required,
                                       on_change=lambda _v, c=cat: self.update_models(c))
                        model = w.Combo(f"{label} model", [], required=required)
                        ui.element("div")                       # third column left empty
                        f[f"{key}_make"], f[f"{key}_model"] = make, model
                        self._make_model[cat] = (make, model, {"last": ""})
                    f["motor_stator"] = w.Combo("Motor stator", ["2004", "2207", "2306", "2506", "2806.5", "2812",
                                                                 "3110", "Other / Custom"])
                    f["motor_kv"] = w.text_input("Motor KV")
                    f["motor_qty"] = w.Combo("Motor quantity", ["1", "2", "4", "6", "8"], "4")
                with ui.row().classes("w-full justify-end"):
                    w.secondary("Manage hardware catalog…", lambda: self.go(TAB_HW))
            with w.card("3. Airframe measurements  —  required for parameter prediction"):
                w.sub("The prediction model learns from these. Without them an aircraft still stores and still works "
                      "for case retrieval, but it cannot train or receive a prediction.")
                with ui.grid(columns=4).classes("w-full gap-3 pt-1"):
                    f["weight_g"] = w.text_input("All-up weight (g)")
                    f["prop_size_in"] = w.Combo("Prop size (in)", ["2", "2.5", "3", "3.5", "4", "5", "6", "7", "8", "10"])
                    f["cell_count"] = w.Combo("Battery cells (S)", ["2", "3", "4", "5", "6", "8", "12"])
                    f["wheelbase_mm"] = w.text_input("Wheelbase (mm)")
            with w.card("4. Optional equipment"), ui.grid(columns=2).classes("w-full gap-3"):
                f["rx_make"] = w.text_input("Receiver manufacturer")
                f["rx_model"] = w.text_input("Receiver model")
                f["vtx_make"] = w.text_input("VTX manufacturer")
                f["vtx_model"] = w.text_input("VTX model")
                f["payload"] = w.text_input("Payload / interface")
            with w.card("5. Configuration evidence & validation"):
                with ui.row().classes("w-full items-center no-wrap gap-2"):
                    f["param_path"] = w.text_input("ArduPilot parameter file (.param)").classes("grow")
                    w.secondary("Browse…", self.pick_param)
                with ui.row().classes("w-full items-center no-wrap gap-2"):
                    f["log_path"] = w.text_input("Flight log (optional)").classes("grow")
                    w.secondary("Browse…", self.pick_log)
                with ui.grid(columns=2).classes("w-full gap-3"):
                    f["ardupilot"] = w.text_input("ArduPilot version")
                    ui.element("div")
                    f["test_date"] = w.text_input("Test date")
                    f["test_result"] = w.Combo("Test result", TEST_RESULTS, "Not evaluated")
                    f["validation"] = w.fixed_select("Validation status", VALIDATION, "UNTESTED", required=True)
                f["notes"] = ui.textarea("Notes / validation evidence").props("outlined autogrow stack-label").classes("w-full")
            with ui.row().classes("w-full justify-end pb-6"):
                w.primary("Save Configuration", self.ingest, icon="save")

    def _v(self, key) -> str:
        value = self.f[key].value
        return value.strip() if isinstance(value, str) else (value or "")

    def update_models(self, category):
        make, model, memo = self._make_model[category]
        models = self.catalog_models(category, make.value)
        model.set_options(models)
        # Clear the model only when the manufacturer actually changed: a model
        # typed by hand for a make not yet in the catalog must not vanish.
        if memo["last"] and make.value != memo["last"] and model.value not in models:
            model.value = ""
        memo["last"] = make.value

    def refresh_ingest_catalog(self):
        for cat, (make, _model, _memo) in self._make_model.items():
            make.set_options(self.catalog_makes(cat))
            self.update_models(cat)

    async def pick_param(self):
        p = await w.pick_file("Select ArduPilot parameter file", PARAM_PATTERNS, memory="params")
        if p:
            self.f["param_path"].value = str(p)

    async def pick_log(self):
        p = await w.pick_file("Select flight log", LOG_PATTERNS, memory="logs")
        if p:
            self.f["log_path"].value = str(p)

    @staticmethod
    def add_component(c, aid, cat, make, model, specs=""):
        if any(str(x).strip() for x in (make, model, specs)):
            c.execute("INSERT INTO components(aircraft_id,category,manufacturer,model,specs) VALUES(?,?,?,?,?)",
                      (aid, cat, str(make).strip(), str(model).strip(), str(specs).strip()))

    def ingest(self):
        v = self._v
        name, frame, fc_model = v("name"), v("frame_type"), v("fc_model")
        if not name or not frame or not fc_model:
            self.alert("Missing required information",
                       "Aircraft name, frame type, and flight controller are required.", "warn")
            return
        try:
            def meas(key):
                raw = v(key)
                if not raw:
                    return None
                try:
                    return float(raw)
                except ValueError:
                    raise ValueError(f"{key.replace('_', ' ')}: {raw!r} is not a number") from None
            w_g, prop_in, cells, wheel = meas("weight_g"), meas("prop_size_in"), meas("cell_count"), meas("wheelbase_mm")
            param_path, log_path, notes = v("param_path"), v("log_path"), v("notes")
            for label, path in (("Parameter file", param_path), ("Flight log", log_path)):
                if path and not Path(path).is_file():
                    raise ValueError(f"{label} not found:\n{path}")
            with db() as c:
                aid = c.execute(
                    "INSERT INTO aircraft(name,frame_type,frame_size,validation_status,notes,created_at,weight_g,"
                    "prop_size_in,cell_count,wheelbase_mm) VALUES(?,?,?,?,?,?,?,?,?,?)",
                    (name, frame, v("frame_size"), v("validation"), notes, now_iso(), w_g, prop_in,
                     int(cells) if cells else None, wheel)).lastrowid
                self.add_component(c, aid, "Flight Controller", v("fc_make"), v("fc_model"))
                self.add_component(c, aid, "ESC", v("esc_make"), v("esc_model"))
                ms = ", ".join(x for x in [("Stator " + v("motor_stator")) if v("motor_stator") else "",
                                           (v("motor_kv") + " KV") if v("motor_kv") else "",
                                           ("Qty " + v("motor_qty")) if v("motor_qty") else ""] if x)
                self.add_component(c, aid, "Motor", v("motor_make"), v("motor_model"), ms)
                self.add_component(c, aid, "Receiver", v("rx_make"), v("rx_model"))
                self.add_component(c, aid, "VTX", v("vtx_make"), v("vtx_model"))
                self.add_component(c, aid, "Payload", "", v("payload"))
                pf = None
                if param_path:
                    src = Path(param_path)
                    dest = FILES_DIR / f"{aid}_{src.name}"
                    shutil.copy2(src, dest)
                    pf = c.execute("INSERT INTO parameter_files(aircraft_id,original_name,stored_path,ardupilot_version,"
                                   "imported_at) VALUES(?,?,?,?,?)",
                                   (aid, src.name, str(dest), v("ardupilot"), now_iso())).lastrowid
                    c.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",
                                  [(pf, a, b) for a, b in parse_param(src)])
                if log_path:
                    src = Path(log_path)
                    dest = FILES_DIR / f"{aid}_{src.name}"
                    shutil.copy2(src, dest)
                    c.execute("INSERT INTO flight_tests(aircraft_id,parameter_file_id,log_name,stored_path,test_date,"
                              "result,notes) VALUES(?,?,?,?,?,?,?)",
                              (aid, pf, src.name, str(dest), v("test_date"), v("test_result"), notes))
            self.set_status(f"Saved {name} • {v('validation')}")
            self.alert("Configuration saved", f"{name} was added to ATLAS.", "ok")
            self.clear_ingest()
            self.refresh_all()
        except sqlite3.IntegrityError:
            self.alert("Aircraft already exists", "Use a unique aircraft name.", "error")
        except Exception as e:  # noqa: BLE001
            self.alert("Could not save configuration", str(e), "error")

    def clear_ingest(self):
        for key, field in self.f.items():
            if key != "validation":                 # a fixed list: never blank
                field.value = ""
        self.f["motor_qty"].value = "4"
        self.f["validation"].value = "UNTESTED"
        self.f["test_result"].value = "Not evaluated"
        for _make, _model, memo in self._make_model.values():
            memo["last"] = ""

    # =====================================================================
    # CBR ASSISTANT
    # =====================================================================
    def build_search(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.title("Explainable Case-Based Reasoning")
            w.sub("RETRIEVE → REUSE → REVISE → RETAIN  |  Similarity is not flight validation.")
            self.search_fields = {}
            with w.card("1. Describe target aircraft"), ui.grid(columns=4).classes("w-full gap-3"):
                for key, label in (("frame", "Frame type"), ("frame_size", "Frame size"),
                                   ("fc", "Flight controller make/model"), ("esc", "ESC make/model"),
                                   ("motor", "Motor make/model"), ("kv", "Motor KV"), ("stator", "Motor stator"),
                                   ("quantity", "Motor quantity")):
                    self.search_fields[key] = (w.Combo(label, FRAME_TYPES[:-1]) if key == "frame"
                                               else w.text_input(label))
            with ui.row().classes("gap-2"):
                w.primary("Rank Similar Cases", self.search, icon="sort")
                w.secondary("Adjust Weights", self.configure_weights)
                w.secondary("Export Candidate .param", self.export_candidate, icon="download")
            self.match_table = w.DataTable([("score", "Similarity"), ("name", "Source aircraft"),
                                            ("status", "Evidence status"), ("files", "Param files")],
                                           height="16rem", on_select=lambda _rows: self.show_match())
            self.result = w.TextPane("20rem")
            self.result.set_text("Enter known hardware and select Rank Similar Cases. Unknown fields are excluded "
                                 "from the score.\nNo result constitutes a safety or compatibility certification.")

    def search(self):
        target = {k: (f.value or "").strip() for k, f in self.search_fields.items()}
        if not any(target.values()):
            self.alert("Missing target", "Enter at least one hardware attribute.", "warn")
            return
        with db() as c:
            self.matches = cbr.retrieve(c, target, self.weights)
        self.match_table.set_rows([{"id": r["id"], "score": f"{r['score']:.1f}%", "name": r["name"],
                                    "status": r["status"], "files": r["files"]} for r in self.matches],
                                  keep_selection=False)
        if not self.matches:
            self.result.set_text("No saved cases yet. Add aircraft and parameter snapshots first.")
            return
        self.match_table.select_key(self.matches[0]["id"])

    def _selected_match(self):
        key = self.match_table.selected_key
        return next((r for r in self.matches if r["id"] == key), None)

    def show_match(self):
        r = self._selected_match()
        if not r:
            return
        t = self.result
        t.set_text(f"SOURCE: {r['name']}\nSIMILARITY: {r['score']}% (weighted retrieval only)\n"
                   f"VALIDATION OF SOURCE: {r['status']}\nPARAMETER SNAPSHOTS: {r['files']}\n\nFEATURE EXPLANATIONS\n")
        for d in r["details"]:
            t.write(f"{d['field'].upper():12} {d['score']:3}%  {d['reason']}\n"
                    f"   target: {d['target']} | stored: {d['case'] or '(missing)'}\n")
        t.write("\nCandidate parameters are unvalidated for the target aircraft. Check firmware, hardware mapping, "
                "failsafes, and flight-dependent tuning before use.\n", "warn")

    async def configure_weights(self):
        with self.root, ui.dialog().props("persistent") as dialog, ui.card().classes("atlas-dialog gap-2"):
            ui.label("Retrieval Feature Weights").classes("atlas-dialog-title")
            w.sub("Relative importance; zero excludes a feature.")
            boxes = {}
            with ui.grid(columns=2).classes("w-full gap-2"):
                for key, val in self.weights.items():
                    boxes[key] = ui.number(key.upper(), value=val, min=0, step=0.5).props("outlined dense")
            err = ui.label("").classes("t-bad text-sm")

            def save():
                try:
                    new = {k: float(b.value) for k, b in boxes.items()}
                except (TypeError, ValueError):
                    err.text = "Every weight must be a number."
                    return
                if any(v < 0 for v in new.values()) or not any(new.values()):
                    err.text = "Weights must be nonnegative and at least one positive."
                    return
                dialog.submit(new)

            with ui.row().classes("w-full justify-end gap-2"):
                w.secondary("Cancel", lambda: dialog.submit(None))
                w.primary("Apply Weights", save)
        new = await dialog
        dialog.delete()
        if new:
            self.weights = new
            self.search()

    async def export_candidate(self):
        r = self._selected_match()
        if not r:
            self.alert("Select a case", "Rank cases and select a source first.")
            return
        try:
            with db() as c:
                pf, params, warnings = cbr.candidate(c, r["id"])
            # The Acro tuning parameters inside the snapshot are checked against
            # safety.py before leaving the application.
            acro_in_file = {k: v for k, v in params.items() if k in safety.ACRO_PARAM_RANGES}
            check = safety.check_param_set(acro_in_file)
            if check["blocked"]:
                lines = "\n".join("   • " + b["message"] for b in check["blocked"])
                if not await self.confirm(
                        "Acro parameters outside approved range",
                        f"{len(check['blocked'])} of {len(acro_in_file)} Acro tuning parameters in this snapshot fall "
                        f"outside the range this tool treats as safe:\n\n{lines}\n\nThese are the source aircraft's "
                        "real stored values, so this may mean the approved range needs widening for this class of "
                        "airframe — or that the snapshot is wrong.\n\nExport anyway, with these flagged in the review "
                        "report?", yes="Export Anyway"):
                    return
            out = await w.pick_save("Save unvalidated candidate", "ATLAS_CANDIDATE_UNVALIDATED.param", ("*.param",))
            if not out:
                return
            if out.suffix.lower() != ".param":
                out = out.with_suffix(".param")
            out.write_text(f"# ATLAS v{APP_VERSION} CANDIDATE - NOT FLIGHT VALIDATED\n# Source case: " + r["name"]
                           + f"\n# Similarity {r['score']}% is a retrieval score, not a safety score\n"
                           f"# Acro tuning parameters within approved range: {len(check['writable'])}/{len(acro_in_file)}\n"
                           + "".join(f"{k},{v}\n" for k, v in params.items()), encoding="utf-8")
            report = {"candidate_status": "UNTESTED", "source_aircraft": r["name"], "source_validation": r["status"],
                      "similarity_percent": r["score"], "source_parameter_file": pf["original_name"],
                      "source_firmware": pf["ardupilot_version"], "features": r["details"], "warnings": warnings,
                      "acro_safety_check": {"checked": len(acro_in_file), "in_range": len(check["writable"]),
                                            "blocked": [b["message"] for b in check["blocked"]]},
                      "required_review": ["Confirm target firmware and vehicle type",
                                          "Review frame, motor and servo mappings",
                                          "Review arming and failsafe settings",
                                          "Tune flight-dependent parameters using appropriate tests",
                                          "Perform bench, SITL and controlled flight validation"]}
            out.with_suffix(".review.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
            self.alert("Candidate exported",
                       f"Saved {len(params)} parameters and {len(warnings)} review flags.\nAcro tuning parameters "
                       f"within approved range: {len(check['writable'])}/{len(acro_in_file)}\n\n{out}\n"
                       f"{out.with_suffix('.review.json')}\n\nNOT flight validated.", "ok")
        except Exception as e:  # noqa: BLE001
            self.alert("Export failed", str(e), "error")

    # =====================================================================
    # PREDICT — the multi-output random forest
    # =====================================================================
    def build_predict(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.eyebrow("PARAMETER PREDICTION")
            w.title("Multi-Output Regression")
            w.sub("Predicts a whole Acro tuning set at once from hardware measurements, learning only from "
                  "configurations this repository records as having flown well.")
            with w.card("TRAINING DATA"):
                self.census_label = w.sub().style("white-space:pre-wrap")
                self.census_detail = w.sub().style("white-space:pre-wrap")
            with w.card("TARGET AIRCRAFT"):
                self.pred_fields = {}
                with ui.grid(columns=6).classes("w-full gap-3"):
                    for feat in bridge.FEATURE_FIELDS:
                        self.pred_fields[feat] = w.text_input(bridge.FEATURE_LABELS[feat])
                with ui.row().classes("items-center gap-2 pt-2"):
                    w.sub("Or load the measurements from a stored aircraft:")
                    self.pred_source = w.fixed_select("Stored aircraft", [], width="w-80")
                    w.secondary("Load", self.load_pred_features)
            with ui.row().classes("items-center gap-2"):
                w.primary("Predict Parameters", self.do_predict, icon="auto_graph")
                w.secondary("Send to Flight Controller →", self.prediction_to_fc)
                w.secondary("Export .param", self.export_prediction, icon="download")
                self.include_sim = ui.checkbox("Include SITL runs in training", value=False).classes("pl-4")
            self.pred_verdict = ui.label("").classes("atlas-verdict")
            self.pred_verdict.set_visibility(False)
            self.pred_out = w.TextPane("26rem")
            self.pred_out.set_text("Enter the target aircraft's measurements and select Predict Parameters.\n\n"
                                   "The model proposes; safety bounds decide what may leave this application; you "
                                   "confirm before anything is written to an aircraft.\n")

    def refresh_predict(self):
        with db() as c:
            cen = bridge.census(c)
            names = [(r["id"], r["name"]) for r in c.execute("SELECT id,name FROM aircraft ORDER BY name")]
        need = max(0, ml_model.MIN_TRAINING_ROWS - cen["trainable"])
        self.census_label.text = (
            f"{cen['trainable']} aircraft usable for training   •   {cen['aircraft_total']} in repository   •   "
            f"{cen['aircraft_with_params']} with parameter snapshots   •   {cen['sim_runs']} completed SITL runs"
            + (f"\nNeed {need} more usable configuration(s) before prediction is possible (minimum "
               f"{ml_model.MIN_TRAINING_ROWS}). Case retrieval on the CBR tab works from a single stored case."
               if need else "\nEnough data to train. Treat predictions as an interpolation between similar builds, "
                            "not a validated tune."))
        lines = [f"Not usable — {s['name']}: {s['reason']}" for s in cen["skipped"][:6]]
        if len(cen["skipped"]) > 6:
            lines.append(f"… and {len(cen['skipped']) - 6} more")
        self.census_detail.text = "\n".join(lines)
        self._pred_sources = {f"#{i} • {n}": i for i, n in names}
        keep = self.pred_source.value
        self.pred_source.set_options(list(self._pred_sources), value=keep if keep in self._pred_sources else None)

    def load_pred_features(self):
        key = self.pred_source.value
        if not key or key not in self._pred_sources:
            self.alert("Select an aircraft", "Choose a stored aircraft to copy measurements from.")
            return
        with db() as c:
            feats, prov = bridge.extract_features(c, self._pred_sources[key])
        for feat in bridge.FEATURE_FIELDS:
            self.pred_fields[feat].value = f"{feats[feat]:g}" if feat in feats else ""
        t = self.pred_out
        t.set_text(f"Loaded measurements from {key}\n\nWHERE EACH VALUE CAME FROM\n")
        for line in prov:
            t.write("  " + line + "\n", "warn" if "INFERRED" in line else "")
        miss = bridge.missing_features(feats)
        if miss:
            t.write("\nStill missing: " + ", ".join(bridge.FEATURE_LABELS[m] for m in miss)
                    + "\nFill these in by hand, or edit the aircraft to record them.\n", "bad")

    def _collect_pred_features(self):
        feats = {}
        for feat in bridge.FEATURE_FIELDS:
            raw = (self.pred_fields[feat].value or "").strip()
            if not raw:
                continue
            try:
                feats[feat] = float(raw)
            except ValueError:
                self.alert("Not a number", f"{bridge.FEATURE_LABELS[feat]}: {raw!r} is not a number.", "warn")
                return None
        return feats

    def _set_verdict(self, text, colour):
        self.pred_verdict.text = text
        self.pred_verdict.style(f"color:{colour};border-left-color:{colour}")
        self.pred_verdict.set_visibility(True)

    def do_predict(self):
        feats = self._collect_pred_features()
        if feats is None:
            return
        missing = bridge.missing_features(feats)
        if missing:
            self.alert("Missing measurements", "Enter a value for:\n\n  "
                       + "\n  ".join(bridge.FEATURE_LABELS[m] for m in missing), "warn")
            return
        with db() as c:
            result = ml_model.recommend(c, feats, include_sim=bool(self.include_sim.value))
        SESSION.last_prediction = result if result.get("ok") else None
        t = self.pred_out
        t.clear()
        if not result.get("ok"):
            self._set_verdict("PREDICTION NOT POSSIBLE", w.C["red"])
            t.write("PREDICTION NOT POSSIBLE\n\n", "bad")
            t.write(result["error"] + "\n")
            return
        t.write(f"Trained on {result['n_training']} configuration(s) ({', '.join(result['training_sources'])})\n\n")
        if not result["in_training_range"]:
            t.write("OUTSIDE TRAINED RANGE\n", "bad")
            for warning in result["range_warnings"]:
                t.write("  ! " + warning + "\n", "warn")
            t.write("\n")
        t.write("PREDICTED PARAMETERS\n")
        t.write(f"  {'parameter':<18}{'value':>14}{'tree spread':>16}   note\n")
        for name, value in result["params"].items():
            spread = result["spread"].get(name)
            derived = " derived from " + bridge.DERIVED_PARAMS[name][0] if name in bridge.DERIVED_PARAMS else ""
            sp = f"±{spread:.4g}" if spread else ""
            t.write(f"  {name:<18}{value:>14.6g}{sp:>16}  {derived}\n")
        if result["clamp_notes"]:
            t.write("\nPULLED INTO RANGE\n", "warn")
            for n in result["clamp_notes"]:
                t.write("  " + n + "\n", "warn")
        v = result["validation"]
        t.write(f"\nSAFETY CHECK: {'all parameters within approved range' if v['all_valid'] else 'BLOCKED'}\n",
                "good" if v["all_valid"] else "bad")
        for b in v["blocked"]:
            t.write("  • " + b["message"] + "\n", "bad")
        if result["oob"]:
            t.write("\nOUT-OF-BAG ERROR (each aircraft predicted only by trees that never saw it)\n")
            for k, val in result["oob"].items():
                t.write(f"  {k:<18}{val:>8.1%} mean relative error\n")
        t.write("\nFEATURE IMPORTANCE\n")
        for k, val in result["importance"]:
            t.write(f"  {bridge.FEATURE_LABELS.get(k, k):<20}{val:>7.0%}\n")
        t.write("\nThis is an interpolation between stored builds, not a validated tune. Bench test, then simulate, "
                "then fly it progressively.\n", "warn")
        # The headline verdict, always visible without scrolling.
        if not v["all_valid"]:
            self._set_verdict(f"BLOCKED — {len(v['blocked'])} parameter(s) outside the approved range. "
                              "Nothing can be written.", w.C["red"])
        elif not result["in_training_range"]:
            self._set_verdict(f"{len(result['params'])} parameters within approved range, but the target is OUTSIDE "
                              "the trained range — see the warnings below.", w.C["amber"])
        else:
            worst = max(((k, sv / abs(result["params"][k])) for k, sv in result["spread"].items()
                         if result["params"].get(k)), key=lambda kv: kv[1], default=None)
            extra = f"  Widest tree disagreement: {worst[0]} ±{worst[1]:.0%}." if worst else ""
            self._set_verdict(f"{len(result['params'])} parameters within approved range, target inside the "
                              f"trained range.{extra}", w.C["green"])

    async def export_prediction(self):
        r = SESSION.last_prediction
        if not r:
            self.alert("Nothing to export", "Run a prediction first.")
            return
        out = await w.pick_save("Save predicted parameters", "ATLAS_PREDICTED_UNVALIDATED.param", ("*.param",))
        if not out:
            return
        if out.suffix.lower() != ".param":
            out = out.with_suffix(".param")
        out.write_text(f"# ATLAS v{APP_VERSION} PREDICTED TUNE - NOT FLIGHT VALIDATED\n"
                       f"# Trained on {r['n_training']} configuration(s): {', '.join(r['training_sources'])}\n"
                       f"# Within trained range: {r['in_training_range']}\n"
                       + "".join(f"{k},{v:.8g}\n" for k, v in r["params"].items()), encoding="utf-8")
        out.with_suffix(".review.json").write_text(json.dumps({
            "candidate_status": "UNTESTED", "origin": "multi-output random forest",
            "n_training": r["n_training"], "training_sources": r["training_sources"],
            "in_training_range": r["in_training_range"], "range_warnings": r["range_warnings"],
            "clamped": r["clamp_notes"], "tree_spread": r["spread"], "out_of_bag_relative_error": r["oob"],
            "feature_importance": dict(r["importance"]), "parameters": r["params"]}, indent=2), encoding="utf-8")
        self.set_status(f"Exported predicted tune • {out.name}")
        self.alert("Predicted tune exported", f"{out}\n{out.with_suffix('.review.json')}\n\nNOT flight validated.", "ok")

    def prediction_to_fc(self):
        r = SESSION.last_prediction
        if not r:
            self.alert("Nothing to send", "Run a prediction first.")
            return
        SESSION.fc_pending = dict(r["params"])
        SESSION.fc_origin = f"random forest, {r['n_training']} training configs"
        self.go(TAB_FC)
        self.refresh_fc_pending()
        self.set_status("Prediction staged for the flight controller — review and confirm there.")

    # =====================================================================
    # FLIGHT CONTROLLER — the only tab that can change an aircraft
    # =====================================================================
    def build_fc(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.eyebrow("HARDWARE LINK")
            w.title("Apply Parameters")
            w.sub("The only screen in ATLAS that writes to an aircraft. Every value is re-checked against the approved "
                  "range here, whatever produced it, and nothing is written while the aircraft is armed.")
            presets = dict(fc.DEFAULT_CONNECTIONS)
            with w.card("CONNECTION"):
                with ui.row().classes("w-full items-center gap-2"):
                    self.fc_preset = w.fixed_select("Preset", list(presets), fc.DEFAULT_CONNECTIONS[0][0], width="w-64",
                                                    on_change=lambda e: setattr(self.fc_conn, "value",
                                                                                presets.get(e.value, self.fc_conn.value)))
                    self.fc_conn = w.text_input("Connection string", fc.DEFAULT_CONNECTIONS[0][1], width="w-80")
                    w.primary("Connect", self.fc_connect, icon="link")
                    w.secondary("Disconnect", self.fc_disconnect)
                    w.secondary("Test Connection…", self.fc_test_link)
                self.fc_status = w.sub("Not connected.").classes("pt-1")
            with w.card("STAGED PARAMETERS"):
                with ui.row().classes("w-full items-center gap-2"):
                    w.secondary("Load from .param file", self.fc_load_file, icon="upload_file")
                    w.secondary("Read current from aircraft", self.fc_read_current)
                    w.secondary("Clear", self.fc_clear)
                    ui.space()
                    w.primary("APPLY TO AIRCRAFT", self.fc_apply, icon="flight_takeoff")
            self.fc_table = w.DataTable([("param", "Parameter"), ("value", "Staged value"),
                                         ("verdict", "Safety verdict"), ("range", "Approved range")],
                                        row_key="param", height="26rem")
            self.fc_note = w.sub(NOTHING_STAGED)

    def fc_clear(self):
        SESSION.fc_pending = {}
        self.refresh_fc_pending()

    def refresh_fc_pending(self):
        if SESSION.vehicle is None:
            self.fc_status.text = "Not connected."
        if not SESSION.fc_pending:
            self.fc_table.set_rows([])
            self.fc_note.text = NOTHING_STAGED
            return
        report = safety.check_param_set(SESSION.fc_pending)
        verdicts = {safety.OK: "will be written", safety.OUT_OF_RANGE: "BLOCKED — outside approved range",
                    safety.NOT_A_NUMBER: "BLOCKED — not a number"}
        rows = []
        for r in report["results"]:
            verdict = verdicts.get(r["verdict"]) or "BLOCKED — " + r["message"].split(": ", 1)[-1]
            rows.append({"param": r["name"],
                         "value": f"{r['value']:.6g}" if isinstance(r["value"], float) else str(r["value"]),
                         "verdict": verdict,
                         "range": f"[{r['low']:g}, {r['high']:g}]" if r["low"] is not None else "—",
                         "_tag": "good" if r["verdict"] == safety.OK else "bad"})
        self.fc_table.set_rows(rows)
        self.fc_note.text = (f"{len(report['writable'])} parameter(s) will be written; {len(report['blocked'])} blocked. "
                             f"Origin: {SESSION.fc_origin}. "
                             + ("Blocked parameters are never sent — not clamped, not silently skipped."
                                if report["blocked"] else "All staged parameters are within the approved range."))

    async def fc_connect(self):
        ok, msg = fc.available()
        if not ok:
            self.fc_status.text = msg
            self.alert("pymavlink not installed", msg, "error")
            return
        conn_str = (self.fc_conn.value or "").strip()
        if SESSION.vehicle is not None:
            self.fc_disconnect()
        vehicle = fc.Vehicle(conn_str)
        try:
            info = await self.run_bg(lambda say: vehicle.connect(progress=say), f"Connecting to {conn_str}")
        except Exception as exc:  # noqa: BLE001
            SESSION.vehicle = None
            self.fc_status.text = str(exc)
            self.alert("Could not connect", str(exc), "error")
            return
        SESSION.vehicle = vehicle
        frame = f"FRAME_CLASS={info.get('FRAME_CLASS')}, FRAME_TYPE={info.get('FRAME_TYPE')}"
        self.fc_status.text = (f"Connected to {conn_str}  •  system {info['system_id']} / component "
                               f"{info['component_id']}  •  {'ARMED' if info['armed'] else 'disarmed'}  •  {frame}")
        self.set_status(f"Connected • {conn_str}")

    async def fc_test_link(self):
        """Run the link diagnostic and show the report.

        A UDP timeout looks identical whether the simulator is off, aimed at the
        wrong address, firewalled, or on a network that will not route — so the
        report separates those rather than repeating "no heartbeat".
        """
        conn = (self.fc_conn.value or "").strip()
        port = 14550
        if ":" in conn:
            tail = conn.rsplit(":", 1)[-1]
            if tail.isdigit():
                port = int(tail)
        with self.root, ui.dialog() as dialog, ui.card().classes("atlas-dialog gap-2").style("width:860px"):
            ui.label("Connection Diagnostic").classes("atlas-dialog-title")
            w.sub(f"Testing {conn}")
            out = w.TextPane("60vh")
            with ui.row().classes("w-full justify-end"):
                w.primary("Close", dialog.close)
        dialog.on("hide", dialog.delete)
        dialog.open()
        if not conn.startswith(("udp:", "udpin:", "udpout:")):
            out.set_text("This diagnostic only tests network (UDP) links.\n\n"
                         f"'{conn}' is a serial port or TCP address. For a serial port, check the cable, the port "
                         "name, the baud rate, and that no other program (Mission Planner, QGroundControl) is "
                         "holding it.\n")
            return
        ok, msg = fc.available()
        if not ok:
            out.set_text(msg + "\n", "bad")
            return
        out.set_text("Running …\n\n")
        try:
            report = await self.run_bg(
                lambda say: check_link.diagnose(port=port, conn=conn if conn.startswith("udpin:") else None,
                                                seconds=6.0), "Testing the link")
        except Exception as exc:  # noqa: BLE001
            out.write(f"\nDiagnostic failed: {exc}\n", "bad")
            return
        if out.is_deleted:            # the operator closed the window while it ran
            return
        out.set_lines([(line, "") for line in report["lines"]])
        out.write("\n" + "=" * 62 + "\n")
        if report["ok"]:
            out.write("LINK IS WORKING. Use this connection string:\n", "good")
            out.write(f"  {report['connection']}\n", "good")
            if report["connection"] != conn:
                out.write("\n(that differs from what is in the field — update it)\n", "bad")
        else:
            out.write("NO LINK. Work through these in order:\n", "bad")
            for i, step in enumerate(check_link.FAILURE_STEPS, 1):
                out.write(f"  {i}. {step}\n")

    def fc_disconnect(self):
        if SESSION.vehicle is not None:
            try:
                SESSION.vehicle.close()
            except Exception:  # noqa: BLE001
                pass
        SESSION.vehicle = None
        self.fc_status.text = "Not connected."
        self.set_status("Disconnected")

    async def fc_load_file(self):
        path = await w.pick_file("Load parameter file", PARAM_PATTERNS, memory="params")
        if not path:
            return
        rows = parse_param(path)
        acro = {k: v for k, v in rows if k in safety.ACRO_PARAM_RANGES}
        if not acro:
            self.alert("No tunable parameters",
                       f"{path.name} contains {len(rows)} parameters but none that this tool tunes.\n\n"
                       "ATLAS only writes Acro rate-controller and stick-feel parameters. Everything else belongs in "
                       "Mission Planner.")
            return
        staged = {}
        for k, v in acro.items():
            try:
                staged[k] = float(v)
            except ValueError:
                staged[k] = v          # shown as BLOCKED — not a number, never sent
        SESSION.fc_pending, SESSION.fc_origin = staged, f"file: {path.name}"
        self.refresh_fc_pending()
        self.set_status(f"Staged {len(staged)} of {len(rows)} parameters from {path.name}")

    async def fc_read_current(self):
        vehicle = SESSION.vehicle
        if vehicle is None:
            self.alert("Not connected", "Connect to a flight controller first.")
            return
        try:
            values = await self.run_bg(lambda say: vehicle.read_acro_params(progress=say), "Reading current parameters")
        except Exception as exc:  # noqa: BLE001
            self.alert("Read failed", str(exc), "error")
            return
        got = {k: v for k, v in values.items() if v is not None}
        SESSION.fc_pending, SESSION.fc_origin = got, "read from aircraft"
        self.refresh_fc_pending()
        missed = [k for k, v in values.items() if v is None]
        self.set_status(f"Read {len(got)} parameters" + (f"; {len(missed)} did not respond" if missed else ""))

    async def fc_apply(self):
        vehicle = SESSION.vehicle
        if vehicle is None:
            self.alert("Not connected", "Connect to a flight controller first.")
            return
        if not SESSION.fc_pending:
            self.alert("Nothing staged", "Stage some parameters first.")
            return
        report = safety.check_param_set(SESSION.fc_pending)
        if not report["writable"]:
            self.alert("Nothing can be written", "Every staged parameter was blocked by the approved range.", "error")
            return
        conn = self.fc_conn.value
        lines = "\n".join(f"   {k} = {v:g}" for k, v in sorted(report["writable"].items()))
        blocked = ("\n\nBLOCKED (will NOT be sent):\n" + "\n".join("   " + b["message"] for b in report["blocked"])
                   if report["blocked"] else "")
        if not await self.confirm("Write parameters to the aircraft?",
                                  f"About to write {len(report['writable'])} parameter(s) to {conn}:\n\n{lines}{blocked}"
                                  "\n\nPropellers should be OFF. The aircraft must be disarmed.\n\nProceed?",
                                  yes="Write to Aircraft"):
            return

        def confirmed(_report):
            # Runs on the worker thread. The operator already confirmed above;
            # this is the interface Vehicle.apply_params demands and it must stay truthful.
            return bool(SESSION.fc_confirmed)

        SESSION.fc_confirmed = True
        aircraft_id = self.selected_repo_aircraft()
        pending, origin = dict(SESSION.fc_pending), SESSION.fc_origin
        try:
            result = await self.run_bg(lambda say: vehicle.apply_params(pending, confirm=confirmed, progress=say),
                                       "Writing parameters")
        except Exception as exc:  # noqa: BLE001
            self.alert("Apply failed", str(exc), "error")
            return
        finally:
            SESSION.fc_confirmed = False
        if not result["applied"]:
            self.alert("Not applied", result["reason"] or "No parameters were written.", "warn")
            return
        with db() as c:
            bridge.record_application(c, aircraft_id, conn, result["validation"]["writable"], result["confirmed"],
                                      origin, "")
        msg = f"Wrote and confirmed {len(result['confirmed'])} parameter(s)."
        if result.get("failures"):
            msg += f"\n\n{len(result['failures'])} failed:\n" + "\n".join("  " + f for f in result["failures"])
        msg += "\n\nRecorded in the audit trail. Bench test before flying."
        self.alert("Parameters applied", msg, "ok")
        self.set_status(msg.splitlines()[0])
        self.refresh_all()

    def selected_repo_aircraft(self) -> int | None:
        return self.repo_table.selected_key

    def open_folder(self, folder: Path, heading: str):
        folder.mkdir(parents=True, exist_ok=True)
        try:
            open_in_file_browser(folder)
        except Exception as exc:  # noqa: BLE001
            self.alert(heading, f"{folder}\n\n(could not open a file browser here: {exc})")

    # =====================================================================
    # SIMULATION — iterate candidate tunes in SITL
    # =====================================================================
    def build_sim(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.eyebrow("SITL EVALUATION")
            w.title("Iterate Candidate Tunes")
            w.sub("Flies a scripted Acro sequence in ArduPilot SITL once per candidate and scores how closely the "
                  "measured body rates followed the commanded rates. Requires a running simulator and pymavlink.")
            with w.card("SWEEP"):
                with ui.grid(columns=4).classes("w-full gap-3"):
                    self.sim_param = w.fixed_select("Parameter to sweep", list(bridge.TARGET_PARAMS), "ATC_RAT_RLL_P")
                    self.sim_low = w.text_input("From", "0.03")
                    self.sim_high = w.text_input("To", "0.12")
                    self.sim_steps = w.text_input("Steps", "5")
                    self.sim_conn = w.text_input("Simulator connection", "udp:127.0.0.1:14550")
                ui.label("Everything not being swept is held at the baseline tune.\n"
                         "SITL on THIS computer:      connection  udp:127.0.0.1:14550      and   "
                         "sim_vehicle.py -v ArduCopter -f quad --out=udp:127.0.0.1:14550\n"
                         "SITL on ANOTHER computer:   connection  udpin:0.0.0.0:14550      and   "
                         "sim_vehicle.py -v ArduCopter -f quad --out=udp:THIS_PC_IP:14550\n"
                         "127.0.0.1 is loopback and can never reach another machine. Run check_link.py to test the "
                         "link on its own.").classes("atlas-sub pt-2") \
                    .style("white-space:pre-wrap;font-family:Consolas,Menlo,monospace;font-size:12px")
            with w.card("FLIGHT LOGS"):
                with ui.row().classes("gap-6"):
                    self.sim_tlog = ui.checkbox("Record .tlog per run", value=True)
                    self.sim_bin = ui.checkbox("Download .bin from autopilot (slow)", value=False)
                    self.sim_file_evidence = ui.checkbox("File each run as flight evidence", value=True)
                w.sub(".tlog is what this program received — cheap, works over any link. .bin is the autopilot's own "
                      "high-rate log, which Mission Planner reads; it is fetched over MAVLink, which is slow but is the "
                      "only route when SITL runs on another computer. Filing needs an aircraft selected on the "
                      "Repository tab; evidence is marked source=sitl and never changes an airframe's validation "
                      "status.")
            with ui.row().classes("w-full items-center gap-2"):
                self.sim_run_btn = w.primary("Run Sweep", self.run_sweep, icon="play_arrow")
                w.secondary("Refresh Results", self.refresh_sim, icon="refresh")
                w.secondary("Open Log Folder", lambda: self.open_folder(LOG_DIR, "Flight log folder"), icon="folder_open")
                ui.space()
                self.sim_evidence_label = w.sub()
            # Bench scores and Acro-sweep scores are different measures; "Type" keeps them apart.
            self.sim_table = w.DataTable([("score", "Score"), ("type", "Type"), ("group", "Run group"),
                                          ("params", "Parameters"), ("logs", "Logs filed"), ("note", "Status")],
                                         height="24rem")
            w.sub("Scores compare runs within one sweep on one simulated airframe. SITL has no frame resonance or motor "
                  "lag unless you configure SIM_* to match, so a winning tune here is a starting point, not a "
                  "validated one.")

    def refresh_sim(self):
        with db() as c:
            rows = c.execute("SELECT * FROM sim_runs ORDER BY outcome_score DESC, id DESC LIMIT 200").fetchall()
            summary = sim_logs.sim_flight_summary(c)
            # One pass for the log counts rather than a query per row: the run
            # group is recorded in the evidence notes when a run is filed.
            counts = {}
            for ft in c.execute("SELECT notes FROM flight_tests WHERE source=?", (bridge.SOURCE_SIM,)):
                for line in (ft["notes"] or "").splitlines():
                    if line.startswith("Run group: "):
                        g = line.split(": ", 1)[1].strip()
                        counts[g] = counts.get(g, 0) + 1
        table = []
        for r in rows:
            try:
                params = json.loads(r["params_json"])
            except (TypeError, ValueError):
                params = {}
            shown = ", ".join(f"{k}={v:g}" for k, v in sorted(params.items()) if k in bridge.TARGET_PARAMS)
            nlogs = counts.get(r["run_group"], 0)
            table.append({"id": r["id"],
                          "score": f"{r['outcome_score']:.4f}" if r["outcome_score"] is not None else "—",
                          "type": "Standard bench" if r["run_group"].startswith(BENCH_GROUP_PREFIX) else "Acro sweep",
                          "group": r["run_group"], "params": shown[:130],
                          "logs": f"{nlogs} file(s)" if nlogs else "—",
                          "note": (r["abort_reason"] or "completed")[:50]})
        self.sim_table.set_rows(table)
        self.sim_evidence_label.text = (
            f"{summary['sim_flight_tests']} simulated log(s) filed  •  mean score {summary['mean_sim_score']}"
            if summary["mean_sim_score"] is not None else f"{summary['sim_flight_tests']} simulated log(s) filed")

    async def run_sweep(self):
        # Validate the sweep BEFORE checking for pymavlink: an out-of-range sweep
        # is the more useful thing to hear about, and the check works with no
        # flight stack installed at all.
        try:
            import sim_iterate
        except ImportError as exc:
            self.alert("Simulation module missing", f"sim_iterate.py could not be imported:\n\n{exc}", "error")
            return
        try:
            low, high, steps = float(self.sim_low.value), float(self.sim_high.value), int(self.sim_steps.value)
        except (TypeError, ValueError):
            self.alert("Invalid sweep", "From, To and Steps must be numbers.", "warn")
            return
        param = self.sim_param.value
        try:
            candidates = sim_iterate.sweep(param, low, high, steps)
        except ValueError as exc:
            self.alert("Invalid sweep", str(exc), "error")
            return
        bad = []
        for cand in candidates:
            bad += [b["message"] for b in safety.check_param_set(cand)["blocked"]]
        if bad:
            self.alert("Sweep leaves the approved range", "This sweep would test values outside the approved range:\n\n"
                       + "\n".join("  • " + b for b in dict.fromkeys(bad)), "error")
            return
        ok, msg = fc.available()
        if not ok:
            self.alert("pymavlink not installed", msg, "error")
            return
        aircraft_id = self.selected_repo_aircraft()
        if not await self.confirm("Run sweep?",
                                  f"Fly {len(candidates)} simulated run(s), sweeping {param} from {low:g} to {high:g}.\n\n"
                                  f"Roughly {len(candidates)} × 90 s.\n\nResults will be linked to aircraft id "
                                  f"{aircraft_id if aircraft_id else '(none selected in Repository)'}.\n\nProceed?",
                                  yes="Run Sweep"):
            return
        file_evidence = bool(self.sim_file_evidence.value)
        if file_evidence and aircraft_id is None:
            if not await self.confirm("No aircraft selected",
                                      "Filing runs as flight evidence needs an aircraft selected on the Repository "
                                      "tab.\n\nRun anyway? The flights will still be scored and logged, but nothing "
                                      "will be filed as evidence.", yes="Run Anyway"):
                return
        cfg = sim_iterate.HarnessConfig(connection=(self.sim_conn.value or "").strip(), db_path=str(DB_PATH))
        tlog, binlog = bool(self.sim_tlog.value), bool(self.sim_bin.value)
        self.sim_run_btn.disable()
        try:
            ranked = await self.run_bg(
                lambda say: sim_iterate.iterate(candidates, cfg, aircraft_id=aircraft_id, progress=say,
                                                record_tlog=tlog, download_bin=binlog, log_dir=str(LOG_DIR),
                                                files_dir=str(FILES_DIR) if file_evidence else None),
                f"Running {len(candidates)} simulated flights")
        except Exception as exc:  # noqa: BLE001
            self.alert("Sweep failed", str(exc), "error")
            return
        finally:
            self.sim_run_btn.enable()
        self.refresh_sim()
        self.refresh_predict()
        best = ranked[0] if ranked else None
        self.alert("Sweep complete",
                   f"Best score {best['score']['outcome_score']:.4f} with "
                   + ", ".join(f"{k}={v:g}" for k, v in sorted(best["params"].items()) if k == param)
                   if best else "No runs completed.", "ok")

    # =====================================================================
    # STANDARD BENCH — sitl_bench/'s standardized flight, filed into ATLAS
    # =====================================================================
    # The bench runs as its own process (it fans out to worker processes and
    # SITL instances, which do not belong inside the GUI process). Its output
    # streams into the log pane; its results land in this database through its
    # own ATLAS link, so the Simulation and Predict tabs pick them up.
    def build_bench(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.eyebrow("SITL STANDARD BENCH")
            w.title("Compare Tunes on a Standard Flight")
            w.sub("Flies one identical test flight (hover, position step, box, climb/descend, yaw steps, speed run, "
                  "attitude steps) in Mission Planner's SITL for every configuration and repeat, scores each run 0–1 "
                  "from its onboard log, and files it as simulated evidence. The simulated airframe and baseline tune "
                  "come from the selected aircraft, and every configuration passes the safety check before anything "
                  "flies.")
            with ui.grid(columns=2).classes("w-full gap-3"):
                with w.card("AIRCRAFT & RUN"):
                    with ui.grid(columns=3).classes("w-full gap-3"):
                        self.bench_aircraft = w.fixed_select("Aircraft", [], required=True,
                                                             on_change=lambda _e: self._bench_load_weight()) \
                            .classes("col-span-2")
                        self.bench_weight = w.text_input("All-up weight (g)", required=True)
                        self.bench_repeats = w.text_input("Repeats per config", "2")
                        self.bench_workers = w.text_input("Parallel runs", "2")
                        self.bench_speedup = w.text_input("Sim speedup", "10")
                    with ui.row().classes("gap-6 pt-1"):
                        self.bench_save_weight = ui.checkbox("Save this weight to the aircraft record", value=False)
                        self.bench_file = ui.checkbox("File runs into ATLAS as simulated evidence", value=True)
                    w.sub("Training needs a recorded weight; without one, runs are flown and filed but skipped by the "
                          "model. Keep parallel runs at 2–3: runs whose log loses data are marked aborted, not scored.")
                with w.card("CONFIGURATIONS"):
                    self.bench_cfg = ui.textarea(value="hardware_tune:\nrate_P_up: ATC_RAT_RLL_P=0.045, ATC_RAT_PIT_P=0.045\n") \
                        .props("outlined input-style='font-family:Consolas,Menlo,monospace;font-size:12.5px' rows=6") \
                        .classes("w-full")
                    w.sub("One per line:  name: PARAM=value, PARAM=value.  The first line is the baseline; leave it "
                          "empty to fly the aircraft's own tune. SIM_* and LOG_* configure the simulator, not the "
                          "aircraft (e.g. SIM_WIND_SPD=8).")
            with ui.row().classes("w-full items-center gap-2"):
                self.bench_run_btn = w.primary("Run Bench", self.run_bench, icon="play_arrow")
                w.danger("Stop", self.stop_bench, icon="stop")
                w.secondary("Open Results Folder", self.open_bench_folder, icon="folder_open")
                ui.space()
                self.bench_summary = w.sub()
            with ui.grid(columns=2).classes("w-full gap-3"):
                self.bench_table = w.DataTable([("config", "Configuration"), ("repeat", "Repeat"), ("score", "Score"),
                                                ("status", "Status")], row_key="key", height="22rem")
                self.bench_log = ui.log(max_lines=4000).classes("atlas-pane w-full") \
                    .style(f"height:22rem;font-family:Consolas,Menlo,monospace;font-size:12px;color:{w.C['muted']}")
        if SESSION.bench_proc is not None and SESSION.bench_proc.poll() is None:
            self.bench_run_btn.disable()

    def refresh_bench(self):
        with db() as c:
            rows = c.execute("SELECT id,name FROM aircraft ORDER BY name").fetchall()
        values = [f"{r['id']} — {r['name']}" for r in rows]
        current = self.bench_aircraft.value
        if current in values:
            self.bench_aircraft.set_options(values, value=current)
            return
        repo = self.selected_repo_aircraft()
        pick = next((v for v in values if repo and v.startswith(f"{repo} — ")), values[0] if values else None)
        self.bench_aircraft.set_options(values, value=pick)
        self._bench_load_weight()

    def _bench_aircraft_id(self):
        v = self.bench_aircraft.value
        return int(v.split(" — ", 1)[0]) if v else None

    def _bench_load_weight(self):
        aid = self._bench_aircraft_id()
        if aid is None:
            return
        with db() as c:
            row = c.execute("SELECT weight_g FROM aircraft WHERE id=?", (aid,)).fetchone()
        self.bench_weight.value = f"{row['weight_g']:g}" if row and row["weight_g"] else ""

    def _bench_parse_configs(self):
        """Ordered {name: {PARAM: value}} from the configuration box."""
        configs = {}
        for n, line in enumerate((self.bench_cfg.value or "").splitlines(), 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            name, sep, rest = line.partition(":")
            name = name.strip()
            if not sep or not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
                raise ValueError(f"line {n}: start with a name made of letters, digits, _ . - followed by ':'")
            if name in configs:
                raise ValueError(f"line {n}: '{name}' is used twice")
            params = {}
            for item in filter(None, (s.strip() for s in rest.split(","))):
                k, eq, v = item.partition("=")
                k = k.strip().upper()
                if not eq or not re.fullmatch(r"[A-Z0-9_]+", k):
                    raise ValueError(f"line {n}: '{item}' is not PARAM=value")
                try:
                    params[k] = float(v)
                except ValueError:
                    raise ValueError(f"line {n}: {k} value '{v.strip()}' is not a number") from None
            configs[name] = params
        if not configs:
            raise ValueError("enter at least one configuration")
        return configs

    async def run_bench(self):
        if SESSION.bench_proc is not None and SESSION.bench_proc.poll() is None:
            self.alert("Bench running", "A bench run is already in progress.")
            return
        runner = BENCH_DIR / "run_bench.py"
        if not runner.exists():
            self.alert("Bench not found", f"Expected the SITL bench at:\n{BENCH_DIR}", "error")
            return
        aid = self._bench_aircraft_id()
        if aid is None:
            self.alert("No aircraft", "Choose an aircraft to simulate.", "warn")
            return
        try:
            configs = self._bench_parse_configs()
            repeats, workers = int(self.bench_repeats.value), int(self.bench_workers.value)
            speedup = float(self.bench_speedup.value)
            if not (1 <= repeats <= 20 and 1 <= workers <= 6 and 1 <= speedup <= 20):
                raise ValueError("repeats 1–20, parallel runs 1–6, speedup 1–20")
            raw_weight = (self.bench_weight.value or "").strip()
            weight = float(raw_weight) if raw_weight else None
        except (TypeError, ValueError) as exc:
            self.alert("Invalid bench setup", str(exc), "warn")
            return
        if weight is None or weight <= 0:
            self.alert("Weight needed", "Enter the aircraft's all-up weight in grams. The simulated airframe is built "
                       "from it.", "warn")
            return

        # Same check the bench applies, run here first so nothing starts if a config is blocked.
        if str(BENCH_DIR) not in sys.path:
            sys.path.insert(0, str(BENCH_DIR))
        try:
            import atlas_link
            with db() as c:
                recorded = c.execute("SELECT weight_g FROM aircraft WHERE id=?", (aid,)).fetchone()["weight_g"]
                aircraft = atlas_link.load_aircraft(c, bridge, aid, None if recorded else weight)
            blocked = []
            for name, overrides in configs.items():
                _, _, report = atlas_link.plan_config(bridge, safety, aircraft, overrides)
                blocked += [f"{name}: {b['name']} = {b.get('value')} ({b.get('message') or b.get('verdict')})"
                            for b in report["blocked"]]
        except Exception as exc:  # noqa: BLE001
            self.alert("Cannot prepare bench", str(exc), "error")
            return
        if blocked:
            self.alert("Configuration leaves the approved range",
                       "These configurations would fly values outside the approved range:\n\n"
                       + "\n".join("  • " + b for b in blocked), "error")
            return

        file_runs = bool(self.bench_file.value)
        nruns = len(configs) * repeats
        if not await self.confirm(
                "Run standard bench?",
                f"Fly {nruns} simulated run(s): {len(configs)} configuration(s) × {repeats} repeat(s) on "
                f"{aircraft['name']}.\n\nRoughly {max(1, round(nruns * 30 / workers / 60))} min at {speedup:g}× speed, "
                f"{workers} at a time.\n\n"
                + ("Each run will be filed into ATLAS as simulated evidence." if file_runs
                   else "Nothing will be filed into ATLAS.") + "\n\nProceed?", yes="Run Bench"):
            return
        if self.bench_save_weight.value and recorded != weight:
            with db() as c:
                c.execute("UPDATE aircraft SET weight_g=? WHERE id=?", (weight, aid))
            recorded = weight

        # The bench resolves paths relative to its config file, so write absolute ones.
        with open(BENCH_DIR / "bench.json") as fh:
            cfg = json.load(fh)
        for key in ("sitl_binary", "copter_defaults", "model_file", "base_params"):
            cfg[key] = str((BENCH_DIR / os.path.expanduser(cfg[key])).resolve())
        cfg.update(repeats=repeats, workers=workers, speedup=speedup, configs=configs)
        cfg_path = LOG_DIR / f"bench_config_{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
        with open(cfg_path, "w") as fh:
            json.dump(cfg, fh, indent=2)
        cmd = [sys.executable, "-u", str(runner), str(cfg_path), "--atlas-db", str(DB_PATH), "--aircraft", str(aid),
               "--atlas-dir", str(APP_DIR)]
        if not recorded:
            cmd += ["--weight-g", f"{weight:g}"]
        if not file_runs:
            cmd.append("--no-file")

        self.bench_table.set_rows([])
        self.bench_log.clear()
        self.bench_summary.text = ""
        SESSION.bench_outdir = None
        self.bench_run_btn.disable()
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        flags = subprocess.CREATE_NO_WINDOW if sys.platform.startswith("win") else 0

        def work(say):
            SESSION.bench_proc = subprocess.Popen(cmd, cwd=str(BENCH_DIR), stdout=subprocess.PIPE,
                                                  stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                                  errors="replace", env=env, creationflags=flags)
            outdir = None
            for line in SESSION.bench_proc.stdout:
                line = line.rstrip()
                SESSION.bench_lines.put(line)
                if " -> " in line and "runs" in line:
                    outdir = line.rsplit(" -> ", 1)[1].strip()
                if line.startswith("["):
                    say(f"Bench: {line}")
            return {"rc": SESSION.bench_proc.wait(), "outdir": outdir}

        try:
            res = await self.run_bg(work, f"Running standard bench: {nruns} simulated flights")
        except Exception as exc:  # noqa: BLE001
            self.alert("Bench failed", str(exc), "error")
            return
        finally:
            self.bench_run_btn.enable()
        SESSION.bench_outdir = res["outdir"]
        self._drain_bench_log()
        self._bench_show_results(res["outdir"])
        self.refresh_all()
        if res["rc"] != 0:
            self.alert("Bench stopped", f"The bench exited with code {res['rc']}. See the log pane for details.", "warn")

    def _bench_show_results(self, outdir):
        if not outdir or not os.path.isdir(outdir):
            return
        by_config, rows = {}, []
        for run_json in sorted(Path(outdir).glob("*/run.json")):
            with open(run_json) as fh:
                r = json.load(fh)
            s = r.get("score") or {}
            score = s.get("outcome_score")
            ok = score is not None and not s.get("aborted")
            status = "completed" if ok else (s.get("abort_reason") or r.get("error") or "failed")
            rows.append({"key": str(run_json), "config": r["config"], "repeat": r["repeat"] + 1,
                         "score": f"{score:.4f}" if ok else "—", "status": status[:80],
                         "_tag": "" if ok else "bad"})
            if ok:
                by_config.setdefault(r["config"], []).append(score)
        self.bench_table.set_rows(rows)
        filed = Path(outdir) / "atlas_filing.json"
        nfiled = len(json.loads(filed.read_text())) if filed.exists() else 0
        means = "   ".join(f"{k}: {sum(v) / len(v):.3f}" for k, v in by_config.items())
        self.bench_summary.text = ((f"Mean score   {means}" if means else "No scored runs")
                                   + (f"   •   {nfiled} filed into ATLAS" if nfiled else ""))

    async def stop_bench(self):
        proc = SESSION.bench_proc
        if proc is None or proc.poll() is not None:
            return
        if not await self.confirm("Stop bench?", "Stop the bench now? Runs in progress are discarded and nothing from "
                                  "this batch is filed.", yes="Stop Bench"):
            return
        _kill_tree(proc)

    def open_bench_folder(self):
        folder = Path(SESSION.bench_outdir) if SESSION.bench_outdir else BENCH_DIR / "runs"
        self.open_folder(folder, "Bench results")

    def _drain_bench_log(self):
        lines = []
        try:
            while True:
                lines.append(SESSION.bench_lines.get_nowait())
        except queue.Empty:
            pass
        if lines:
            self.bench_log.push("\n".join(lines))

    # =====================================================================
    # REPOSITORY
    # =====================================================================
    def build_browser(self):
        with ui.column().classes("w-full gap-3 p-2"):
            with ui.row().classes("w-full items-center"):
                w.title("Configuration Repository")
                ui.space()
                w.primary("Add Parameter File", self.add_parameter_file, icon="note_add")
                w.secondary("Add Flight Evidence", self.add_flight_evidence)
                w.secondary("Edit Validation", self.edit_validation)
                w.secondary("Details", self._details_selected)
                w.secondary("Refresh", self.refresh_browser, icon="refresh")
            self.repo_table = w.DataTable([("name", "Aircraft"), ("frame", "Frame"), ("validation", "Validation Status"),
                                           ("components", "Components"), ("params", "Parameters"),
                                           ("evidence", "Flights (hw / sim)")],
                                          height="60vh", on_double_click=lambda row: self.details(row["id"]))
            w.sub("Select a configuration to update validation. Double-click for full details.")

    def refresh_browser(self):
        rows = []
        with db() as c:
            for a in c.execute("SELECT * FROM aircraft ORDER BY id DESC").fetchall():
                nc = c.execute("SELECT COUNT(*) FROM components WHERE aircraft_id=?", (a["id"],)).fetchone()[0]
                # Parameters from HARDWARE snapshots only — a simulated run files
                # the tune it flew, and lumping those in would read as more real
                # configuration evidence than the aircraft actually has.
                np_ = c.execute("SELECT COUNT(*) FROM parameters p JOIN parameter_files f ON p.parameter_file_id=f.id "
                                "WHERE f.aircraft_id=? AND f.source='hardware'", (a["id"],)).fetchone()[0]
                nhw = c.execute("SELECT COUNT(*) FROM flight_tests WHERE aircraft_id=? AND source='hardware'",
                                (a["id"],)).fetchone()[0]
                nsim = c.execute("SELECT COUNT(*) FROM flight_tests WHERE aircraft_id=? AND source<>'hardware'",
                                 (a["id"],)).fetchone()[0]
                rows.append({"id": a["id"], "name": a["name"],
                             "frame": f"{a['frame_type'] or ''} {a['frame_size'] or ''}".strip(),
                             "validation": a["validation_status"], "components": nc, "params": np_,
                             "evidence": f"{nhw} / {nsim}"})
        self.repo_table.set_rows(rows)

    def _need_repo_selection(self):
        aid = self.selected_repo_aircraft()
        if aid is None:
            self.alert("Select a configuration", "Choose an aircraft first.")
        return aid

    async def add_parameter_file(self):
        aid = self._need_repo_selection()
        if aid is None:
            return
        src = await w.pick_file("Add ArduPilot parameter file", PARAM_PATTERNS, memory="params")
        if not src:
            return
        with self.root:
            ver = await w.prompt("ArduPilot version", "ArduPilot version (optional)")
        if ver is None:
            return
        try:
            dest = FILES_DIR / f"{aid}_{datetime.now().strftime('%Y%m%d%H%M%S')}_{src.name}"
            shutil.copy2(src, dest)
            with db() as c:
                pf = c.execute("INSERT INTO parameter_files(aircraft_id,original_name,stored_path,ardupilot_version,"
                               "imported_at) VALUES(?,?,?,?,?)",
                               (aid, src.name, str(dest), ver.strip(), now_iso())).lastrowid
                c.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",
                              [(pf, a, b) for a, b in parse_param(src)])
            self.set_status(f"Added parameter file • {src.name}")
            self.refresh_all()
            self.alert("Parameter file saved", f"{src.name} is now stored with this configuration.", "ok")
        except Exception as e:  # noqa: BLE001
            self.alert("Could not save parameter file", str(e), "error")

    async def add_flight_evidence(self):
        aid = self._need_repo_selection()
        if aid is None:
            return
        with db() as c:
            pfs = c.execute("SELECT id,original_name,imported_at FROM parameter_files WHERE aircraft_id=? "
                            "ORDER BY id DESC", (aid,)).fetchall()
        if not pfs:
            self.alert("No parameter file", "Add the parameter file used for this flight before adding the flight log.",
                       "warn")
            return
        by_label = {f"#{r['id']} • {r['original_name']} • {r['imported_at'] or ''}": r["id"] for r in pfs}
        with self.root, ui.dialog().props("persistent") as dialog, \
                ui.card().classes("atlas-dialog gap-2").style("width:620px"):
            ui.label("Link Flight Log to Parameter File").classes("atlas-dialog-title")
            w.sub("Choose the exact .param snapshot that was loaded for this flight.")
            pv = w.fixed_select("Parameter file used", list(by_label), next(iter(by_label)))
            with ui.row().classes("w-full items-center no-wrap gap-2"):
                logv = w.text_input("Flight log").classes("grow")

                async def browse():
                    p = await w.pick_file("Select flight log", LOG_PATTERNS, memory="logs")
                    if p:
                        logv.value = str(p)
                w.secondary("Browse…", browse)
            datev = w.text_input("Test date")
            resultv = w.fixed_select("Result", TEST_RESULTS, "Not evaluated")
            note = ui.textarea("Flight notes").props("outlined autogrow stack-label").classes("w-full")
            err = ui.label("").classes("t-bad text-sm")

            def save():
                log = (logv.value or "").strip()
                if not log:
                    err.text = "Choose a flight log file."
                    return
                if not Path(log).is_file():
                    err.text = f"File not found: {log}"
                    return
                try:
                    src = Path(log)
                    dest = FILES_DIR / f"{aid}_{datetime.now().strftime('%Y%m%d%H%M%S')}_{src.name}"
                    shutil.copy2(src, dest)
                    with db() as c:
                        c.execute("INSERT INTO flight_tests(aircraft_id,parameter_file_id,log_name,stored_path,"
                                  "test_date,result,notes) VALUES(?,?,?,?,?,?,?)",
                                  (aid, by_label[pv.value], src.name, str(dest), (datev.value or "").strip(),
                                   resultv.value, (note.value or "").strip()))
                except Exception as e:  # noqa: BLE001
                    err.text = f"Could not save flight evidence: {e}"
                    return
                dialog.submit(src.name)

            with ui.row().classes("w-full justify-end gap-2"):
                w.secondary("Cancel", lambda: dialog.submit(None))
                w.primary("Save Linked Flight", save)
        saved = await dialog
        dialog.delete()
        if saved:
            self.set_status(f"Flight evidence linked • {saved}")
            self.refresh_all()
            self.alert("Flight evidence saved", f"{saved} is linked to {pv.value.split(' • ')[1]}.", "ok")

    async def edit_validation(self):
        aid = self._need_repo_selection()
        if aid is None:
            return
        with db() as c:
            a = c.execute("SELECT * FROM aircraft WHERE id=?", (aid,)).fetchone()
        with self.root, ui.dialog().props("persistent") as dialog, \
                ui.card().classes("atlas-dialog gap-2").style("width:520px"):
            ui.label(a["name"]).classes("atlas-dialog-title")
            w.sub("Update the status only when supported by configuration evidence.")
            v = w.fixed_select("Validation status", VALIDATION, a["validation_status"])
            note = ui.textarea("Validation notes", value=a["notes"] or "").props("outlined autogrow stack-label").classes("w-full")
            with ui.row().classes("w-full justify-end gap-2"):
                w.secondary("Cancel", lambda: dialog.submit(False))
                w.primary("Save Validation", lambda: dialog.submit(True))
        if await dialog:
            with db() as c:
                c.execute("UPDATE aircraft SET validation_status=?, notes=? WHERE id=?",
                          (v.value, (note.value or "").strip(), aid))
            self.set_status(f"Updated {a['name']} → {v.value}")
            self.refresh_all()
        dialog.delete()

    def _details_selected(self):
        aid = self._need_repo_selection()
        if aid is not None:
            self.details(aid)

    def details(self, aid: int):
        with db() as c:
            a = c.execute("SELECT * FROM aircraft WHERE id=?", (aid,)).fetchone()
            comps = c.execute("SELECT * FROM components WHERE aircraft_id=?", (aid,)).fetchall()
            pfs = c.execute("SELECT * FROM parameter_files WHERE aircraft_id=? ORDER BY id", (aid,)).fetchall()
            tests = c.execute("SELECT ft.*, pf.original_name AS parameter_name FROM flight_tests ft LEFT JOIN "
                              "parameter_files pf ON ft.parameter_file_id=pf.id WHERE ft.aircraft_id=? "
                              "ORDER BY ft.id DESC", (aid,)).fetchall()
            outcomes = c.execute("SELECT * FROM acro_outcomes WHERE aircraft_id=? ORDER BY id DESC", (aid,)).fetchall()
            feats, prov = bridge.extract_features(c, aid)
            label, why = bridge.outcome_label(c, aid)
        with self.root, ui.dialog() as dialog, ui.card().classes("atlas-dialog gap-2").style("width:900px"):
            with ui.row().classes("w-full items-center"):
                ui.label(a["name"]).classes("atlas-dialog-title")
                ui.space()
                ui.button(icon="close", on_click=dialog.close).props("flat round dense")
            t = w.TextPane("68vh")
        dialog.on("hide", dialog.delete)

        t.write(f"{a['name']}\nVALIDATION: {a['validation_status']}\nFrame: {a['frame_type']} {a['frame_size'] or ''}\n")
        t.write(f"TRAINING LABEL: {'excluded' if label is None else f'{label:.2f}'} — {why}\n", "muted")
        t.write("\nHARDWARE\n")
        for x in comps:
            t.write(f"• {x['category']}: {x['manufacturer'] or ''} {x['model'] or ''} {x['specs'] or ''}\n")
        t.write("\nMEASUREMENTS FOR PREDICTION\n")
        miss = bridge.missing_features(feats)
        for feat in bridge.FEATURE_FIELDS:
            v = feats.get(feat)
            t.write(f"• {bridge.FEATURE_LABELS[feat]:<20} {(f'{v:g}' if v is not None else 'not recorded')}\n",
                    "" if v is not None else "sim")
        if miss:
            t.write(f"  missing {len(miss)} — this aircraft cannot train or receive a prediction until they are "
                    "filled in\n", "sim")
        for line in prov:
            if "INFERRED" in line:
                t.write(f"  note: {line}\n", "sim")
        # Source is shown on every row, so a SITL flight never looks like one that actually flew.
        t.write("\nPARAMETER FILES\n")
        for x in pfs:
            src = (x["source"] if "source" in x.keys() else "hardware") or "hardware"
            t.write(f"• [{src.upper():<8}] {x['original_name']}  ArduPilot "
                    f"{x['ardupilot_version'] or 'version not recorded'}\n", "sim" if src != "hardware" else "hw")
        if not pfs:
            t.write("  (none)\n", "muted")
        t.write("\nFLIGHT TESTS\n")
        for x in tests:
            src = (x["source"] if "source" in x.keys() else "hardware") or "hardware"
            t.write(f"• [{src.upper():<8}] {x['test_date'] or 'date not recorded'} • {x['result'] or ''} • "
                    f"{x['log_name'] or ''}\n", "sim" if src != "hardware" else "hw")
            t.write(f"    PARAM: {x['parameter_name'] or 'legacy / not linked'}\n", "muted")
        if not tests:
            t.write("  (none)\n", "muted")
        if outcomes:
            t.write("\nRECORDED SCORES\n")
            for o in outcomes:
                src = o["source"] or "hardware"
                t.write(f"• [{src.upper():<8}] score {o['outcome_score']:.4f}"
                        + (f"  vibration {o['vibration_max']:g} m/s^2" if o["vibration_max"] is not None else "")
                        + f"  ({o['recorded_at'] or ''})\n", "sim" if src != "hardware" else "hw")
            t.write("  SITL scores never change validation status and are excluded from hardware training.\n", "muted")
        if a["notes"]:
            t.write(f"\nVALIDATION NOTES\n{a['notes']}\n")
        dialog.open()

    # =====================================================================
    # HARDWARE CATALOG
    # =====================================================================
    def build_hardware(self):
        with ui.column().classes("w-full gap-3 p-2"):
            w.title("Hardware Catalog")
            w.sub("Add, edit, deactivate, or remove reusable hardware choices without changing the Python source.")
            with w.card("Catalog item"):
                with ui.grid(columns=3).classes("w-full gap-3"):
                    self.hw_cat = w.fixed_select("Category", CATEGORIES, "Flight Controller")
                    self.hw_make = w.text_input("Manufacturer")
                    self.hw_model = w.text_input("Model")
                    self.hw_specs = w.text_input("Specs / notes").classes("col-span-3")
                with ui.row().classes("w-full items-center justify-end gap-2"):
                    self.hw_editing = w.sub("")
                    w.secondary("Clear", self.clear_hw_form)
                    w.primary("Save Hardware", self.save_hardware, icon="save")
            self.hw_table = w.DataTable([("category", "Category"), ("manufacturer", "Manufacturer"), ("model", "Model"),
                                         ("specs", "Specs / Notes"), ("active", "Active")],
                                        height="46vh", on_select=lambda _rows: self.load_hw_selection())
            with ui.row().classes("w-full items-center gap-2"):
                w.secondary("Toggle Active", self.toggle_hardware)
                w.danger("Delete", self.delete_hardware, icon="delete")
                w.secondary("Add Missing From Aircraft", self.catalog_from_aircraft)
                ui.space()
                w.sub("Inactive items stay in the catalog history but disappear from aircraft dropdowns.")

    async def catalog_from_aircraft(self):
        """Catalog every part recorded against an aircraft but not yet listed."""
        import import_build
        with db() as c:
            missing = c.execute(
                "SELECT DISTINCT cm.category, cm.manufacturer, cm.model FROM components cm "
                "WHERE TRIM(COALESCE(cm.model,'')) <> '' AND NOT EXISTS ("
                " SELECT 1 FROM hardware_catalog h WHERE h.category=cm.category "
                " AND h.manufacturer=COALESCE(cm.manufacturer,'') AND h.model=cm.model)").fetchall()
        if not missing:
            self.alert("Nothing to add", "Every part fitted to an aircraft is already in the catalog.")
            return
        listing = "\n".join(f"   [{r['category']}]  {r['manufacturer'] or '(no make)'}  {r['model']}"
                            for r in missing[:18])
        if len(missing) > 18:
            listing += f"\n   … and {len(missing) - 18} more"
        if not await self.confirm("Add to hardware catalog",
                                  f"{len(missing)} part(s) are fitted to an aircraft but not in the reusable catalog:"
                                  f"\n\n{listing}\n\nAdd them? They will then appear in the Add Aircraft dropdowns.",
                                  yes="Add to Catalog", kind="info"):
            return
        with db() as c:
            added, already = import_build.catalog_sync_from_aircraft(c)
        self.set_status(f"Hardware catalog: {added} added" + (f", {already} already present" if already else ""))
        self.refresh_all()
        self.alert("Hardware catalog updated",
                   f"Added {added} part(s)." + (f"\n{already} were already present." if already else ""), "ok")

    def refresh_hardware(self):
        with db() as c:
            rows = [{"id": r["id"], "category": r["category"], "manufacturer": r["manufacturer"], "model": r["model"],
                     "specs": r["specs"] or "", "active": "Yes" if r["active"] else "No",
                     "_tag": "" if r["active"] else "muted"}
                    for r in c.execute("SELECT * FROM hardware_catalog ORDER BY category,manufacturer,model")]
        self.hw_table.set_rows(rows)

    def clear_hw_form(self):
        self.edit_hw_id = None
        self.hw_make.value = self.hw_model.value = self.hw_specs.value = ""
        self.hw_editing.text = ""
        self.hw_table.set_rows(self.hw_table.rows, keep_selection=False)

    def load_hw_selection(self):
        sel = self.hw_table.selected
        if not sel:
            return
        r = sel[0]
        self.edit_hw_id = r["id"]
        self.hw_cat.value, self.hw_make.value, self.hw_model.value, self.hw_specs.value = (
            r["category"], r["manufacturer"], r["model"], r["specs"])
        self.hw_editing.text = f"Editing #{r['id']} — Clear to add a new item instead."

    def save_hardware(self):
        cat = (self.hw_cat.value or "").strip()
        make, model, specs = ((x.value or "").strip() for x in (self.hw_make, self.hw_model, self.hw_specs))
        if not cat or not make or not model:
            self.alert("Missing hardware information", "Category, manufacturer, and model are required.", "warn")
            return
        try:
            with db() as c:
                if self.edit_hw_id:
                    c.execute("UPDATE hardware_catalog SET category=?,manufacturer=?,model=?,specs=? WHERE id=?",
                              (cat, make, model, specs, self.edit_hw_id))
                else:
                    c.execute("INSERT INTO hardware_catalog(category,manufacturer,model,specs,created_at) "
                              "VALUES(?,?,?,?,?)", (cat, make, model, specs, now_iso()))
            self.set_status(f"Hardware catalog updated • {make} {model}")
            self.clear_hw_form()
            self.refresh_all()
        except sqlite3.IntegrityError:
            self.alert("Duplicate hardware", "That category / manufacturer / model already exists.", "error")

    def toggle_hardware(self):
        hid = self.hw_table.selected_key
        if hid is None:
            self.alert("Select an item", "Choose a catalog item first.")
            return
        with db() as c:
            c.execute("UPDATE hardware_catalog SET active=CASE active WHEN 1 THEN 0 ELSE 1 END WHERE id=?", (hid,))
        self.refresh_all()

    async def delete_hardware(self):
        hid = self.hw_table.selected_key
        if hid is None:
            self.alert("Select an item", "Choose a catalog item first.")
            return
        if not await self.confirm("Delete hardware", "Remove this item from the reusable hardware catalog? Existing "
                                  "aircraft records will not be changed.", yes="Delete"):
            return
        with db() as c:
            c.execute("DELETE FROM hardware_catalog WHERE id=?", (hid,))
        self.clear_hw_form()
        self.refresh_all()

    # =====================================================================
    def refresh_all(self):
        self.refresh_browser()
        self.refresh_hardware()
        self.refresh_ingest_catalog()
        self.refresh_predict()
        self.refresh_fc_pending()
        self.refresh_sim()
        self.refresh_bench()
        self.amc.refresh()
        with db() as c:
            counts = {"air": c.execute("SELECT COUNT(*) FROM aircraft").fetchone()[0],
                      "comp": c.execute("SELECT COUNT(*) FROM components").fetchone()[0],
                      "hw": c.execute("SELECT COUNT(*) FROM hardware_catalog WHERE active=1").fetchone()[0],
                      "param": c.execute("SELECT COUNT(*) FROM parameters").fetchone()[0],
                      "train": bridge.census(c)["trainable"]}
        for key, n in counts.items():
            self.stats[key].text = str(n)


@ui.page("/")
def index():
    AtlasApp()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="ATLAS — Acro Configuration Intelligence (NiceGUI interface)")
    ap.add_argument("--native", action="store_true", help="open in a desktop window instead of the browser "
                                                          "(needs: pip install pywebview)")
    ap.add_argument("--port", type=int, default=8765, help="local port (default 8765)")
    ap.add_argument("--no-browser", action="store_true", help="start the server without opening a browser")
    args = ap.parse_args(argv)
    init_db()
    app.on_shutdown(SESSION.close)
    ui.run(host="127.0.0.1", port=args.port, title="ATLAS // Acro Configuration Intelligence", dark=True,
           native=args.native, window_size=(1440, 920) if args.native else None, reload=False,
           show=not args.no_browser, favicon="🛩", show_welcome_message=False)


if __name__ == "__main__":
    main()
