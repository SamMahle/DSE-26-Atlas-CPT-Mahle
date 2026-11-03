"""
gui_amc.py — the "Configuration" tab of the NiceGUI interface.

Primary path (PROTOTYPE, option B): generate a full first-flight parameter
file inside ATLAS with the embedded ArduPilot Methodic Configurator engine
(amc_engine.py). Advanced path: hand the build to AMC as a project
(amc_bridge.py). Also seeds the repository from AMC's published templates.

Presentation only, like amc_tab.py which it replaces for this interface; the
logic lives in the two modules, which run without a display.
"""

from __future__ import annotations

from pathlib import Path

from nicegui import background_tasks, ui

import amc_bridge as amc
import amc_engine as engine
from atlas_core import db, open_in_file_browser
import gui_widgets as w

SOURCE_TAGS = {amc.SRC_RECORD: "rec", amc.SRC_PARAMS: "par", amc.SRC_INFERRED: "inf", amc.SRC_TEMPLATE: "tpl"}


class ConfigurationTab:
    """Builds the tab inside the current tab panel and keeps its own state."""

    def __init__(self, app):
        self.app = app
        self.settings = amc.load_settings()
        self.templates: list[amc.Template] = []
        self.templates_dir: Path | None = None
        self.plan: amc.ExportPlan | None = None
        self.last_export: Path | None = None
        self.engine_ok = False
        self.overrides: dict[str, str] = {}      # operator's settlement of AMC-vs-tune conflicts
        self.conflicts: list[dict] = []
        self.conflict_source = ""
        self._labels: dict[str, amc.Template] = {}
        self._summary_base = ""
        self._check_token = 0                    # discards stale background engine checks
        self.cdialog = None                      # the AMC-vs-tune dialog, when open
        self._quiet = False                      # True while the template list is being replaced
        self._build()
        self.reload_amc()

    # ------------------------------------------------------------------ layout
    def _build(self):
        with ui.column().classes("w-full gap-3 p-2"):
            with ui.row().classes("w-full items-start no-wrap"):
                with ui.column().classes("gap-1 grow"):
                    w.eyebrow("CONFIGURATION ENGINE  ·  PROTOTYPE")
                    w.title("Generate a Configuration")
                    w.sub("Turn a build's components and tune into a full first-flight parameter file, computed by "
                          "ArduPilot Methodic Configurator's engine inside ATLAS. Export as an AMC project for full "
                          "setup.")
                w.secondary("Locate AMC…", self.locate_amc, icon="travel_explore")
                w.secondary("Import AMC Templates", self.import_templates, icon="library_add")
            self.amc_status = ui.label("").classes("atlas-pane atlas-sub w-full") \
                .style("white-space:pre-wrap;padding:8px 12px")

            with ui.grid(columns="minmax(0,2fr) minmax(0,3fr)").classes("w-full gap-3"):
                with w.card("1. AIRCRAFT"):
                    self.tree = w.DataTable([("name", "Aircraft"), ("frame", "Frame"), ("status", "Validation"),
                                             ("params", "Params")], height="58vh",
                                            on_select=lambda _rows: self.on_aircraft())
                with ui.column().classes("w-full gap-3"):
                    with w.card("2. STARTING TEMPLATE"):
                        self.template_box = w.fixed_select("Template", [], on_change=lambda _e: self._quiet or self.preview())
                        self.reason = w.sub("Select an aircraft.")
                        self.same_fc = ui.checkbox("Same physical flight controller — keep its sensor and radio "
                                                   "calibrations", value=False, on_change=lambda _e: self.preview())
                    with w.card("3. REVIEW"):
                        with ui.row().classes("w-full items-start no-wrap gap-2"):
                            self.summary = w.sub("").classes("grow").style("white-space:pre-wrap")
                            self.conflict_btn = w.secondary("Decide AMC vs Tune", self.open_conflicts)
                            self.conflict_btn.disable()
                        with ui.tabs().props("dense no-caps align=left").classes("atlas-tabs") as inner:
                            ui.tab("Review")
                            ui.tab("Component mapping")
                        with ui.tab_panels(inner, value="Review").props("animated=false keep-alive") \
                                .classes("w-full bg-transparent"):
                            with ui.tab_panel("Review").classes("p-0 pt-2"):
                                self.notes = w.TextPane("16rem", mono=False)
                            with ui.tab_panel("Component mapping").classes("p-0 pt-2"):
                                self.map = w.DataTable([("field", "AMC component field"), ("value", "Value"),
                                                        ("source", "Source")], row_key="key", height="16rem")
                    with ui.row().classes("w-full items-center no-wrap gap-2"):
                        self.dest = w.text_input("Export to", self.settings.get(
                            "amc_export_dir", str(Path.home() / "ATLAS_AMC_projects"))).classes("grow")
                        w.secondary("Browse…", self.pick_dest)
                    with ui.row().classes("w-full justify-end gap-2"):
                        self.generate_btn = w.primary("Generate Parameter File", self.generate, icon="bolt")
                        self.export_btn = w.secondary("Export AMC Project", self.export, icon="drive_file_move")
                        self.folder_btn = w.secondary("Open Folder", self.open_folder, icon="folder_open")
                        self.open_btn = w.secondary("Open in AMC", self.open_in_amc, icon="open_in_new")
                        for b in (self.generate_btn, self.export_btn, self.folder_btn, self.open_btn):
                            b.disable()

    # ------------------------------------------------------------------ data
    def reload_amc(self):
        self.templates_dir = amc.find_templates_dir(self.settings)
        self.templates = amc.list_templates(self.templates_dir) if self.templates_dir else []
        cmd = amc.find_amc_command(self.settings)
        parts = []
        if self.templates_dir:
            populated = sum(1 for t in self.templates if not t.is_empty)
            parts.append(f"AMC templates  ●  {len(self.templates)} found ({populated} published builds)  —  "
                         f"{self.templates_dir}")
        else:
            parts.append("AMC templates  ○  not found. Install ArduPilot Methodic Configurator, or use Locate AMC… "
                         "to point at its install or source folder.")
        parts.append("AMC program  " + ("●  " + " ".join(cmd) if cmd else
                                        "○  not found — projects can still be exported and opened in AMC by hand"))
        self.engine_ok, why = engine.status()
        parts.append("Engine  " + ("●  " if self.engine_ok else "○  ") + why)
        self.amc_status.text = "\n".join(parts)
        self.refresh()

    def refresh(self):
        keep = self.tree.selected_key
        with db() as c:
            rows = c.execute(
                "SELECT a.id, a.name, a.frame_type, a.frame_size, a.validation_status, "
                " (SELECT COUNT(*) FROM parameters p JOIN parameter_files f ON p.parameter_file_id=f.id "
                "   WHERE f.aircraft_id=a.id AND f.source='hardware' "
                "   AND f.id=(SELECT MAX(id) FROM parameter_files WHERE aircraft_id=a.id AND source='hardware')) AS np "
                "FROM aircraft a ORDER BY a.name").fetchall()
        self.tree.set_rows([{"id": r["id"], "name": r["name"],
                             "frame": f"{r['frame_type'] or ''} {r['frame_size'] or ''}".strip(),
                             "status": r["validation_status"], "params": r["np"]} for r in rows])
        if keep is not None and self.tree.selected_key is None:
            self._clear_review("Select an aircraft.")

    def _aircraft_id(self):
        return self.tree.selected_key

    def _clear_review(self, reason: str):
        self._quiet = True
        try:
            self.template_box.set_options([], value=None)
        finally:
            self._quiet = False
        self.reason.text = reason
        self.preview()

    def on_aircraft(self):
        aid = self._aircraft_id()
        if aid is None:
            return
        if not self.templates:
            self.reason.text = "AMC templates were not found. Use Locate AMC… first."
            return
        with db() as c:
            rec = amc.load_aircraft(c, aid)
        suggested, why = engine.default_template(rec, self.templates)
        if not rec.params:
            ref, _ = amc.suggest_template(rec, self.templates)
            if ref is not None and not ref.is_empty:
                why += f"  Closest published build, for reference: {ref.name}."
        vt = amc.vehicle_type_for(rec)
        pool = [t for t in self.templates if vt is None or t.vehicle_type == vt]
        self._labels = {t.label(): t for t in pool}
        self.overrides = {}          # choices belong to one aircraft
        value = suggested.label() if suggested and suggested.label() in self._labels else None
        self._quiet = True               # replacing the list must not preview the old selection
        try:
            self.template_box.set_options(list(self._labels), value=value)
        finally:
            self._quiet = False
        self.reason.text = why
        self.preview()

    def _template(self):
        return self._labels.get(self.template_box.value)

    def preview(self):
        aid, tpl = self._aircraft_id(), self._template()
        self.map.set_rows([])
        self.plan = None
        self.export_btn.disable()
        self.generate_btn.disable()
        self._summary_base = ""
        if aid is None or tpl is None:
            self.summary.text = ""
            self.conflict_btn.text = "Decide AMC vs Tune"
            self.conflict_btn.disable()
            self._notes([("Choose an aircraft and a starting template.", "")])
            return
        with db() as c:
            self.plan = amc.plan_export(c, aid, tpl, same_flight_controller=bool(self.same_fc.value),
                                        overrides=self.overrides)
        plan = self.plan
        self._summary_base = (
            f"{len(plan.delta)} tuned parameters carried{' in an imported file' if plan.delta else ' (template only)'}"
            f"   •   {len(plan.stripped_calibration)} calibration values left out"
            f"   •   offline parameter docs {'included' if plan.metadata else 'not included'}")
        self.map.set_rows([{"key": " › ".join(pr.path), "field": " › ".join(pr.path), "value": str(pr.value),
                            "source": pr.source + (f" — {pr.note}" if pr.note else ""),
                            "_tag": SOURCE_TAGS.get(pr.source, "rec")}
                           for pr in sorted(plan.provenance, key=lambda x: x.path)])
        lines = [(f"BLOCKED  {b}", "block") for b in plan.blockers]
        lines += [(f"•  {x}", "block" if x.startswith("SAFETY") else "warn") for x in amc.conflict_warnings(plan.conflicts)]
        lines += [(f"•  {x}", "warn") for x in plan.warnings]
        if plan.ok:
            lines.append(("Ready to export. AMC re-validates every component and asks before each write.", "ok"))
        self._notes(lines)
        self._show_conflicts(plan.conflicts, "mandatory values, whole AMC procedure")
        if plan.ok:
            self.export_btn.enable()
            if self.engine_ok and plan.metadata is not None:
                self.generate_btn.enable()
                background_tasks.create(self._engine_check(aid, tpl), name="amc-engine-check")

    def _notes(self, lines):
        self.notes.set_lines(lines)

    # ------------------------------------------------------------------ AMC vs tune
    def _show_conflicts(self, conflicts, source):
        self.conflicts, self.conflict_source = conflicts, source
        n_amc = sum(1 for c in conflicts if c["winner"] == amc.AMC)
        n_safe = sum(1 for c in conflicts if c["safety"] and c["winner"] == amc.ATLAS)
        self.conflict_btn.text = f"Decide AMC vs Tune ({len(conflicts)})"
        self.conflict_btn.set_enabled(bool(conflicts))
        self.summary.text = (self._summary_base + f"\n{len(conflicts)} AMC-vs-tune disagreements: {n_amc} using AMC"
                             + (f", {n_safe} SAFETY VALUES OVERRULED" if n_safe else "") + f"   ({source})")
        if self.cdialog is not None:
            self._fill_conflict_table()

    def open_conflicts(self):
        if self.cdialog is not None:
            self.cdialog.open()
            return
        name = self.plan.aircraft.name if self.plan else ""
        with self.app.root, ui.dialog() as dialog, ui.card().classes("atlas-dialog gap-2") \
                .style("width:1180px;max-width:96vw"):
            ui.label(f"Where AMC and {name}'s tune disagree").classes("atlas-dialog-title")
            w.sub("Default rule: AMC's mandatory SAFETY values (failsafes, arming, fences, return-to-launch) use AMC; "
                  "everything else uses the build's validated tune. Click rows to select them and choose, or "
                  "double-click a row to switch it. Every choice is recorded in the report.")
            self.ctable = w.DataTable([("param", "Parameter"), ("tune", "Tune"), ("amc", "AMC"), ("kind", "AMC rule"),
                                       ("winner", "Using"), ("step", "AMC step"), ("why", "AMC's reason")],
                                      row_key="param", selection="multiple", height="52vh", wrap=True,
                                      on_double_click=lambda row: self.toggle_conflict(row["param"]))
            with ui.row().classes("w-full items-center gap-2"):
                self.cnote = w.sub("")
                ui.space()
                w.primary("Use AMC", lambda: self.settle(amc.AMC))
                w.secondary("Use Tune", lambda: self.settle(amc.ATLAS))
                w.secondary("Reset to Default Rule", self.reset_conflicts)
                w.secondary("Close", dialog.close)

        def closed():
            self.cdialog = None
            dialog.delete()
        dialog.on("hide", closed)
        self.cdialog = dialog
        self._fill_conflict_table()
        dialog.open()

    def _fill_conflict_table(self):
        rows = []
        for c in sorted(self.conflicts, key=lambda c: (not c["safety"], c["kind"], c["parameter"])):
            rows.append({"param": c["parameter"], "tune": str(c["atlas"]), "amc": str(c["amc"]),
                         "kind": ("SAFETY · " if c["safety"] else "") + ("mandatory" if c["kind"] == "forced"
                                                                          else "calculated"),
                         "winner": ("AMC" if c["winner"] == amc.AMC else "Tune")
                                   + ("  (you)" if c["decided_by"] == "operator" else ""),
                         "step": c["step"], "why": c["reason"],
                         "_tag": "amc" if c["winner"] == amc.AMC else ("safety_atlas" if c["safety"] else "atlas")})
        self.ctable.set_rows(rows)
        n_amc = sum(1 for c in self.conflicts if c["winner"] == amc.AMC)
        self.cnote.text = f"{len(self.conflicts)} disagreements, {n_amc} using AMC  ·  {self.conflict_source}"

    async def _engine_check(self, aid, tpl):
        """Ask the engine for the full list (calculated values too) without writing anything."""
        self._check_token += 1
        token, same_fc, overrides = self._check_token, bool(self.same_fc.value), dict(self.overrides)
        self.summary.text = self._summary_base + "\nChecking every value with the AMC engine …"

        def work(_say):
            conn = db()
            try:
                return engine.generate(conn, aid, tpl, same_flight_controller=same_fc, overrides=overrides)
            finally:
                conn.close()

        try:
            cfg = await self.app.run_bg(work, "Checking with the AMC engine")
        except Exception as exc:  # noqa: BLE001
            if token == self._check_token:
                self.summary.text = self._summary_base + f"\nEngine check failed: {str(exc).splitlines()[0]}"
            return
        if token == self._check_token:
            self._show_conflicts(cfg.plan.conflicts, "first-flight configuration, from the engine")
            self._notes([(f"•  {x}", "block" if x.startswith(("SAFETY", "BATTERY")) else "warn")
                         for x in amc.conflict_warnings(cfg.plan.conflicts) + cfg.plan.warnings]
                        + [(f"AMC component error: {e}", "block") for e in cfg.component_errors])

    async def settle(self, winner, names=None):
        names = names or [r["param"] for r in self.ctable.selected]
        if not names:
            self.app.alert("Select parameters", "Select one or more rows first.")
            return
        overruling = [n for n in names if winner == amc.ATLAS and
                      next((c for c in self.conflicts if c["parameter"] == n), {}).get("safety")]
        if overruling and not await self.app.confirm(
                "Overrule a safety value",
                "AMC sets these as safety values:\n\n   " + "\n   ".join(overruling)
                + "\n\nUse the tune's values instead? The report will record that you decided this.",
                yes="Use Tune", kind="error"):
            return
        for n in names:
            self.overrides[n] = winner
        self.preview()

    async def toggle_conflict(self, name):
        c = next((x for x in self.conflicts if x["parameter"] == name), None)
        if c:
            await self.settle(amc.ATLAS if c["winner"] == amc.AMC else amc.AMC, [name])

    def reset_conflicts(self):
        self.overrides = {}
        self.preview()

    # ------------------------------------------------------------------ actions
    async def pick_dest(self):
        d = await w.pick_dir("Folder for AMC projects", start=self.dest.value or str(Path.home()), memory="amc")
        if d:
            self.dest.value = str(d)

    def export(self):
        if not self.plan or not self.plan.ok:
            return
        dest = Path(self.dest.value or "").expanduser()
        try:
            dest.mkdir(parents=True, exist_ok=True)
            out = amc.write_export(self.plan, dest)
        except (amc.BridgeError, OSError) as exc:
            self.app.alert("Export failed", str(exc), "error")
            return
        self.settings["amc_export_dir"] = str(dest)
        amc.save_settings(self.settings)
        self.last_export = out
        self.open_btn.enable()
        self.folder_btn.enable()
        self.app.set_status(f"AMC project written • {out}")
        n_warn = len(self.plan.warnings)
        self.app.alert("AMC project exported",
                       f"{out}\n\n{len(self.plan.delta)} parameters in the imported file."
                       + (f"\n{n_warn} item(s) to review in AMC are listed in atlas_handoff.json." if n_warn else ""),
                       "ok")

    async def generate(self):
        aid, tpl = self._aircraft_id(), self._template()
        if aid is None or tpl is None:
            return
        same_fc, overrides = bool(self.same_fc.value), dict(self.overrides)
        dest = Path(self.dest.value or "").expanduser()
        self.generate_btn.disable()

        def work(say):
            say("Computing configuration with the AMC engine")
            conn = db()   # a new connection, opened in this worker thread
            try:
                cfg = engine.generate(conn, aid, tpl, same_flight_controller=same_fc, overrides=overrides)
            finally:
                conn.close()
            if not cfg.ok:
                return cfg, None
            dest.mkdir(parents=True, exist_ok=True)
            base = f"{cfg.plan.project_name()}_first_flight"
            path = dest / f"{base}.param"
            n = 2
            while path.exists() or path.with_suffix(".report.json").exists():
                path = dest / f"{base}_{n}.param"
                n += 1
            return cfg, engine.write_param_file(cfg, path)

        try:
            cfg, path = await self.app.run_bg(work, "Generating configuration")
        except Exception as exc:  # noqa: BLE001
            self.app.alert("Configuration not generated", str(exc), "error")
            return
        finally:
            self.generate_btn.enable()
        if path is None:
            self.app.alert("Configuration not generated",
                           "AMC rejected the components:\n• " + "\n• ".join(cfg.component_errors), "error")
            return
        self.last_export = path.parent
        self.folder_btn.enable()
        self.app.set_status(f"Configuration written • {path}")
        top = [x for x in amc.conflict_warnings(cfg.plan.conflicts) + cfg.plan.warnings
               if x.startswith(("BATTERY", "SAFETY", "AMC value", "Battery"))][:5]
        self.app.alert("First-flight configuration generated",
                       f"{path}\n\n{len(cfg.params)} parameters; {cfg.from_atlas} from this build's tune; "
                       f"{len(cfg.plan.stripped_calibration)} calibration values left out.\n\n"
                       + ("Review before flight:\n• " + "\n• ".join(top) if top else "")
                       + "\n\nThe full review is in the .report.json beside the file.", "ok")

    def open_in_amc(self):
        if not self.last_export:
            return
        try:
            cmd = amc.launch_amc(self.last_export, self.settings)
            self.app.set_status("Opened in AMC • " + " ".join(cmd))
        except amc.BridgeError as exc:
            self.app.alert("AMC not found", str(exc), "warn")

    def open_folder(self):
        if not self.last_export:
            return
        try:
            open_in_file_browser(self.last_export)
        except OSError as exc:
            self.app.alert("Could not open folder", str(exc), "error")

    async def locate_amc(self):
        d = await w.pick_dir("Select the AMC install, source, or vehicle_templates folder", memory="amc_install")
        if not d:
            return
        found = amc.resolve_templates_dir(d)
        if not found:
            self.app.alert("Templates not found", f"No AMC vehicle templates were found under\n{d}", "warn")
            return
        self.settings["amc_templates_dir"] = str(found)
        exe = Path(d) / "ardupilot_methodic_configurator.exe"
        if exe.is_file():
            self.settings["amc_executable"] = str(exe)
        amc.save_settings(self.settings)
        self.reload_amc()

    async def import_templates(self):
        if not self.templates:
            self.app.alert("AMC templates not found", "Use Locate AMC… first.", "warn")
            return
        populated = [t for t in self.templates if not t.is_empty]
        with db() as c:
            present = {r[0] for r in c.execute("SELECT name FROM aircraft")}
        new = [t for t in populated if amc.template_aircraft_name(t) not in present]
        if not new:
            self.app.alert("Nothing to import", "Every published AMC build is already in the repository.")
            return
        listing = "\n".join(f"   {amc.template_aircraft_name(t)}" for t in new[:14])
        if len(new) > 14:
            listing += f"\n   … and {len(new) - 14} more"
        if not await self.app.confirm("Import AMC templates",
                                      f"Add {len(new)} published build(s) to the repository?\n\n{listing}\n\n"
                                      "They are stored as UNTESTED: they were configured by the AMC project, not flown "
                                      "by this team, so they are excluded from training until there is evidence on our "
                                      "hardware.", yes="Import", kind="info"):
            return
        with db() as c:
            added, _ = amc.import_templates(c, new)
        self.app.refresh_all()
        self.app.set_status(f"Imported {len(added)} AMC template build(s)")
