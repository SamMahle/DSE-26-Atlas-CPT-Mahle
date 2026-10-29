"""
amc_tab.py — the "Configuration" tab.

Primary path (PROTOTYPE, option B): generate a full first-flight parameter
file inside ATLAS with the embedded ArduPilot Methodic Configurator engine
(amc_engine.py). Advanced path: hand the build to AMC as a project
(amc_bridge.py). Also seeds the repository from AMC's published templates.
Presentation only; the logic lives in the two modules, which run without a display.
"""

from __future__ import annotations

import os
import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import amc_bridge as amc
import amc_engine as engine

SOURCE_TAGS = {amc.SRC_RECORD: "rec", amc.SRC_PARAMS: "par", amc.SRC_INFERRED: "inf", amc.SRC_TEMPLATE: "tpl"}


class AMCHandoffTab:
    """Builds the tab inside the ATLAS window and keeps its own state."""

    def __init__(self, app, notebook: ttk.Notebook, db):
        self.app, self.db, self.C = app, db, app.C
        self.settings = amc.load_settings()
        self.templates: list[amc.Template] = []
        self.templates_dir: Path | None = None
        self.plan: amc.ExportPlan | None = None
        self.last_export: Path | None = None
        self.dest = tk.StringVar(value=self.settings.get("amc_export_dir", str(Path.home() / "ATLAS_AMC_projects")))
        self.template_var = tk.StringVar()
        self.same_fc = tk.BooleanVar(value=False)
        self.reason = tk.StringVar(value="Select an aircraft.")
        self.amc_status = tk.StringVar()
        self.overrides: dict[str, str] = {}      # operator's settlement of AMC-vs-tune conflicts
        self.conflicts: list[dict] = []
        self._check_token = 0                    # discards stale background engine checks
        self._build(notebook)
        self.reload_amc()

    # ------------------------------------------------------------------ layout
    def _build(self, notebook):
        c = self.C
        p = ttk.Frame(notebook, padding=18)
        notebook.add(p, text="  Configuration  ")
        head = ttk.Frame(p); head.pack(fill="x")
        left = ttk.Frame(head); left.pack(side="left", fill="x", expand=True)
        ttk.Label(left, text="CONFIGURATION ENGINE  ·  PROTOTYPE", style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(left, text="Generate a Configuration", style="Big.TLabel").pack(anchor="w", pady=(3, 2))
        ttk.Label(left, style="Sub.TLabel", wraplength=900, justify="left",
                  text="Turn a build's components and tune into a full first-flight parameter file, computed by "
                       "ArduPilot Methodic Configurator's engine inside ATLAS. Export as an AMC project for full setup.").pack(anchor="w")
        right = ttk.Frame(head); right.pack(side="right", anchor="n")
        ttk.Button(right, text="Locate AMC…", style="Secondary.TButton", command=self.locate_amc).pack(side="left", padx=4)
        ttk.Button(right, text="Import AMC Templates", style="Secondary.TButton",
                   command=self.import_templates).pack(side="left", padx=4)

        status = tk.Frame(p, bg=c["panel"], highlightbackground=c["line"], highlightthickness=1)
        status.pack(fill="x", pady=(12, 10))
        tk.Label(status, textvariable=self.amc_status, bg=c["panel"], fg=c["muted"], font=("Segoe UI", 9),
                 anchor="w", justify="left").pack(fill="x", padx=12, pady=8)

        body = ttk.Frame(p); body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=2, uniform="b"); body.columnconfigure(1, weight=3, uniform="b")
        body.rowconfigure(0, weight=1)

        # 1. aircraft
        a = ttk.LabelFrame(body, text="1. AIRCRAFT", style="Card.TLabelframe")
        a.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        cols = ("name", "frame", "status", "params")
        wrap = ttk.Frame(a); wrap.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(wrap, columns=cols, show="headings", selectmode="browse")
        for col, label, width, stretch in (("name", "Aircraft", 180, True), ("frame", "Frame", 120, False),
                                           ("status", "Validation", 140, False), ("params", "Params", 56, False)):
            self.tree.heading(col, text=label); self.tree.column(col, width=width, minwidth=50, anchor="w", stretch=stretch)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.tree.yview); self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True); sb.pack(side="right", fill="y")
        self.tree.bind("<<TreeviewSelect>>", lambda e: self.on_aircraft())

        # 2 + 3. template, review, export
        r = ttk.Frame(body); r.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        # Packed bottom-up first so the export controls never fall off a short window.
        act = ttk.Frame(r); act.pack(side="bottom", fill="x", pady=(8, 0))
        ex = ttk.Frame(r); ex.pack(side="bottom", fill="x", pady=(10, 0))
        t = ttk.LabelFrame(r, text="2. STARTING TEMPLATE", style="Card.TLabelframe"); t.pack(fill="x")
        self.template_box = ttk.Combobox(t, textvariable=self.template_var, state="readonly")
        self.template_box.pack(fill="x", pady=(2, 6)); self.template_box.bind("<<ComboboxSelected>>", lambda e: self.preview())
        ttk.Label(t, textvariable=self.reason, style="Sub.TLabel", background=c["panel"], wraplength=640,
                  justify="left").pack(anchor="w")
        tk.Checkbutton(t, text="Same physical flight controller — keep its sensor and radio calibrations",
                       variable=self.same_fc, command=self.preview, bg=c["panel"], fg=c["text"],
                       selectcolor=c["panel2"], activebackground=c["panel"], activeforeground=c["text"],
                       highlightthickness=0, bd=0).pack(anchor="w", pady=(8, 0))

        rv = ttk.LabelFrame(r, text="3. REVIEW", style="Card.TLabelframe"); rv.pack(fill="both", expand=True, pady=(10, 0))
        srow = ttk.Frame(rv); srow.pack(fill="x", pady=(0, 6))
        self.conflict_btn = ttk.Button(srow, text="Decide AMC vs Tune", style="Secondary.TButton",
                                       command=self.open_conflicts, state="disabled")
        self.conflict_btn.pack(side="right", anchor="n", padx=(8, 0))
        self.summary = ttk.Label(srow, text="", style="Sub.TLabel", background=c["panel"], justify="left", wraplength=560)
        self.summary.pack(side="left", anchor="w")
        self.cwin = None                         # the AMC-vs-tune dialog, when open
        self.conflict_source = ""
        inner = ttk.Notebook(rv); inner.pack(fill="both", expand=True)
        nwrap = ttk.Frame(inner, padding=(0, 6, 0, 0)); inner.add(nwrap, text="  Review  ")
        self.notes = tk.Text(nwrap, height=6, wrap="word", relief="flat", bd=0, bg=c["panel2"], fg=c["text"],
                             font=("Segoe UI", 9), padx=10, pady=8)
        nsb = ttk.Scrollbar(nwrap, orient="vertical", command=self.notes.yview); self.notes.configure(yscrollcommand=nsb.set)
        self.notes.pack(side="left", fill="both", expand=True); nsb.pack(side="right", fill="y")
        mwrap = ttk.Frame(inner, padding=(0, 6, 0, 0)); inner.add(mwrap, text="  Component mapping  ")
        self.map = ttk.Treeview(mwrap, columns=("field", "value", "source"), show="headings", height=5)
        for col, label, width in (("field", "AMC component field", 250), ("value", "Value", 190), ("source", "Source", 230)):
            self.map.heading(col, text=label); self.map.column(col, width=width, minwidth=80, anchor="w")
        self.map.tag_configure("rec", foreground=c["text"]); self.map.tag_configure("par", foreground=c["green"])
        self.map.tag_configure("inf", foreground=c["muted"]); self.map.tag_configure("tpl", foreground=c["amber"])
        msb = ttk.Scrollbar(mwrap, orient="vertical", command=self.map.yview); self.map.configure(yscrollcommand=msb.set)
        self.map.pack(side="left", fill="both", expand=True); msb.pack(side="right", fill="y")

        self.notes.tag_configure("block", foreground="#ff8795"); self.notes.tag_configure("warn", foreground=c["amber"])
        self.notes.tag_configure("ok", foreground=c["green"]); self.notes.configure(state="disabled")

        ttk.Label(ex, text="Export to").pack(side="left")
        ttk.Entry(ex, textvariable=self.dest).pack(side="left", fill="x", expand=True, padx=8)
        ttk.Button(ex, text="Browse…", command=self.pick_dest).pack(side="left")
        self.open_btn = ttk.Button(act, text="Open in AMC", style="Secondary.TButton", command=self.open_in_amc, state="disabled")
        self.open_btn.pack(side="right", padx=(8, 0))
        self.folder_btn = ttk.Button(act, text="Open Folder", style="Secondary.TButton", command=self.open_folder, state="disabled")
        self.folder_btn.pack(side="right", padx=(8, 0))
        self.export_btn = ttk.Button(act, text="Export AMC Project", style="Secondary.TButton", command=self.export, state="disabled")
        self.export_btn.pack(side="right", padx=(8, 0))
        self.generate_btn = ttk.Button(act, text="Generate Parameter File", style="Primary.TButton",
                                       command=self.generate, state="disabled")
        self.generate_btn.pack(side="right")

    # ------------------------------------------------------------------ data
    def reload_amc(self):
        self.templates_dir = amc.find_templates_dir(self.settings)
        self.templates = amc.list_templates(self.templates_dir) if self.templates_dir else []
        cmd = amc.find_amc_command(self.settings)
        parts = []
        if self.templates_dir:
            populated = sum(1 for t in self.templates if not t.is_empty)
            parts.append(f"AMC templates  ●  {len(self.templates)} found ({populated} published builds)  —  {self.templates_dir}")
        else:
            parts.append("AMC templates  ○  not found. Install ArduPilot Methodic Configurator, or use Locate AMC… "
                         "to point at its install or source folder.")
        parts.append("AMC program  " + ("●  " + " ".join(cmd) if cmd else
                                        "○  not found — projects can still be exported and opened in AMC by hand"))
        self.engine_ok, why = engine.status()
        parts.append("Engine  " + ("●  " if self.engine_ok else "○  ") + why)
        self.amc_status.set("\n".join(parts))
        self.refresh()

    def refresh(self):
        sel = self.tree.selection()
        keep = sel[0] if sel else None
        self.tree.delete(*self.tree.get_children())
        with self.db() as c:
            rows = c.execute(
                "SELECT a.id, a.name, a.frame_type, a.frame_size, a.validation_status, "
                " (SELECT COUNT(*) FROM parameters p JOIN parameter_files f ON p.parameter_file_id=f.id "
                "   WHERE f.aircraft_id=a.id AND f.source='hardware' "
                "   AND f.id=(SELECT MAX(id) FROM parameter_files WHERE aircraft_id=a.id AND source='hardware')) AS np "
                "FROM aircraft a ORDER BY a.name").fetchall()
        for r in rows:
            self.tree.insert("", "end", iid=str(r["id"]), values=(
                r["name"], f"{r['frame_type'] or ''} {r['frame_size'] or ''}".strip(), r["validation_status"], r["np"]))
        if keep and self.tree.exists(keep):
            self.tree.selection_set(keep); self.tree.see(keep)

    def _aircraft_id(self):
        sel = self.tree.selection()
        return int(sel[0]) if sel else None

    def on_aircraft(self):
        aid = self._aircraft_id()
        if aid is None:
            return
        self.tree.see(str(aid))
        if not self.templates:
            self.reason.set("AMC templates were not found. Use Locate AMC… first.")
            return
        with self.db() as c:
            rec = amc.load_aircraft(c, aid)
        suggested, why = engine.default_template(rec, self.templates)
        if not rec.params:
            ref, _ = amc.suggest_template(rec, self.templates)
            if ref is not None and not ref.is_empty:
                why += f"  Closest published build, for reference: {ref.name}."
        vt = amc.vehicle_type_for(rec)
        pool = [t for t in self.templates if vt is None or t.vehicle_type == vt]
        self._labels = {t.label(): t for t in pool}
        self.template_box.configure(values=list(self._labels))
        self.template_var.set(suggested.label() if suggested else "")
        self.reason.set(why)
        self.overrides = {}          # choices belong to one aircraft
        self.preview()

    def _template(self):
        return getattr(self, "_labels", {}).get(self.template_var.get())

    def preview(self):
        aid, tpl = self._aircraft_id(), self._template()
        self.map.delete(*self.map.get_children())
        self.plan = None
        self.export_btn.configure(state="disabled"); self.generate_btn.configure(state="disabled")
        self._summary_base = ""
        if aid is None or tpl is None:
            self.summary.configure(text="")
            self.conflict_btn.configure(state="disabled", text="Decide AMC vs Tune")
            self._notes([("Choose an aircraft and a starting template.", "")])
            return
        with self.db() as c:
            self.plan = amc.plan_export(c, aid, tpl, same_flight_controller=self.same_fc.get(),
                                        overrides=self.overrides)
        plan = self.plan
        self._summary_base = (
            f"{len(plan.delta)} tuned parameters carried{' in an imported file' if plan.delta else ' (template only)'}"
            f"   •   {len(plan.stripped_calibration)} calibration values left out"
            f"   •   offline parameter docs {'included' if plan.metadata else 'not included'}")
        for pr in sorted(plan.provenance, key=lambda x: x.path):
            self.map.insert("", "end", values=(" › ".join(pr.path), pr.value, pr.source + (f" — {pr.note}" if pr.note else "")),
                            tags=(SOURCE_TAGS.get(pr.source, "rec"),))
        lines = [(f"BLOCKED  {b}", "block") for b in plan.blockers]
        lines += [(f"•  {w}", "block" if w.startswith("SAFETY") else "warn")
                  for w in amc.conflict_warnings(plan.conflicts)]
        lines += [(f"•  {w}", "warn") for w in plan.warnings]
        if plan.ok:
            lines.append(("Ready to export. AMC re-validates every component and asks before each write.", "ok"))
        self._notes(lines)
        self._show_conflicts(plan.conflicts, "mandatory values, whole AMC procedure")
        if plan.ok:
            self.export_btn.configure(state="normal")
            if getattr(self, "engine_ok", False) and plan.metadata is not None:
                self.generate_btn.configure(state="normal")
                self._engine_check(aid, tpl)

    # ------------------------------------------------------------------ AMC vs tune
    def _show_conflicts(self, conflicts, source):
        self.conflicts, self.conflict_source = conflicts, source
        n_amc = sum(1 for c in conflicts if c["winner"] == amc.AMC)
        n_safe = sum(1 for c in conflicts if c["safety"] and c["winner"] == amc.ATLAS)
        label = f"Decide AMC vs Tune ({len(conflicts)})"
        self.conflict_btn.configure(text=label, state="normal" if conflicts else "disabled")
        self.summary.configure(text=self._summary_base + f"\n{len(conflicts)} AMC-vs-tune disagreements: {n_amc} using AMC"
                               + (f", {n_safe} SAFETY VALUES OVERRULED" if n_safe else "") + f"   ({source})")
        if self.cwin is not None and self.cwin.winfo_exists():
            self._fill_conflict_tree()

    def open_conflicts(self):
        if self.cwin is not None and self.cwin.winfo_exists():
            self.cwin.lift(); return
        c = self.C
        win = tk.Toplevel(self.app); win.title("AMC vs tune"); win.geometry("1100x560"); win.configure(bg=c["bg"])
        win.transient(self.app); self.cwin = win
        body = ttk.Frame(win, padding=18); body.pack(fill="both", expand=True)
        name = self.plan.aircraft.name if self.plan else ""
        ttk.Label(body, text=f"Where AMC and {name}'s tune disagree", style="Big.TLabel").pack(anchor="w")
        ttk.Label(body, style="Sub.TLabel", wraplength=1040, justify="left",
                  text="Default rule: AMC's mandatory SAFETY values (failsafes, arming, fences, return-to-launch) use "
                       "AMC; everything else uses the build's validated tune. Select rows and choose, or double-click "
                       "a row to switch it. Every choice is recorded in the report.").pack(anchor="w", pady=(2, 10))
        bar = ttk.Frame(body); bar.pack(side="bottom", fill="x", pady=(10, 0))
        self.cnote = ttk.Label(bar, text="", style="Sub.TLabel"); self.cnote.pack(side="left")
        ttk.Button(bar, text="Close", style="Secondary.TButton", command=win.destroy).pack(side="right")
        ttk.Button(bar, text="Reset to Default Rule", style="Secondary.TButton",
                   command=self.reset_conflicts).pack(side="right", padx=8)
        ttk.Button(bar, text="Use Tune", style="Secondary.TButton",
                   command=lambda: self.settle(amc.ATLAS)).pack(side="right")
        ttk.Button(bar, text="Use AMC", style="Primary.TButton",
                   command=lambda: self.settle(amc.AMC)).pack(side="right", padx=8)
        wrap = ttk.Frame(body); wrap.pack(fill="both", expand=True)
        cols = ("param", "tune", "amc", "kind", "winner", "step", "why")
        self.ctree = ttk.Treeview(wrap, columns=cols, show="headings", selectmode="extended")
        for col, label, width in (("param", "Parameter", 170), ("tune", "Tune", 90), ("amc", "AMC", 90),
                                  ("kind", "AMC rule", 150), ("winner", "Using", 100), ("step", "AMC step", 200),
                                  ("why", "AMC's reason", 280)):
            self.ctree.heading(col, text=label); self.ctree.column(col, width=width, minwidth=60, anchor="w")
        self.ctree.tag_configure("amc", foreground=c["amber"]); self.ctree.tag_configure("atlas", foreground=c["text"])
        self.ctree.tag_configure("safety_atlas", foreground="#ff8795")
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.ctree.yview); self.ctree.configure(yscrollcommand=sb.set)
        self.ctree.pack(side="left", fill="both", expand=True); sb.pack(side="right", fill="y")
        self.ctree.bind("<Double-1>", lambda e: self.toggle_conflict())
        self._fill_conflict_tree()

    def _fill_conflict_tree(self):
        keep = self.ctree.selection()
        self.ctree.delete(*self.ctree.get_children())
        for c in sorted(self.conflicts, key=lambda c: (not c["safety"], c["kind"], c["parameter"])):
            kind = ("SAFETY · " if c["safety"] else "") + ("mandatory" if c["kind"] == "forced" else "calculated")
            using = ("AMC" if c["winner"] == amc.AMC else "Tune") + ("  (you)" if c["decided_by"] == "operator" else "")
            tag = "amc" if c["winner"] == amc.AMC else ("safety_atlas" if c["safety"] else "atlas")
            self.ctree.insert("", "end", iid=c["parameter"], tags=(tag,),
                              values=(c["parameter"], c["atlas"], c["amc"], kind, using, c["step"], c["reason"]))
        for k in keep:
            if self.ctree.exists(k):
                self.ctree.selection_add(k)
        n_amc = sum(1 for c in self.conflicts if c["winner"] == amc.AMC)
        self.cnote.configure(text=f"{len(self.conflicts)} disagreements, {n_amc} using AMC  ·  {self.conflict_source}")

    def _engine_check(self, aid, tpl):
        """Ask the engine for the full list (calculated values too) without writing anything."""
        self._check_token += 1
        token, same_fc, overrides = self._check_token, self.same_fc.get(), dict(self.overrides)
        self.summary.configure(text=self._summary_base + "\nChecking every value with the AMC engine …")

        def work(say):
            conn = self.db()
            try:
                return engine.generate(conn, aid, tpl, same_flight_controller=same_fc, overrides=overrides)
            finally:
                conn.close()

        def done(cfg):
            if token == self._check_token:
                self._show_conflicts(cfg.plan.conflicts, "first-flight configuration, from the engine")
                self._notes([(f"•  {w}", "block" if w.startswith(("SAFETY", "BATTERY")) else "warn")
                             for w in amc.conflict_warnings(cfg.plan.conflicts) + cfg.plan.warnings]
                            + [(f"AMC component error: {e}", "block") for e in cfg.component_errors])

        def failed(exc):
            if token == self._check_token:
                self.summary.configure(text=self._summary_base + f"\nEngine check failed: {str(exc).splitlines()[0]}")

        self.app.run_bg(work, on_done=done, on_error=failed, label="Checking with the AMC engine")

    def settle(self, winner):
        names = self.ctree.selection()
        if not names:
            messagebox.showinfo("Select parameters", "Select one or more rows first."); return
        overruling = [n for n in names if winner == amc.ATLAS and
                      next((c for c in self.conflicts if c["parameter"] == n), {}).get("safety")]
        if overruling and not messagebox.askyesno(
                "Overrule a safety value",
                "AMC sets these as safety values:\n\n   " + "\n   ".join(overruling) +
                "\n\nUse the tune's values instead? The report will record that you decided this."):
            return
        for n in names:
            self.overrides[n] = winner
        self.preview()

    def toggle_conflict(self):
        sel = self.ctree.selection()
        if len(sel) != 1:
            return
        c = next((x for x in self.conflicts if x["parameter"] == sel[0]), None)
        if c:
            self.ctree.selection_set(sel[0])
            self.settle(amc.ATLAS if c["winner"] == amc.AMC else amc.AMC)

    def reset_conflicts(self):
        self.overrides = {}
        self.preview()

    def _notes(self, lines):
        self.notes.configure(state="normal"); self.notes.delete("1.0", "end")
        for text, tag in lines:
            self.notes.insert("end", text + "\n", tag)
        self.notes.configure(state="disabled")

    # ------------------------------------------------------------------ actions
    def pick_dest(self):
        d = filedialog.askdirectory(title="Folder for AMC projects", initialdir=self.dest.get() or str(Path.home()))
        if d:
            self.dest.set(d)

    def export(self):
        if not self.plan or not self.plan.ok:
            return
        dest = Path(self.dest.get()).expanduser()
        try:
            dest.mkdir(parents=True, exist_ok=True)
            out = amc.write_export(self.plan, dest)
        except (amc.BridgeError, OSError) as exc:
            messagebox.showerror("Export failed", str(exc)); return
        self.settings["amc_export_dir"] = str(dest); amc.save_settings(self.settings)
        self.last_export = out
        self.open_btn.configure(state="normal"); self.folder_btn.configure(state="normal")
        self.app.status.set(f"AMC project written • {out}")
        n_warn = len(self.plan.warnings)
        messagebox.showinfo("AMC project exported",
                            f"{out}\n\n{len(self.plan.delta)} parameters in the imported file."
                            + (f"\n{n_warn} item(s) to review in AMC are listed in atlas_handoff.json." if n_warn else ""))

    def generate(self):
        aid, tpl = self._aircraft_id(), self._template()
        if aid is None or tpl is None:
            return
        same_fc, overrides = self.same_fc.get(), dict(self.overrides)
        dest = Path(self.dest.get()).expanduser()
        self.generate_btn.configure(state="disabled")

        def work(say):
            say("Computing configuration with the AMC engine")
            conn = self.db()   # a new connection, opened in this worker thread
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
                path = dest / f"{base}_{n}.param"; n += 1
            return cfg, engine.write_param_file(cfg, path)

        def done(result):
            cfg, path = result
            self.generate_btn.configure(state="normal")
            if path is None:
                messagebox.showerror("Configuration not generated",
                                     "AMC rejected the components:\n• " + "\n• ".join(cfg.component_errors)); return
            self.last_export = path.parent
            self.folder_btn.configure(state="normal")
            self.app.status.set(f"Configuration written • {path}")
            top = [w for w in amc.conflict_warnings(cfg.plan.conflicts) + cfg.plan.warnings
                   if w.startswith(("BATTERY", "SAFETY", "AMC value", "Battery"))][:5]
            messagebox.showinfo("First-flight configuration generated",
                                f"{path}\n\n{len(cfg.params)} parameters; {cfg.from_atlas} from this build's tune; "
                                f"{len(cfg.plan.stripped_calibration)} calibration values left out.\n\n"
                                + ("Review before flight:\n• " + "\n• ".join(top) if top else "")
                                + "\n\nThe full review is in the .report.json beside the file.")

        def failed(exc):
            self.generate_btn.configure(state="normal")
            messagebox.showerror("Configuration not generated", str(exc))

        self.app.run_bg(work, on_done=done, on_error=failed, label="Generating configuration")

    def open_in_amc(self):
        if not self.last_export:
            return
        try:
            cmd = amc.launch_amc(self.last_export, self.settings)
            self.app.status.set("Opened in AMC • " + " ".join(cmd))
        except amc.BridgeError as exc:
            messagebox.showwarning("AMC not found", str(exc))

    def open_folder(self):
        if not self.last_export:
            return
        path = str(self.last_export)
        try:
            if os.name == "nt":
                os.startfile(path)  # noqa: S606
            elif sys.platform == "darwin":
                subprocess.Popen(["open", path])
            else:
                subprocess.Popen(["xdg-open", path])
        except OSError as exc:
            messagebox.showerror("Could not open folder", str(exc))

    def locate_amc(self):
        d = filedialog.askdirectory(title="Select the AMC install, source, or vehicle_templates folder")
        if not d:
            return
        found = amc.resolve_templates_dir(d)
        if not found:
            messagebox.showwarning("Templates not found", f"No AMC vehicle templates were found under\n{d}")
            return
        self.settings["amc_templates_dir"] = str(found)
        exe = Path(d) / "ardupilot_methodic_configurator.exe"
        if exe.is_file():
            self.settings["amc_executable"] = str(exe)
        amc.save_settings(self.settings)
        self.reload_amc()

    def import_templates(self):
        if not self.templates:
            messagebox.showwarning("AMC templates not found", "Use Locate AMC… first."); return
        populated = [t for t in self.templates if not t.is_empty]
        with self.db() as c:
            present = {r[0] for r in c.execute("SELECT name FROM aircraft")}
        new = [t for t in populated if amc.template_aircraft_name(t) not in present]
        if not new:
            messagebox.showinfo("Nothing to import", "Every published AMC build is already in the repository."); return
        listing = "\n".join(f"   {amc.template_aircraft_name(t)}" for t in new[:14])
        if len(new) > 14:
            listing += f"\n   … and {len(new) - 14} more"
        if not messagebox.askyesno("Import AMC templates",
                                   f"Add {len(new)} published build(s) to the repository?\n\n{listing}\n\n"
                                   "They are stored as UNTESTED: they were configured by the AMC project, not flown by "
                                   "this team, so they are excluded from training until there is evidence on our hardware."):
            return
        with self.db() as c:
            added, _ = amc.import_templates(c, new)
        self.app.refresh_all()
        self.app.status.set(f"Imported {len(added)} AMC template build(s)")
