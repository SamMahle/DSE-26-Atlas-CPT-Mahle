import sqlite3
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
from pathlib import Path
from datetime import datetime
import shutil
import os
import sys
import queue
import subprocess
import threading
import traceback
import cbr_engine as cbr
import json

# --- Acro tuning layer -----------------------------------------------------
# These four modules are the acro_llm_tuner project. ATLAS supplies the data
# and the interface; they supply validation, prediction, and the only path to
# a flight controller.
import acro_bridge as bridge
import safety
import ml_model
import ardupilot_interface as fc
import sim_logs
import check_link
import amc_tab          # Configuration tab: AMC engine (prototype) and AMC handoff

APP_DIR = Path(__file__).resolve().parent
DB_PATH = APP_DIR / "atlas_repository.db"
FILES_DIR = APP_DIR / "repository_files"
FILES_DIR.mkdir(exist_ok=True)
LOG_DIR = APP_DIR / "sim_logs"          # raw flight logs from simulated runs
LOG_DIR.mkdir(exist_ok=True)
BENCH_DIR = APP_DIR.parent / "sitl_bench"   # the standardized SITL test bench
BENCH_GROUP_PREFIX = "bench_"               # run_group prefix of bench runs in sim_runs

VALIDATION = ["UNTESTED", "PARTIALLY_VALIDATED", "FULLY_VALIDATED", "REJECTED"]
# Frame and GPS were added in v9: import_build.py classifies them, so the
# catalog has to be able to hold and offer them too.
CATEGORIES = ["Flight Controller", "ESC", "Motor", "Frame", "GPS", "Receiver", "VTX", "Payload"]
STARTER_HARDWARE = {
    "Flight Controller": {"Holybro": ["Pixhawk 6C", "Pixhawk 6X", "Durandal"], "CubePilot": ["Cube Orange+", "Cube Red", "Cube Yellow"], "Matek": ["H743-WING V3", "H743-SLIM", "F405-WTE"]},
    "ESC": {"Holybro": ["Tekko32 F4 45A", "Tekko32 F4 65A"], "T-Motor": ["F55A Pro II", "F60A Mini"], "Hobbywing": ["XRotor Micro 60A", "XRotor 40A"]},
    "Motor": {"T-Motor": ["F60 Pro V", "F90", "MN3110"], "BrotherHobby": ["Avenger 2806.5", "VY 2004"], "iFlight": ["XING2 2806.5", "XING2 2207"]}
}

def db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    return conn

def init_db():
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
            now = datetime.now().isoformat(timespec="seconds")
            for cat, makes in STARTER_HARDWARE.items():
                for make, models in makes.items():
                    for model in models:
                        c.execute("INSERT OR IGNORE INTO hardware_catalog(category,manufacturer,model,created_at) VALUES(?,?,?,?)", (cat,make,model,now))

def parse_param(path):
    out=[]
    for raw in Path(path).read_text(errors="ignore").splitlines():
        line=raw.strip()
        if not line or line.startswith(("#","//")): continue
        if "," in line: a,b=line.split(",",1)
        elif "\t" in line: a,b=line.split("\t",1)
        elif " " in line: a,b=line.split(None,1)
        else: continue
        a,b=a.strip(),b.strip()
        if a: out.append((a,b))
    return out

APP_VERSION = "9.1 ACRO"

# Tab indices, so the quick-action buttons don't depend on counting.
# Order must match the build_* calls in __init__. The Standard Bench tab was
# added without updating this line, which sent the Repository and Hardware
# Catalog buttons to the wrong tabs.
TAB_OVERVIEW, TAB_INGEST, TAB_CBR, TAB_PREDICT, TAB_FC, TAB_SIM, TAB_BENCH, TAB_REPO, TAB_HW, TAB_AMC = range(10)


class ATLAS(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ATLAS // Acro Configuration Intelligence")
        self.geometry("1380x900"); self.minsize(1140,760)
        self.option_add("*Font", ("Segoe UI", 10))
        self.param_path=tk.StringVar(); self.log_path=tk.StringVar(); self.status=tk.StringVar(value="SYSTEM READY   •   LOCAL REPOSITORY   •   TRACEABILITY ENABLED")
        self.vehicle=None                 # the one live flight-controller link
        self.worker_q=queue.Queue()       # background thread -> GUI thread
        self.last_prediction=None
        self._style(); self._header()
        self.tabs=ttk.Notebook(self); self.tabs.pack(fill="both",expand=True,padx=24,pady=(8,10))
        self.build_home(); self.build_ingest(); self.build_search()
        self.build_predict(); self.build_fc(); self.build_sim(); self.build_bench()
        self.build_browser(); self.build_hardware()
        self.amc=amc_tab.AMCHandoffTab(self,self.tabs,db)
        ttk.Label(self,textvariable=self.status,style="Status.TLabel").pack(fill="x",padx=24,pady=(0,12))
        self.refresh_all()
        self.after(120, self._drain_worker_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------------
    # Background work. Tkinter is not thread safe, so a worker thread never
    # touches a widget — it posts messages and the GUI thread applies them.
    # ---------------------------------------------------------------------
    def run_bg(self, fn, *, on_done=None, on_error=None, label="working"):
        def wrapper():
            try:
                result = fn(lambda m: self.worker_q.put(("progress", m, None)))
                self.worker_q.put(("done", result, on_done))
            except Exception as exc:                       # noqa: BLE001
                self.worker_q.put(("error", (exc, traceback.format_exc()), on_error))
        self.status.set(label + " …")
        threading.Thread(target=wrapper, daemon=True).start()

    def _drain_worker_queue(self):
        try:
            while True:
                kind, payload, cb = self.worker_q.get_nowait()
                if kind == "progress":
                    self.status.set(str(payload))
                elif kind == "done":
                    self.status.set("Ready")
                    if cb: cb(payload)
                elif kind == "error":
                    exc, tb = payload
                    self.status.set(f"Failed: {exc}")
                    if cb: cb(exc)
                    else: messagebox.showerror("Operation failed", str(exc))
        except queue.Empty:
            pass
        self.after(120, self._drain_worker_queue)

    def _on_close(self):
        if self.vehicle is not None:
            try: self.vehicle.close()
            except Exception: pass
        self.destroy()

    def _style(self):
        s=ttk.Style(self)
        try: s.theme_use("clam")
        except: pass
        self.C={"bg":"#0b0f14","panel":"#121821","panel2":"#18212c","line":"#263241","text":"#edf2f7","muted":"#8fa0b3","red":"#d33a4e","red2":"#a51f35","green":"#43c17a","amber":"#e6ad45"}
        c=self.C; self.configure(bg=c["bg"])
        self.option_add("*Font", ("Segoe UI",10))
        self.option_add("*TCombobox*Listbox.font", ("Segoe UI",10))
        s.configure("TFrame",background=c["bg"]); s.configure("TLabel",background=c["bg"],foreground=c["text"])
        s.configure("Title.TLabel",font=("Segoe UI Semibold",28),foreground=c["text"])
        s.configure("Eyebrow.TLabel",font=("Segoe UI Semibold",9),foreground=c["red"])
        s.configure("Sub.TLabel",foreground=c["muted"]); s.configure("Big.TLabel",font=("Segoe UI Semibold",19),foreground=c["text"])
        s.configure("Metric.TLabel",font=("Segoe UI Semibold",24),foreground=c["text"],background=c["panel"])
        s.configure("MetricSub.TLabel",font=("Segoe UI",9),foreground=c["muted"],background=c["panel"])
        s.configure("Card.TLabelframe",background=c["panel"],padding=16,relief="solid",borderwidth=1,bordercolor=c["line"])
        s.configure("Card.TLabelframe.Label",font=("Segoe UI Semibold",10),background=c["bg"],foreground=c["muted"])
        s.configure("Primary.TButton",font=("Segoe UI Semibold",10),padding=(16,10),foreground="white",background=c["red"],borderwidth=0)
        s.map("Primary.TButton",background=[("active",c["red2"]),("pressed",c["red2"])])
        s.configure("Secondary.TButton",font=("Segoe UI Semibold",10),padding=(13,9),foreground=c["text"],background=c["panel2"],borderwidth=0)
        s.map("Secondary.TButton",background=[("active",c["line"])])
        s.configure("Danger.TButton",padding=(12,8),foreground="#ff8795",background=c["panel2"],borderwidth=0)
        s.configure("Status.TLabel",foreground=c["muted"],background=c["bg"],font=("Segoe UI",9))
        s.configure("Treeview",rowheight=34,background=c["panel"],fieldbackground=c["panel"],foreground=c["text"],borderwidth=0)
        s.map("Treeview",background=[("selected",c["red2"])],foreground=[("selected","white")])
        s.configure("Treeview.Heading",font=("Segoe UI Semibold",9),background=c["panel2"],foreground=c["muted"],relief="flat",padding=(8,8))
        s.map("Treeview.Heading",background=[("active",c["line"])])
        s.configure("TNotebook",background=c["bg"],borderwidth=0); s.configure("TNotebook.Tab",padding=(14,11),background=c["panel"],foreground=c["muted"],borderwidth=0)
        s.map("TNotebook.Tab",background=[("selected",c["panel2"])],foreground=[("selected",c["text"])])
        s.configure("TEntry",fieldbackground=c["panel2"],foreground=c["text"],insertcolor=c["text"],padding=7)
        s.configure("TCombobox",fieldbackground=c["panel2"],foreground=c["text"],arrowcolor=c["muted"],padding=6)
        s.map("TCombobox",fieldbackground=[("readonly",c["panel2"])],foreground=[("readonly",c["text"])])
        s.configure("Vertical.TScrollbar",background=c["panel2"],troughcolor=c["bg"],borderwidth=0)

    def _header(self):
        c=self.C
        shell=tk.Frame(self,bg=c["bg"],height=82); shell.pack(fill="x",padx=28,pady=(18,2)); shell.pack_propagate(False)
        left=tk.Frame(shell,bg=c["bg"]); left.pack(side="left",fill="y")
        tk.Label(left,text="ATLAS",font=("Segoe UI Semibold",27),fg=c["text"],bg=c["bg"]).pack(side="left",anchor="s",pady=(12,10))
        tk.Label(left,text="  //  ACRO CONFIGURATION INTELLIGENCE",font=("Segoe UI Semibold",9),fg=c["red"],bg=c["bg"]).pack(side="left",anchor="s",pady=(0,17))
        badge=tk.Frame(shell,bg=c["panel"],highlightbackground=c["line"],highlightthickness=1); badge.pack(side="right",anchor="e",pady=16)
        tk.Label(badge,text="●",fg=c["green"],bg=c["panel"],font=("Segoe UI",9)).pack(side="left",padx=(12,5),pady=8)
        tk.Label(badge,text="LOCAL / OFFLINE",fg=c["text"],bg=c["panel"],font=("Segoe UI Semibold",9)).pack(side="left",padx=(0,12),pady=8)

    def field(self,parent,label,var,row,col=0,width=30,values=None,required=False,state="normal"):
        ttk.Label(parent,text=label+("  *" if required else "")).grid(row=row,column=col,sticky="w",padx=6,pady=(7,2))
        w=ttk.Entry(parent,textvariable=var,width=width) if values is None else ttk.Combobox(parent,textvariable=var,values=values,width=width-2,state=state)
        w.grid(row=row+1,column=col,sticky="ew",padx=6,pady=(0,7)); parent.columnconfigure(col,weight=1); return w

    def build_home(self):
        p=ttk.Frame(self.tabs,padding=24); self.tabs.add(p,text="  OVERVIEW  ")
        ttk.Label(p,text="SYSTEM OVERVIEW",style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(p,text="Configuration Repository",style="Big.TLabel").pack(anchor="w",pady=(3,2))
        ttk.Label(p,text="Trace hardware, parameter snapshots, flight evidence, and validation from one local workspace.",style="Sub.TLabel").pack(anchor="w",pady=(0,20))
        stats=tk.Frame(p,bg=self.C["bg"]); stats.pack(fill="x")
        self.stat_air=self._metric_card(stats,"0","AIRCRAFT")
        self.stat_comp=self._metric_card(stats,"0","INSTALLED COMPONENTS")
        self.stat_hw=self._metric_card(stats,"0","CATALOG HARDWARE")
        self.stat_param=self._metric_card(stats,"0","PARAMETERS")
        self.stat_train=self._metric_card(stats,"0","TRAINABLE CASES")
        actions=ttk.LabelFrame(p,text="QUICK ACTIONS",style="Card.TLabelframe"); actions.pack(fill="x",pady=(22,14))
        ttk.Button(actions,text="＋  New Configuration",style="Primary.TButton",command=lambda:self.tabs.select(TAB_INGEST)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="⌕  Find Match",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_CBR)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="Predict Tuning",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_PREDICT)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="Flight Controller",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_FC)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="Repository",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_REPO)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="Hardware Catalog",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_HW)).pack(side="left",padx=7,pady=8)
        ttk.Button(actions,text="Configuration",style="Secondary.TButton",command=lambda:self.tabs.select(TAB_AMC)).pack(side="left",padx=7,pady=8)
        info=ttk.LabelFrame(p,text="HOW A TUNE REACHES AN AIRCRAFT",style="Card.TLabelframe"); info.pack(fill="x")
        ttk.Label(info,text="RETRIEVE OR PREDICT     →     SAFETY BOUNDS     →     YOU CONFIRM     →     MAVLINK WRITE     →     FLIGHT EVIDENCE",font=("Segoe UI Semibold",10),foreground=self.C["text"],background=self.C["panel"]).pack(anchor="w",pady=(8,4))
        ttk.Label(info,text="No model output reaches an aircraft directly. Case retrieval and the prediction model both hand their proposal to the same fail-closed check, which rejects anything outside its approved range or not on its list at all — then a person confirms before a single parameter is written.",style="Sub.TLabel",background=self.C["panel"],wraplength=1200,justify="left").pack(anchor="w",pady=(0,8))
        ttk.Label(info,text="FULL BUILD     →     AMC HANDOFF     →     AMC CONFIGURES, CALIBRATES, TUNES     →     AMC WRITE     →     FLIGHT EVIDENCE",font=("Segoe UI Semibold",10),foreground=self.C["text"],background=self.C["panel"]).pack(anchor="w",pady=(10,4))
        ttk.Label(info,text="A whole configuration — not just the Acro tune — goes to ArduPilot Methodic Configurator as a project. ATLAS strips the source board's calibrations and runs the same Acro bounds check; AMC then walks every configuration step, keeps its own mandatory safety values, and asks before each write.",style="Sub.TLabel",background=self.C["panel"],wraplength=1200,justify="left").pack(anchor="w",pady=(0,8))

    def _metric_card(self,parent,value,label):
        card=tk.Frame(parent,bg=self.C["panel"],highlightbackground=self.C["line"],highlightthickness=1)
        card.pack(side="left",fill="x",expand=True,padx=6,ipady=10)
        val=tk.Label(card,text=value,font=("Segoe UI Semibold",25),fg=self.C["text"],bg=self.C["panel"]); val.pack(anchor="w",padx=16,pady=(12,0))
        tk.Label(card,text=label,font=("Segoe UI Semibold",8),fg=self.C["muted"],bg=self.C["panel"]).pack(anchor="w",padx=16,pady=(1,12))
        return val

    def catalog_makes(self,category):
        with db() as c: return [r[0] for r in c.execute("SELECT DISTINCT manufacturer FROM hardware_catalog WHERE category=? AND active=1 ORDER BY manufacturer",(category,))]
    def catalog_models(self,category,make):
        with db() as c: return [r[0] for r in c.execute("SELECT model FROM hardware_catalog WHERE category=? AND manufacturer=? AND active=1 ORDER BY model",(category,make))]

    def build_ingest(self):
        outer=ttk.Frame(self.tabs,padding=16); self.tabs.add(outer,text="  Add Aircraft  ")
        canvas=tk.Canvas(outer,highlightthickness=0,bg=self.C["bg"]); sb=ttk.Scrollbar(outer,orient="vertical",command=canvas.yview)
        form=ttk.Frame(canvas,padding=(8,4,16,20)); form.bind("<Configure>",lambda e:canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0,0),window=form,anchor="nw",tags="form"); canvas.bind("<Configure>",lambda e:canvas.itemconfigure("form",width=e.width)); canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left",fill="both",expand=True); sb.pack(side="right",fill="y")
        ttk.Label(form,text="New Aircraft Configuration",style="Big.TLabel").pack(anchor="w")
        ttk.Label(form,text="Create the configuration record, attach evidence, and assign its current validation state.",style="Sub.TLabel").pack(anchor="w",pady=(2,14))
        keys=["name","frame_type","frame_size","fc_make","fc_model","esc_make","esc_model","motor_make","motor_model","motor_stator","motor_kv","motor_qty","rx_make","rx_model","vtx_make","vtx_model","payload","ardupilot","test_date","test_result","validation","weight_g","prop_size_in","cell_count","wheelbase_mm"]
        self.vars={k:tk.StringVar() for k in keys}; self.vars["motor_qty"].set("4"); self.vars["validation"].set("UNTESTED"); self.vars["test_result"].set("Not evaluated")
        a=ttk.LabelFrame(form,text="1. Aircraft identity",style="Card.TLabelframe"); a.pack(fill="x",pady=6)
        self.field(a,"Aircraft name",self.vars["name"],0,0,required=True); self.field(a,"Frame type",self.vars["frame_type"],0,1,values=["Quadcopter","Hexacopter","Octocopter","Fixed Wing","VTOL","Other"],required=True); self.field(a,"Frame size",self.vars["frame_size"],0,2)
        hw=ttk.LabelFrame(form,text="2. Core hardware",style="Card.TLabelframe"); hw.pack(fill="x",pady=6)
        self.fc_make_cb=self.field(hw,"Flight controller manufacturer",self.vars["fc_make"],0,0,values=self.catalog_makes("Flight Controller"),required=True)
        self.fc_model_cb=self.field(hw,"Flight controller model",self.vars["fc_model"],0,1,values=[],required=True)
        self.esc_make_cb=self.field(hw,"ESC manufacturer",self.vars["esc_make"],2,0,values=self.catalog_makes("ESC")); self.esc_model_cb=self.field(hw,"ESC model",self.vars["esc_model"],2,1,values=[])
        self.motor_make_cb=self.field(hw,"Motor manufacturer",self.vars["motor_make"],4,0,values=self.catalog_makes("Motor")); self.motor_model_cb=self.field(hw,"Motor model",self.vars["motor_model"],4,1,values=[])
        for cat,mv,modv,cb in [("Flight Controller",self.vars["fc_make"],self.vars["fc_model"],self.fc_model_cb),("ESC",self.vars["esc_make"],self.vars["esc_model"],self.esc_model_cb),("Motor",self.vars["motor_make"],self.vars["motor_model"],self.motor_model_cb)]:
            make_cb={"Flight Controller":self.fc_make_cb,"ESC":self.esc_make_cb,"Motor":self.motor_make_cb}[cat]
            make_cb.bind("<<ComboboxSelected>>",lambda e,c=cat,m=mv,mo=modv,w=cb:self.update_models(c,m,mo,w))
        self.field(hw,"Motor stator",self.vars["motor_stator"],6,0,values=["2004","2207","2306","2506","2806.5","2812","3110","Other / Custom"]); self.field(hw,"Motor KV",self.vars["motor_kv"],6,1); self.field(hw,"Motor quantity",self.vars["motor_qty"],6,2,values=["1","2","4","6","8"])
        ttk.Button(hw,text="Manage hardware catalog…",command=lambda:self.tabs.select(TAB_HW)).grid(row=8,column=2,sticky="e",padx=6,pady=8)
        meas=ttk.LabelFrame(form,text="3. Airframe measurements  —  required for parameter prediction",style="Card.TLabelframe"); meas.pack(fill="x",pady=6)
        ttk.Label(meas,text="The prediction model learns from these. Without them an aircraft still stores and still works for case retrieval, but it cannot train or receive a prediction.",style="Sub.TLabel",background=self.C["panel"],wraplength=1150,justify="left").grid(row=0,column=0,columnspan=4,sticky="w",padx=6,pady=(0,6))
        self.field(meas,"All-up weight (g)",self.vars["weight_g"],1,0)
        self.field(meas,"Prop size (in)",self.vars["prop_size_in"],1,1,values=["2","2.5","3","3.5","4","5","6","7","8","10"])
        self.field(meas,"Battery cells (S)",self.vars["cell_count"],1,2,values=["2","3","4","5","6","8","12"])
        self.field(meas,"Wheelbase (mm)",self.vars["wheelbase_mm"],1,3)
        self.measure_hint=ttk.Label(meas,text="",style="Sub.TLabel",background=self.C["panel"])
        self.measure_hint.grid(row=3,column=0,columnspan=4,sticky="w",padx=6,pady=(2,4))
        opt=ttk.LabelFrame(form,text="4. Optional equipment",style="Card.TLabelframe"); opt.pack(fill="x",pady=6)
        self.field(opt,"Receiver manufacturer",self.vars["rx_make"],0,0); self.field(opt,"Receiver model",self.vars["rx_model"],0,1); self.field(opt,"VTX manufacturer",self.vars["vtx_make"],2,0); self.field(opt,"VTX model",self.vars["vtx_model"],2,1); self.field(opt,"Payload / interface",self.vars["payload"],4,0)
        ev=ttk.LabelFrame(form,text="5. Configuration evidence & validation",style="Card.TLabelframe"); ev.pack(fill="x",pady=6)
        ttk.Label(ev,text="ArduPilot parameter file (.param)").grid(row=0,column=0,sticky="w",padx=6,pady=(7,2)); ttk.Entry(ev,textvariable=self.param_path).grid(row=1,column=0,sticky="ew",padx=6,pady=(0,7)); ttk.Button(ev,text="Browse…",command=self.pick_param).grid(row=1,column=1,padx=6)
        ttk.Label(ev,text="Flight log (optional)").grid(row=2,column=0,sticky="w",padx=6,pady=(7,2)); ttk.Entry(ev,textvariable=self.log_path).grid(row=3,column=0,sticky="ew",padx=6,pady=(0,7)); ttk.Button(ev,text="Browse…",command=self.pick_log).grid(row=3,column=1,padx=6); ev.columnconfigure(0,weight=1)
        self.field(ev,"ArduPilot version",self.vars["ardupilot"],4,0); self.field(ev,"Test date",self.vars["test_date"],6,0); self.field(ev,"Test result",self.vars["test_result"],6,1,values=["Not evaluated","Pass","Fail","Mixed"]); self.field(ev,"Validation status",self.vars["validation"],8,0,values=VALIDATION,required=True,state="readonly")
        ttk.Label(ev,text="Notes / validation evidence").grid(row=10,column=0,sticky="w",padx=6,pady=(7,2)); self.notes=tk.Text(ev,height=4,wrap="word",relief="flat",borderwidth=0,bg=self.C["panel2"],fg=self.C["text"],insertbackground=self.C["text"],padx=10,pady=8); self.notes.grid(row=11,column=0,columnspan=2,sticky="ew",padx=6,pady=(0,7))
        ttk.Button(form,text="Save Configuration",style="Primary.TButton",command=self.ingest).pack(anchor="e",pady=14)

    def update_models(self,category,make_var,model_var,widget):
        vals=self.catalog_models(category,make_var.get().strip()); widget.configure(values=vals)
        if model_var.get().strip() not in vals: model_var.set("")
    def refresh_ingest_catalog(self):
        if not hasattr(self,"fc_make_cb"): return
        for cat,w in [("Flight Controller",self.fc_make_cb),("ESC",self.esc_make_cb),("Motor",self.motor_make_cb)]: w.configure(values=self.catalog_makes(cat))
        self.update_models("Flight Controller",self.vars["fc_make"],self.vars["fc_model"],self.fc_model_cb); self.update_models("ESC",self.vars["esc_make"],self.vars["esc_model"],self.esc_model_cb); self.update_models("Motor",self.vars["motor_make"],self.vars["motor_model"],self.motor_model_cb)

    def pick_param(self):
        p=filedialog.askopenfilename(title="Select ArduPilot parameter file",filetypes=[("ArduPilot parameter files","*.param *.parm *.txt"),("All files","*.*")]);
        if p:self.param_path.set(p)
    def pick_log(self):
        p=filedialog.askopenfilename(title="Select flight log",filetypes=[("Flight logs","*.bin *.log *.tlog"),("All files","*.*")]);
        if p:self.log_path.set(p)
    def add_component(self,c,aid,cat,make,model,specs=""):
        if any(str(x).strip() for x in (make,model,specs)): c.execute("INSERT INTO components(aircraft_id,category,manufacturer,model,specs) VALUES(?,?,?,?,?)",(aid,cat,str(make).strip(),str(model).strip(),str(specs).strip()))

    def ingest(self):
        name=self.vars["name"].get().strip(); frame=self.vars["frame_type"].get().strip(); fc=self.vars["fc_model"].get().strip()
        if not name or not frame or not fc: messagebox.showwarning("Missing required information","Aircraft name, frame type, and flight controller are required."); return
        try:
            def meas(key):
                raw=self.vars[key].get().strip()
                if not raw: return None
                try: return float(raw)
                except ValueError: raise ValueError(f"{key.replace('_',' ')}: {raw!r} is not a number")
            w_g, prop_in, cells, wheel = meas("weight_g"), meas("prop_size_in"), meas("cell_count"), meas("wheelbase_mm")
            with db() as c:
                aid=c.execute("INSERT INTO aircraft(name,frame_type,frame_size,validation_status,notes,created_at,weight_g,prop_size_in,cell_count,wheelbase_mm) VALUES(?,?,?,?,?,?,?,?,?,?)",(name,frame,self.vars["frame_size"].get().strip(),self.vars["validation"].get(),self.notes.get("1.0","end").strip(),datetime.now().isoformat(timespec="seconds"),w_g,prop_in,int(cells) if cells else None,wheel)).lastrowid
                self.add_component(c,aid,"Flight Controller",self.vars["fc_make"].get(),self.vars["fc_model"].get()); self.add_component(c,aid,"ESC",self.vars["esc_make"].get(),self.vars["esc_model"].get())
                ms=", ".join(x for x in [("Stator "+self.vars["motor_stator"].get().strip()) if self.vars["motor_stator"].get().strip() else "",(self.vars["motor_kv"].get().strip()+" KV") if self.vars["motor_kv"].get().strip() else "",("Qty "+self.vars["motor_qty"].get().strip()) if self.vars["motor_qty"].get().strip() else ""] if x)
                self.add_component(c,aid,"Motor",self.vars["motor_make"].get(),self.vars["motor_model"].get(),ms); self.add_component(c,aid,"Receiver",self.vars["rx_make"].get(),self.vars["rx_model"].get()); self.add_component(c,aid,"VTX",self.vars["vtx_make"].get(),self.vars["vtx_model"].get()); self.add_component(c,aid,"Payload","",self.vars["payload"].get())
                pf = None
                if self.param_path.get():
                    src=Path(self.param_path.get()); dest=FILES_DIR/f"{aid}_{src.name}"; shutil.copy2(src,dest); pf=c.execute("INSERT INTO parameter_files(aircraft_id,original_name,stored_path,ardupilot_version,imported_at) VALUES(?,?,?,?,?)",(aid,src.name,str(dest),self.vars["ardupilot"].get().strip(),datetime.now().isoformat(timespec="seconds"))).lastrowid; rows=parse_param(src); c.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",[(pf,a,b) for a,b in rows])
                if self.log_path.get():
                    src=Path(self.log_path.get()); dest=FILES_DIR/f"{aid}_{src.name}"; shutil.copy2(src,dest); c.execute("INSERT INTO flight_tests(aircraft_id,parameter_file_id,log_name,stored_path,test_date,result,notes) VALUES(?,?,?,?,?,?,?)",(aid,pf,src.name,str(dest),self.vars["test_date"].get().strip(),self.vars["test_result"].get(),self.notes.get("1.0","end").strip()))
            self.status.set(f"Saved {name} • {self.vars['validation'].get()}"); messagebox.showinfo("Configuration saved",f"{name} was added to ATLAS."); self.clear_ingest(); self.refresh_all()
        except sqlite3.IntegrityError: messagebox.showerror("Aircraft already exists","Use a unique aircraft name.")
        except Exception as e: messagebox.showerror("Could not save configuration",str(e))
    def clear_ingest(self):
        for v in self.vars.values(): v.set("")
        self.vars["motor_qty"].set("4"); self.vars["validation"].set("UNTESTED"); self.vars["test_result"].set("Not evaluated"); self.param_path.set(""); self.log_path.set(""); self.notes.delete("1.0","end")

    def build_search(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  CBR Assistant  ")
        ttk.Label(p,text="Explainable Case-Based Reasoning",style="Big.TLabel").pack(anchor="w")
        ttk.Label(p,text="RETRIEVE → REUSE → REVISE → RETAIN  |  Similarity is not flight validation.",style="Sub.TLabel").pack(anchor="w",pady=(2,12))
        box=ttk.LabelFrame(p,text="1. Describe target aircraft",style="Card.TLabelframe");box.pack(fill="x")
        self.search_vars={k:tk.StringVar() for k in cbr.WEIGHTS}
        fields=[("frame","Frame type"),("frame_size","Frame size"),("fc","Flight controller make/model"),("esc","ESC make/model"),("motor","Motor make/model"),("kv","Motor KV"),("stator","Motor stator"),("quantity","Motor quantity")]
        for i,(key,label) in enumerate(fields):
            self.field(box,label,self.search_vars[key],(i//4)*2,i%4,values=["Quadcopter","Hexacopter","Octocopter","Fixed Wing","VTOL"] if key=="frame" else None)
        bar=ttk.Frame(p);bar.pack(fill="x",pady=10)
        ttk.Button(bar,text="Rank Similar Cases",style="Primary.TButton",command=self.search).pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Adjust Weights",command=self.configure_weights).pack(side="left",padx=4)
        ttk.Button(bar,text="Export Candidate .param",command=self.export_candidate).pack(side="left",padx=4)
        self.weights=dict(cbr.WEIGHTS);self.matches=[]
        cols=("score","name","status","files")
        self.match_tree=ttk.Treeview(p,columns=cols,show="headings",height=7)
        for k,title,width in [("score","Similarity",105),("name","Source aircraft",270),("status","Evidence status",210),("files","Param files",100)]:
            self.match_tree.heading(k,text=title);self.match_tree.column(k,width=width)
        self.match_tree.pack(fill="x",pady=(0,8));self.match_tree.bind("<<TreeviewSelect>>",self.show_match)
        reswrap=ttk.Frame(p); reswrap.pack(fill="both",expand=True)
        self.result=tk.Text(reswrap,height=12,wrap="word",font=("Consolas",10),bg=self.C["panel"],fg=self.C["text"],insertbackground=self.C["text"],padx=14,pady=12)
        res_sb=ttk.Scrollbar(reswrap,orient="vertical",command=self.result.yview)
        self.result.configure(yscrollcommand=res_sb.set)
        self.result.pack(side="left",fill="both",expand=True); res_sb.pack(side="right",fill="y")
        self.result.insert("end","Enter known hardware and select Rank Similar Cases. Unknown fields are excluded from the score.\nNo result constitutes a safety or compatibility certification.")

    def search(self):
        target={k:v.get().strip() for k,v in self.search_vars.items()}
        if not any(target.values()):messagebox.showwarning("Missing target","Enter at least one hardware attribute.");return
        with db() as c:self.matches=cbr.retrieve(c,target,self.weights)
        for item in self.match_tree.get_children():self.match_tree.delete(item)
        for r in self.matches:self.match_tree.insert("","end",iid=str(r['id']),values=(f"{r['score']:.1f}%",r['name'],r['status'],r['files']))
        self.result.delete("1.0","end")
        if not self.matches:self.result.insert("end","No saved cases yet. Add aircraft and parameter snapshots first.");return
        self.match_tree.selection_set(str(self.matches[0]['id']));self.show_match()

    def show_match(self,event=None):
        sel=self.match_tree.selection()
        if not sel:return
        r=next((r for r in self.matches if str(r['id'])==sel[0]),None)
        if not r:return
        self.result.delete("1.0","end")
        self.result.insert("end",f"SOURCE: {r['name']}\nSIMILARITY: {r['score']}% (weighted retrieval only)\nVALIDATION OF SOURCE: {r['status']}\nPARAMETER SNAPSHOTS: {r['files']}\n\nFEATURE EXPLANATIONS\n")
        for d in r['details']:self.result.insert("end",f"{d['field'].upper():12} {d['score']:3}%  {d['reason']}\n   target: {d['target']} | stored: {d['case'] or '(missing)'}\n")
        self.result.insert("end","\nCandidate parameters are unvalidated for the target aircraft. Check firmware, hardware mapping, failsafes, and flight-dependent tuning before use.\n")

    def configure_weights(self):
        win=tk.Toplevel(self);win.title("CBR Feature Weights");win.geometry("390x460")
        frm=ttk.Frame(win,padding=18);frm.pack(fill="both",expand=True)
        ttk.Label(frm,text="Retrieval Feature Weights",style="Big.TLabel").pack(anchor="w")
        ttk.Label(frm,text="Relative importance; zero excludes a feature.",style="Sub.TLabel").pack(anchor="w",pady=8)
        vars={}
        for key,val in self.weights.items():
            row=ttk.Frame(frm);row.pack(fill="x",pady=3);ttk.Label(row,text=key.upper(),width=16).pack(side="left");v=tk.StringVar(value=str(val));vars[key]=v;ttk.Entry(row,textvariable=v,width=10).pack(side="left")
        def save():
            try:
                new={k:float(v.get()) for k,v in vars.items()}
                if any(v<0 for v in new.values()) or not any(new.values()):raise ValueError('Weights must be nonnegative and at least one positive.')
                self.weights=new;win.destroy();self.search()
            except ValueError as e:messagebox.showerror("Invalid weights",str(e),parent=win)
        ttk.Button(frm,text="Apply Weights",style="Primary.TButton",command=save).pack(pady=12)

    def export_candidate(self):
        sel=self.match_tree.selection()
        if not sel:messagebox.showinfo("Select a case","Rank cases and select a source first.");return
        r=next(x for x in self.matches if str(x['id'])==sel[0])
        try:
            with db() as c:pf,params,warnings=cbr.candidate(c,r['id'])

            # v9: the Acro tuning parameters inside the snapshot are now checked
            # against safety.py before leaving the application. v8 copied them
            # out with only a prefix-based text warning, so a stored gain
            # outside the approved envelope left ATLAS unflagged.
            acro_in_file={k:v for k,v in params.items() if k in safety.ACRO_PARAM_RANGES}
            check=safety.check_param_set(acro_in_file)
            if check["blocked"]:
                lines="\n".join("   • "+b["message"] for b in check["blocked"])
                if not messagebox.askyesno("Acro parameters outside approved range",
                        f"{len(check['blocked'])} of {len(acro_in_file)} Acro tuning parameters "
                        f"in this snapshot fall outside the range this tool treats as safe:\n\n{lines}\n\n"
                        "These are the source aircraft's real stored values, so this may mean the "
                        "approved range needs widening for this class of airframe — or that the "
                        "snapshot is wrong.\n\nExport anyway, with these flagged in the review report?"):
                    return

            path=filedialog.asksaveasfilename(title="Save unvalidated candidate",defaultextension=".param",initialfile="ATLAS_CANDIDATE_UNVALIDATED.param",filetypes=[("ArduPilot parameters","*.param")])
            if not path:return
            out=Path(path);out.write_text(f"# ATLAS v{APP_VERSION} CANDIDATE - NOT FLIGHT VALIDATED\n# Source case: "+r['name']+f"\n# Similarity {r['score']}% is a retrieval score, not a safety score\n# Acro tuning parameters within approved range: {len(check['writable'])}/{len(acro_in_file)}\n"+"".join(f"{k},{v}\n" for k,v in params.items()),encoding="utf-8")
            report={"candidate_status":"UNTESTED","source_aircraft":r['name'],"source_validation":r['status'],"similarity_percent":r['score'],"source_parameter_file":pf['original_name'],"source_firmware":pf['ardupilot_version'],"features":r['details'],"warnings":warnings,
                    "acro_safety_check":{"checked":len(acro_in_file),"in_range":len(check["writable"]),"blocked":[b["message"] for b in check["blocked"]]},
                    "required_review":["Confirm target firmware and vehicle type","Review frame, motor and servo mappings","Review arming and failsafe settings","Tune flight-dependent parameters using appropriate tests","Perform bench, SITL and controlled flight validation"]}
            out.with_suffix('.review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            messagebox.showinfo("Candidate exported",f"Saved {len(params)} parameters and {len(warnings)} review flags.\nAcro tuning parameters within approved range: {len(check['writable'])}/{len(acro_in_file)}\n\n{out}\n{out.with_suffix('.review.json')}\n\nNOT flight validated.")
        except Exception as e:messagebox.showerror("Export failed",str(e))

    # =====================================================================
    # PREDICT — the multi-output random forest
    # =====================================================================
    def build_predict(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Predict Tuning  ")
        ttk.Label(p,text="PARAMETER PREDICTION",style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(p,text="Multi-Output Regression",style="Big.TLabel").pack(anchor="w",pady=(3,2))
        ttk.Label(p,text="Predicts a whole Acro tuning set at once from hardware measurements, learning only from configurations this repository records as having flown well.",style="Sub.TLabel",wraplength=1200,justify="left").pack(anchor="w",pady=(0,12))

        census=ttk.LabelFrame(p,text="TRAINING DATA",style="Card.TLabelframe"); census.pack(fill="x")
        self.census_label=ttk.Label(census,text="",style="Sub.TLabel",background=self.C["panel"],justify="left",wraplength=1180)
        self.census_label.pack(anchor="w",pady=(2,6))
        self.census_detail=ttk.Label(census,text="",style="Sub.TLabel",background=self.C["panel"],justify="left",wraplength=1180)
        self.census_detail.pack(anchor="w",pady=(0,4))

        box=ttk.LabelFrame(p,text="TARGET AIRCRAFT",style="Card.TLabelframe"); box.pack(fill="x",pady=(10,0))
        self.pred_vars={f:tk.StringVar() for f in bridge.FEATURE_FIELDS}
        for i,f in enumerate(bridge.FEATURE_FIELDS):
            self.field(box,bridge.FEATURE_LABELS[f],self.pred_vars[f],0,i,width=16)
        row=ttk.Frame(box); row.grid(row=2,column=0,columnspan=6,sticky="ew",padx=6,pady=(8,2))
        ttk.Label(row,text="Or load the measurements from a stored aircraft:",style="Sub.TLabel",background=self.C["panel"]).pack(side="left",padx=(0,8))
        self.pred_source=tk.StringVar()
        self.pred_source_cb=ttk.Combobox(row,textvariable=self.pred_source,values=[],state="readonly",width=34)
        self.pred_source_cb.pack(side="left"); ttk.Button(row,text="Load",command=self.load_pred_features,style="Secondary.TButton").pack(side="left",padx=6)

        bar=ttk.Frame(p); bar.pack(fill="x",pady=10)
        ttk.Button(bar,text="Predict Parameters",style="Primary.TButton",command=self.do_predict).pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Send to Flight Controller →",style="Secondary.TButton",command=self.prediction_to_fc).pack(side="left",padx=4)
        ttk.Button(bar,text="Export .param",style="Secondary.TButton",command=self.export_prediction).pack(side="left",padx=4)
        self.include_sim=tk.BooleanVar(value=False)
        ttk.Checkbutton(bar,text="Include SITL runs in training",variable=self.include_sim).pack(side="left",padx=16)

        self.pred_verdict=tk.Label(p,text="",font=("Segoe UI Semibold",10),bg=self.C["panel"],
                                   fg=self.C["muted"],anchor="w",padx=14,pady=9)
        self.pred_verdict.pack(fill="x",pady=(0,8))

        outwrap=ttk.Frame(p); outwrap.pack(fill="both",expand=True)
        self.pred_out=tk.Text(outwrap,height=14,wrap="word",font=("Consolas",10),bg=self.C["panel"],fg=self.C["text"],insertbackground=self.C["text"],padx=14,pady=12)
        pred_sb=ttk.Scrollbar(outwrap,orient="vertical",command=self.pred_out.yview)
        self.pred_out.configure(yscrollcommand=pred_sb.set)
        self.pred_out.pack(side="left",fill="both",expand=True); pred_sb.pack(side="right",fill="y")
        self.pred_out.tag_configure("warn",foreground=self.C["amber"])
        self.pred_out.tag_configure("bad",foreground="#ff8795")
        self.pred_out.tag_configure("good",foreground=self.C["green"])
        self.pred_out.insert("end","Enter the target aircraft's measurements and select Predict Parameters.\n\nThe model proposes; safety bounds decide what may leave this application; you confirm before anything is written to an aircraft.\n")

    def refresh_predict(self):
        if not hasattr(self,"census_label"): return
        with db() as c: cen=bridge.census(c)
        need=max(0, ml_model.MIN_TRAINING_ROWS-cen["trainable"])
        self.census_label.configure(text=(
            f"{cen['trainable']} aircraft usable for training   •   "
            f"{cen['aircraft_total']} in repository   •   "
            f"{cen['aircraft_with_params']} with parameter snapshots   •   "
            f"{cen['sim_runs']} completed SITL runs"
            + (f"\nNeed {need} more usable configuration(s) before prediction is possible "
               f"(minimum {ml_model.MIN_TRAINING_ROWS}). Case retrieval on the CBR tab works from a single stored case."
               if need else "\nEnough data to train. Treat predictions as an interpolation between similar builds, not a validated tune.")))
        lines=[f"Not usable — {s['name']}: {s['reason']}" for s in cen["skipped"][:6]]
        if len(cen["skipped"])>6: lines.append(f"… and {len(cen['skipped'])-6} more")
        self.census_detail.configure(text="\n".join(lines))
        with db() as c:
            names=[(r["id"],r["name"]) for r in c.execute("SELECT id,name FROM aircraft ORDER BY name")]
        self._pred_sources={f"#{i} • {n}":i for i,n in names}
        self.pred_source_cb.configure(values=list(self._pred_sources))

    def load_pred_features(self):
        key=self.pred_source.get()
        if not key or key not in getattr(self,"_pred_sources",{}):
            messagebox.showinfo("Select an aircraft","Choose a stored aircraft to copy measurements from."); return
        with db() as c: feats,prov=bridge.extract_features(c,self._pred_sources[key])
        for f in bridge.FEATURE_FIELDS:
            self.pred_vars[f].set(f"{feats[f]:g}" if f in feats else "")
        self.pred_out.delete("1.0","end")
        self.pred_out.insert("end",f"Loaded measurements from {key}\n\nWHERE EACH VALUE CAME FROM\n")
        for line in prov:
            self.pred_out.insert("end","  "+line+"\n","warn" if "INFERRED" in line else "")
        miss=bridge.missing_features(feats)
        if miss:
            self.pred_out.insert("end","\nStill missing: "+", ".join(bridge.FEATURE_LABELS[m] for m in miss)+"\nFill these in by hand, or edit the aircraft to record them.\n","bad")

    def _collect_pred_features(self):
        feats={}
        for f in bridge.FEATURE_FIELDS:
            raw=self.pred_vars[f].get().strip()
            if not raw: continue
            try: feats[f]=float(raw)
            except ValueError:
                messagebox.showwarning("Not a number",f"{bridge.FEATURE_LABELS[f]}: {raw!r} is not a number."); return None
        return feats

    def do_predict(self):
        feats=self._collect_pred_features()
        if feats is None: return
        missing=bridge.missing_features(feats)
        if missing:
            messagebox.showwarning("Missing measurements","Enter a value for:\n\n  "+"\n  ".join(bridge.FEATURE_LABELS[m] for m in missing)); return
        with db() as c: result=ml_model.recommend(c,feats,include_sim=self.include_sim.get())
        self.last_prediction=result if result.get("ok") else None
        t=self.pred_out; t.delete("1.0","end")
        if not result.get("ok"):
            self._set_verdict("PREDICTION NOT POSSIBLE", self.C["red"])
            t.insert("end","PREDICTION NOT POSSIBLE\n\n","bad"); t.insert("end",result["error"]+"\n"); return

        t.insert("end",f"Trained on {result['n_training']} configuration(s) "
                       f"({', '.join(result['training_sources'])})\n\n")
        if not result["in_training_range"]:
            t.insert("end","OUTSIDE TRAINED RANGE\n","bad")
            for w in result["range_warnings"]: t.insert("end","  ! "+w+"\n","warn")
            t.insert("end","\n")
        t.insert("end","PREDICTED PARAMETERS\n")
        t.insert("end",f"  {'parameter':<18}{'value':>14}{'tree spread':>16}   note\n")
        for name,value in result["params"].items():
            spread=result["spread"].get(name)
            derived=" derived from "+bridge.DERIVED_PARAMS[name][0] if name in bridge.DERIVED_PARAMS else ""
            sp=f"±{spread:.4g}" if spread else ""
            t.insert("end",f"  {name:<18}{value:>14.6g}{sp:>16}  {derived}\n")
        if result["clamp_notes"]:
            t.insert("end","\nPULLED INTO RANGE\n","warn")
            for n in result["clamp_notes"]: t.insert("end","  "+n+"\n","warn")
        v=result["validation"]
        t.insert("end",f"\nSAFETY CHECK: {'all parameters within approved range' if v['all_valid'] else 'BLOCKED'}\n",
                 "good" if v["all_valid"] else "bad")
        for b in v["blocked"]: t.insert("end","  • "+b["message"]+"\n","bad")
        if result["oob"]:
            t.insert("end","\nOUT-OF-BAG ERROR (each aircraft predicted only by trees that never saw it)\n")
            for k,val in result["oob"].items(): t.insert("end",f"  {k:<18}{val:>8.1%} mean relative error\n")
        t.insert("end","\nFEATURE IMPORTANCE\n")
        for k,val in result["importance"]: t.insert("end",f"  {bridge.FEATURE_LABELS.get(k,k):<20}{val:>7.0%}\n")
        t.insert("end","\nThis is an interpolation between stored builds, not a validated tune. "
                       "Bench test, then simulate, then fly it progressively.\n","warn")

        # The headline verdict, always visible without scrolling.
        if not v["all_valid"]:
            self._set_verdict(f"BLOCKED — {len(v['blocked'])} parameter(s) outside the approved range. Nothing can be written.", self.C["red"])
        elif not result["in_training_range"]:
            self._set_verdict(f"{len(result['params'])} parameters within approved range, but the target is OUTSIDE the trained range — see the warnings below.", self.C["amber"])
        else:
            worst=max(((k,sv/abs(result["params"][k])) for k,sv in result["spread"].items()
                       if result["params"].get(k)), key=lambda kv: kv[1], default=None)
            extra=f"  Widest tree disagreement: {worst[0]} ±{worst[1]:.0%}." if worst else ""
            self._set_verdict(f"{len(result['params'])} parameters within approved range, target inside the trained range.{extra}", self.C["green"])

    def _set_verdict(self, text, color):
        self.pred_verdict.configure(text="  "+text, fg=color)

    def export_prediction(self):
        if not self.last_prediction:
            messagebox.showinfo("Nothing to export","Run a prediction first."); return
        path=filedialog.asksaveasfilename(title="Save predicted parameters",defaultextension=".param",initialfile="ATLAS_PREDICTED_UNVALIDATED.param",filetypes=[("ArduPilot parameters","*.param")])
        if not path: return
        r=self.last_prediction; out=Path(path)
        out.write_text(f"# ATLAS v{APP_VERSION} PREDICTED TUNE - NOT FLIGHT VALIDATED\n"
                       f"# Trained on {r['n_training']} configuration(s): {', '.join(r['training_sources'])}\n"
                       f"# Within trained range: {r['in_training_range']}\n"
                       +"".join(f"{k},{v:.8g}\n" for k,v in r["params"].items()),encoding="utf-8")
        out.with_suffix(".review.json").write_text(json.dumps({
            "candidate_status":"UNTESTED","origin":"multi-output random forest",
            "n_training":r["n_training"],"training_sources":r["training_sources"],
            "in_training_range":r["in_training_range"],"range_warnings":r["range_warnings"],
            "clamped":r["clamp_notes"],"tree_spread":r["spread"],
            "out_of_bag_relative_error":r["oob"],
            "feature_importance":dict(r["importance"]),
            "parameters":r["params"]},indent=2),encoding="utf-8")
        self.status.set(f"Exported predicted tune • {out.name}")
        messagebox.showinfo("Predicted tune exported",f"{out}\n{out.with_suffix('.review.json')}\n\nNOT flight validated.")

    def prediction_to_fc(self):
        if not self.last_prediction:
            messagebox.showinfo("Nothing to send","Run a prediction first."); return
        self.fc_pending=dict(self.last_prediction["params"])
        self.fc_origin=f"random forest, {self.last_prediction['n_training']} training configs"
        self.tabs.select(TAB_FC); self.refresh_fc_pending()
        self.status.set("Prediction staged for the flight controller — review and confirm there.")

    # =====================================================================
    # FLIGHT CONTROLLER — the only tab that can change an aircraft
    # =====================================================================
    def build_fc(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Flight Controller  ")
        ttk.Label(p,text="HARDWARE LINK",style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(p,text="Apply Parameters",style="Big.TLabel").pack(anchor="w",pady=(3,2))
        ttk.Label(p,text="The only screen in ATLAS that writes to an aircraft. Every value is re-checked against the approved range here, whatever produced it, and nothing is written while the aircraft is armed.",style="Sub.TLabel",wraplength=1200,justify="left").pack(anchor="w",pady=(0,12))

        conn=ttk.LabelFrame(p,text="CONNECTION",style="Card.TLabelframe"); conn.pack(fill="x")
        self.fc_conn=tk.StringVar(value=fc.DEFAULT_CONNECTIONS[0][1])
        self.fc_preset=tk.StringVar(value=fc.DEFAULT_CONNECTIONS[0][0])
        row=ttk.Frame(conn); row.grid(row=0,column=0,columnspan=4,sticky="ew",padx=6,pady=4)
        ttk.Label(row,text="Preset",style="Sub.TLabel",background=self.C["panel"]).pack(side="left",padx=(0,6))
        presets=[n for n,_ in fc.DEFAULT_CONNECTIONS]
        cb=ttk.Combobox(row,textvariable=self.fc_preset,values=presets,state="readonly",width=26); cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>",lambda e:self.fc_conn.set(dict(fc.DEFAULT_CONNECTIONS)[self.fc_preset.get()]))
        ttk.Label(row,text="   Connection string",style="Sub.TLabel",background=self.C["panel"]).pack(side="left",padx=(12,6))
        ttk.Entry(row,textvariable=self.fc_conn,width=32).pack(side="left")
        ttk.Button(row,text="Connect",style="Primary.TButton",command=self.fc_connect).pack(side="left",padx=10)
        ttk.Button(row,text="Disconnect",command=self.fc_disconnect,style="Secondary.TButton").pack(side="left")
        ttk.Button(row,text="Test Connection…",command=self.fc_test_link,style="Secondary.TButton").pack(side="left",padx=10)
        self.fc_status=ttk.Label(conn,text="Not connected.",style="Sub.TLabel",background=self.C["panel"],justify="left",wraplength=1180)
        self.fc_status.grid(row=1,column=0,columnspan=4,sticky="w",padx=6,pady=(6,2))

        stage=ttk.LabelFrame(p,text="STAGED PARAMETERS",style="Card.TLabelframe"); stage.pack(fill="x",pady=(10,0))
        bar=ttk.Frame(stage); bar.grid(row=0,column=0,sticky="ew",padx=6,pady=4)
        ttk.Button(bar,text="Load from .param file",command=self.fc_load_file,style="Secondary.TButton").pack(side="left",padx=(0,6))
        ttk.Button(bar,text="Read current from aircraft",command=self.fc_read_current,style="Secondary.TButton").pack(side="left",padx=6)
        ttk.Button(bar,text="Clear",command=lambda:(setattr(self,"fc_pending",{}),self.refresh_fc_pending()),style="Secondary.TButton").pack(side="left",padx=6)
        ttk.Button(bar,text="APPLY TO AIRCRAFT",style="Primary.TButton",command=self.fc_apply).pack(side="right",padx=6)
        self.fc_pending={}; self.fc_origin="manual"

        cols=("param","value","verdict","range")
        fcwrap=ttk.Frame(p)
        self.fc_tree=ttk.Treeview(fcwrap,columns=cols,show="headings",height=12)
        for k,title,w in [("param","Parameter",230),("value","Staged value",150),("verdict","Safety verdict",260),("range","Approved range",220)]:
            self.fc_tree.heading(k,text=title); self.fc_tree.column(k,width=w,anchor="w")
        self.fc_note=ttk.Label(p,text="Nothing staged. Predict a tune, load a .param file, or read the current values from a connected aircraft.",style="Sub.TLabel",wraplength=1200,justify="left")
        self.fc_note.pack(side="bottom",anchor="w",pady=(8,0))
        fc_sb=ttk.Scrollbar(fcwrap,orient="vertical",command=self.fc_tree.yview)
        self.fc_tree.configure(yscrollcommand=fc_sb.set)
        fcwrap.pack(fill="both",expand=True,pady=(12,0))
        self.fc_tree.pack(side="left",fill="both",expand=True); fc_sb.pack(side="right",fill="y")
        self.fc_tree.tag_configure("ok",foreground=self.C["green"])
        self.fc_tree.tag_configure("bad",foreground="#ff8795")

    def refresh_fc_pending(self):
        if not hasattr(self,"fc_tree"): return
        for i in self.fc_tree.get_children(): self.fc_tree.delete(i)
        if not self.fc_pending:
            self.fc_note.configure(text="Nothing staged. Predict a tune, load a .param file, or read the current values from a connected aircraft."); return
        report=safety.check_param_set(self.fc_pending)
        VERDICT={safety.OK:"will be written",
                 safety.OUT_OF_RANGE:"BLOCKED — outside approved range",
                 safety.NOT_A_NUMBER:"BLOCKED — not a number"}
        for r in report["results"]:
            rng=f"[{r['low']:g}, {r['high']:g}]" if r["low"] is not None else "—"
            val=f"{r['value']:.6g}" if isinstance(r["value"],float) else str(r["value"])
            verdict=VERDICT.get(r["verdict"])
            if verdict is None:     # not_tunable — the reason is the useful part
                verdict="BLOCKED — "+r["message"].split(": ",1)[-1]
            self.fc_tree.insert("","end",values=(r["name"],val,verdict,rng),
                                tags=("ok" if r["verdict"]==safety.OK else "bad",))
        self.fc_note.configure(text=(
            f"{len(report['writable'])} parameter(s) will be written; {len(report['blocked'])} blocked. "
            f"Origin: {self.fc_origin}. "
            + ("Blocked parameters are never sent — not clamped, not silently skipped."
               if report["blocked"] else "All staged parameters are within the approved range.")))

    def fc_connect(self):
        ok,msg=fc.available()
        if not ok:
            messagebox.showerror("pymavlink not installed",msg); self.fc_status.configure(text=msg); return
        conn_str=self.fc_conn.get().strip()
        if self.vehicle is not None: self.fc_disconnect()
        vehicle=fc.Vehicle(conn_str)
        def work(say): return vehicle.connect(progress=say)
        def done(info):
            self.vehicle=vehicle
            frame=f"FRAME_CLASS={info.get('FRAME_CLASS')}, FRAME_TYPE={info.get('FRAME_TYPE')}"
            self.fc_status.configure(text=(
                f"Connected to {conn_str}  •  system {info['system_id']} / component {info['component_id']}  •  "
                f"{'ARMED' if info['armed'] else 'disarmed'}  •  {frame}"))
            self.status.set(f"Connected • {conn_str}")
        def failed(exc):
            self.vehicle=None; self.fc_status.configure(text=str(exc))
            messagebox.showerror("Could not connect",str(exc))
        self.run_bg(work,on_done=done,on_error=failed,label=f"Connecting to {conn_str}")

    def fc_test_link(self):
        """Run the link diagnostic and show the report in a window.

        A UDP timeout looks identical whether the simulator is off, aimed at
        the wrong address, firewalled, or on a network that will not route —
        so the report separates those rather than repeating "no heartbeat".
        """
        conn=self.fc_conn.get().strip()
        port=14550
        if ":" in conn:
            tail=conn.rsplit(":",1)[-1]
            if tail.isdigit(): port=int(tail)
        is_udp=conn.startswith(("udp:","udpin:","udpout:"))

        win=tk.Toplevel(self); win.title("Connection Diagnostic"); win.geometry("820x600")
        win.transient(self); win.configure(bg=self.C["bg"])
        body=ttk.Frame(win,padding=18); body.pack(fill="both",expand=True)
        ttk.Label(body,text="Connection Diagnostic",style="Big.TLabel").pack(anchor="w")
        ttk.Label(body,text=f"Testing {conn}",style="Sub.TLabel").pack(anchor="w",pady=(2,12))
        out=tk.Text(body,wrap="word",font=("Consolas",9),bg=self.C["panel"],
                    fg=self.C["text"],insertbackground=self.C["text"],padx=12,pady=10)
        sb=ttk.Scrollbar(body,orient="vertical",command=out.yview)
        out.configure(yscrollcommand=sb.set)
        out.pack(side="left",fill="both",expand=True); sb.pack(side="right",fill="y")
        out.tag_configure("good",foreground=self.C["green"])
        out.tag_configure("bad",foreground="#ff8795")

        if not is_udp:
            out.insert("end","This diagnostic only tests network (UDP) links.\n\n")
            out.insert("end",f"'{conn}' is a serial port or TCP address. For a serial port, check "
                             "the cable, the port name, the baud rate, and that no other program "
                             "(Mission Planner, QGroundControl) is holding it.\n")
            out.configure(state="disabled"); return

        ok,msg=fc.available()
        if not ok:
            out.insert("end",msg+"\n","bad"); out.configure(state="disabled"); return

        out.insert("end","Running …\n\n")
        win.update_idletasks()
        def work(say):
            return check_link.diagnose(port=port,
                                       conn=conn if conn.startswith("udpin:") else None,
                                       seconds=6.0)
        def done(report):
            out.delete("1.0","end")
            for line in report["lines"]:
                out.insert("end",line+"\n")
            out.insert("end","\n"+"="*62+"\n")
            if report["ok"]:
                out.insert("end","LINK IS WORKING. Use this connection string:\n","good")
                out.insert("end",f"  {report['connection']}\n","good")
                if report["connection"]!=conn:
                    out.insert("end","\n(that differs from what is in the field — update it)\n","bad")
            else:
                out.insert("end","NO LINK. Work through these in order:\n","bad")
                for i,step in enumerate(check_link.FAILURE_STEPS,1):
                    out.insert("end",f"  {i}. {step}\n")
            out.configure(state="disabled")
        def failed(exc):
            out.insert("end",f"\nDiagnostic failed: {exc}\n","bad")
            out.configure(state="disabled")
        self.run_bg(work,on_done=done,on_error=failed,label="Testing the link")

    def fc_disconnect(self):
        if self.vehicle is not None:
            try: self.vehicle.close()
            except Exception: pass
        self.vehicle=None; self.fc_status.configure(text="Not connected."); self.status.set("Disconnected")

    def fc_load_file(self):
        path=filedialog.askopenfilename(title="Load parameter file",filetypes=[("ArduPilot parameter files","*.param *.parm *.txt"),("All files","*.*")])
        if not path: return
        rows=parse_param(path)
        acro={k:v for k,v in rows if k in safety.ACRO_PARAM_RANGES}
        if not acro:
            messagebox.showinfo("No tunable parameters",
                f"{Path(path).name} contains {len(rows)} parameters but none that this tool tunes.\n\n"
                "ATLAS only writes Acro rate-controller and stick-feel parameters. "
                "Everything else belongs in Mission Planner."); return
        self.fc_pending={k:float(v) for k,v in acro.items()}
        self.fc_origin=f"file: {Path(path).name}"
        self.refresh_fc_pending()
        self.status.set(f"Staged {len(self.fc_pending)} of {len(rows)} parameters from {Path(path).name}")

    def fc_read_current(self):
        if self.vehicle is None:
            messagebox.showinfo("Not connected","Connect to a flight controller first."); return
        def work(say): return self.vehicle.read_acro_params(progress=say)
        def done(values):
            got={k:v for k,v in values.items() if v is not None}
            self.fc_pending=got; self.fc_origin="read from aircraft"; self.refresh_fc_pending()
            missed=[k for k,v in values.items() if v is None]
            self.status.set(f"Read {len(got)} parameters"+(f"; {len(missed)} did not respond" if missed else ""))
        self.run_bg(work,on_done=done,label="Reading current parameters")

    def fc_apply(self):
        if self.vehicle is None:
            messagebox.showinfo("Not connected","Connect to a flight controller first."); return
        if not self.fc_pending:
            messagebox.showinfo("Nothing staged","Stage some parameters first."); return

        def confirm(report):
            # Runs on the worker thread, so it cannot touch widgets. The
            # operator already confirmed in the dialog below; this is the
            # interface Vehicle.apply_params demands and it must stay truthful.
            return bool(getattr(self,"_fc_confirmed",False))

        report=safety.check_param_set(self.fc_pending)
        if not report["writable"]:
            messagebox.showerror("Nothing can be written","Every staged parameter was blocked by the approved range."); return
        lines="\n".join(f"   {k} = {v:g}" for k,v in sorted(report["writable"].items()))
        blocked=("\n\nBLOCKED (will NOT be sent):\n"+"\n".join("   "+b["message"] for b in report["blocked"])) if report["blocked"] else ""
        if not messagebox.askyesno("Write parameters to the aircraft?",
                f"About to write {len(report['writable'])} parameter(s) to {self.fc_conn.get()}:\n\n{lines}{blocked}\n\n"
                "Propellers should be OFF. The aircraft must be disarmed.\n\nProceed?"):
            return
        self._fc_confirmed=True
        aircraft_id=self._selected_repo_aircraft()
        def work(say): return self.vehicle.apply_params(dict(self.fc_pending),confirm=confirm,progress=say)
        def done(result):
            self._fc_confirmed=False
            if not result["applied"]:
                messagebox.showwarning("Not applied",result["reason"] or "No parameters were written."); return
            with db() as c:
                bridge.record_application(c,aircraft_id,self.fc_conn.get(),
                                          result["validation"]["writable"],result["confirmed"],
                                          self.fc_origin,"")
            msg=f"Wrote and confirmed {len(result['confirmed'])} parameter(s)."
            if result.get("failures"): msg+=f"\n\n{len(result['failures'])} failed:\n"+"\n".join("  "+f for f in result["failures"])
            msg+="\n\nRecorded in the audit trail. Bench test before flying."
            messagebox.showinfo("Parameters applied",msg); self.status.set(msg.splitlines()[0]); self.refresh_all()
        def failed(exc):
            self._fc_confirmed=False; messagebox.showerror("Apply failed",str(exc))
        self.run_bg(work,on_done=done,on_error=failed,label="Writing parameters")

    def open_log_folder(self):
        """Show the folder holding the raw simulated flight logs."""
        import subprocess, sys as _sys
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        try:
            if _sys.platform.startswith("win"): os.startfile(str(LOG_DIR))  # noqa: S606
            elif _sys.platform == "darwin": subprocess.Popen(["open", str(LOG_DIR)])
            else: subprocess.Popen(["xdg-open", str(LOG_DIR)])
        except Exception as exc:
            messagebox.showinfo("Flight log folder",
                f"{LOG_DIR}\n\n(could not open a file browser here: {exc})")

    def _selected_repo_aircraft(self):
        sel=self.tree.selection() if hasattr(self,"tree") else ()
        return int(sel[0]) if sel else None

    # =====================================================================
    # SIMULATION — iterate candidate tunes in SITL
    # =====================================================================
    def build_sim(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Simulation  ")
        ttk.Label(p,text="SITL EVALUATION",style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(p,text="Iterate Candidate Tunes",style="Big.TLabel").pack(anchor="w",pady=(3,2))
        ttk.Label(p,text="Flies a scripted Acro sequence in ArduPilot SITL once per candidate and scores how closely the measured body rates followed the commanded rates. Requires a running simulator and pymavlink.",style="Sub.TLabel",wraplength=1200,justify="left").pack(anchor="w",pady=(0,12))

        setup=ttk.LabelFrame(p,text="SWEEP",style="Card.TLabelframe"); setup.pack(fill="x")
        self.sim_param=tk.StringVar(value="ATC_RAT_RLL_P"); self.sim_low=tk.StringVar(value="0.03")
        self.sim_high=tk.StringVar(value="0.12"); self.sim_steps=tk.StringVar(value="5")
        self.sim_conn=tk.StringVar(value="udp:127.0.0.1:14550")
        self.field(setup,"Parameter to sweep",self.sim_param,0,0,values=bridge.TARGET_PARAMS,state="readonly")
        self.field(setup,"From",self.sim_low,0,1); self.field(setup,"To",self.sim_high,0,2); self.field(setup,"Steps",self.sim_steps,0,3)
        self.field(setup,"Simulator connection",self.sim_conn,2,0,width=36)
        ttk.Label(setup,text="Everything not being swept is held at the baseline tune.\nSITL on THIS computer:      connection  udp:127.0.0.1:14550      and   sim_vehicle.py -v ArduCopter -f quad --out=udp:127.0.0.1:14550\nSITL on ANOTHER computer:   connection  udpin:0.0.0.0:14550      and   sim_vehicle.py -v ArduCopter -f quad --out=udp:THIS_PC_IP:14550\n127.0.0.1 is loopback and can never reach another machine. Run check_link.py to test the link on its own.",style="Sub.TLabel",background=self.C["panel"],wraplength=1150,justify="left").grid(row=4,column=0,columnspan=4,sticky="w",padx=6,pady=(6,2))

        logs=ttk.LabelFrame(p,text="FLIGHT LOGS",style="Card.TLabelframe"); logs.pack(fill="x",pady=(10,0))
        self.sim_tlog=tk.BooleanVar(value=True); self.sim_bin=tk.BooleanVar(value=False)
        self.sim_file_evidence=tk.BooleanVar(value=True)
        lrow=ttk.Frame(logs); lrow.grid(row=0,column=0,sticky="ew",padx=6,pady=2)
        ttk.Checkbutton(lrow,text="Record .tlog per run",variable=self.sim_tlog).pack(side="left",padx=(0,18))
        ttk.Checkbutton(lrow,text="Download .bin from autopilot (slow)",variable=self.sim_bin).pack(side="left",padx=(0,18))
        ttk.Checkbutton(lrow,text="File each run as flight evidence",variable=self.sim_file_evidence).pack(side="left")
        ttk.Label(logs,text=".tlog is what this program received — cheap, works over any link. .bin is the autopilot's own high-rate log, which Mission Planner reads; it is fetched over MAVLink, which is slow but is the only route when SITL runs on another computer. Filing needs an aircraft selected on the Repository tab; evidence is marked source=sitl and never changes an airframe's validation status.",style="Sub.TLabel",background=self.C["panel"],wraplength=1180,justify="left").grid(row=1,column=0,sticky="w",padx=6,pady=(4,2))

        bar=ttk.Frame(p); bar.pack(fill="x",pady=10)
        ttk.Button(bar,text="Run Sweep",style="Primary.TButton",command=self.run_sweep).pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Refresh Results",command=self.refresh_sim,style="Secondary.TButton").pack(side="left",padx=4)
        ttk.Button(bar,text="Open Log Folder",command=self.open_log_folder,style="Secondary.TButton").pack(side="left",padx=4)
        self.sim_evidence_label=ttk.Label(bar,text="",style="Sub.TLabel")
        self.sim_evidence_label.pack(side="right",padx=6)

        cols=("score","type","group","params","logs","note")
        simwrap=ttk.Frame(p)
        self.sim_tree=ttk.Treeview(simwrap,columns=cols,show="headings",height=14)
        # Bench scores and Acro-sweep scores are different measures; "Type" keeps them apart.
        for k,title,w in [("score","Score",80),("type","Type",120),("group","Run group",150),("params","Parameters",380),("logs","Logs filed",100),("note","Status",170)]:
            self.sim_tree.heading(k,text=title); self.sim_tree.column(k,width=w,anchor="w")
        self.sim_note=ttk.Label(p,text="Scores compare runs within one sweep on one simulated airframe. SITL has no frame resonance or motor lag unless you configure SIM_* to match, so a winning tune here is a starting point, not a validated one.",style="Sub.TLabel",wraplength=1200,justify="left")
        self.sim_note.pack(side="bottom",anchor="w",pady=(8,0))
        sim_sb=ttk.Scrollbar(simwrap,orient="vertical",command=self.sim_tree.yview)
        self.sim_tree.configure(yscrollcommand=sim_sb.set)
        simwrap.pack(fill="both",expand=True)
        self.sim_tree.pack(side="left",fill="both",expand=True); sim_sb.pack(side="right",fill="y")

    def refresh_sim(self):
        if not hasattr(self,"sim_tree"): return
        for i in self.sim_tree.get_children(): self.sim_tree.delete(i)
        with db() as c:
            rows=c.execute("SELECT * FROM sim_runs ORDER BY outcome_score DESC, id DESC LIMIT 200").fetchall()
            summary=sim_logs.sim_flight_summary(c)
            # One pass for the log counts rather than a query per row: the
            # run group is recorded in the evidence notes when a run is filed.
            counts={}
            for ft in c.execute("SELECT notes FROM flight_tests WHERE source=?",(bridge.SOURCE_SIM,)):
                for line in (ft["notes"] or "").splitlines():
                    if line.startswith("Run group: "):
                        g=line.split(": ",1)[1].strip()
                        counts[g]=counts.get(g,0)+1
        for r in rows:
            try: params=json.loads(r["params_json"])
            except (TypeError,ValueError): params={}
            shown=", ".join(f"{k}={v:g}" for k,v in sorted(params.items()) if k in bridge.TARGET_PARAMS)
            nlogs=counts.get(r["run_group"],0)
            kind="Standard bench" if r["run_group"].startswith(BENCH_GROUP_PREFIX) else "Acro sweep"
            self.sim_tree.insert("","end",values=(
                f"{r['outcome_score']:.4f}" if r["outcome_score"] is not None else "—",
                kind, r["run_group"], shown[:130], (f"{nlogs} file(s)" if nlogs else "—"),
                (r["abort_reason"] or "completed")[:50]))
        if hasattr(self,"sim_evidence_label"):
            self.sim_evidence_label.configure(text=(
                f"{summary['sim_flight_tests']} simulated log(s) filed  •  "
                f"mean score {summary['mean_sim_score']}" if summary["mean_sim_score"] is not None
                else f"{summary['sim_flight_tests']} simulated log(s) filed"))

    def run_sweep(self):
        # Validate the sweep BEFORE checking for pymavlink: an out-of-range
        # sweep is the more useful thing to hear about, and the check works
        # with no flight stack installed at all.
        try:
            import sim_iterate
        except ImportError as exc:
            messagebox.showerror("Simulation module missing",f"sim_iterate.py could not be imported:\n\n{exc}"); return
        try:
            low,high,steps=float(self.sim_low.get()),float(self.sim_high.get()),int(self.sim_steps.get())
        except ValueError:
            messagebox.showwarning("Invalid sweep","From, To and Steps must be numbers."); return
        param=self.sim_param.get()
        try:
            candidates=sim_iterate.sweep(param,low,high,steps)
        except ValueError as exc:
            messagebox.showerror("Invalid sweep",str(exc)); return
        bad=[]
        for cand in candidates:
            rep=safety.check_param_set(cand)
            bad+= [b["message"] for b in rep["blocked"]]
        if bad:
            messagebox.showerror("Sweep leaves the approved range",
                f"This sweep would test values outside the approved range:\n\n"
                +"\n".join("  • "+b for b in dict.fromkeys(bad))); return
        ok,msg=fc.available()
        if not ok: messagebox.showerror("pymavlink not installed",msg); return
        aircraft_id=self._selected_repo_aircraft()
        if not messagebox.askyesno("Run sweep?",
                f"Fly {len(candidates)} simulated run(s), sweeping {param} from {low:g} to {high:g}.\n\n"
                f"Roughly {len(candidates)} × 90 s. ATLAS will be busy until it finishes.\n\n"
                f"Results will be linked to aircraft id {aircraft_id if aircraft_id else '(none selected in Repository)'}.\n\nProceed?"):
            return
        if self.sim_file_evidence.get() and aircraft_id is None:
            if not messagebox.askyesno("No aircraft selected",
                    "Filing runs as flight evidence needs an aircraft selected on the "
                    "Repository tab.\n\nRun anyway? The flights will still be scored and "
                    "logged, but nothing will be filed as evidence."):
                return
        cfg=sim_iterate.HarnessConfig(connection=self.sim_conn.get().strip(),db_path=str(DB_PATH))
        def work(say):
            return sim_iterate.iterate(
                candidates,cfg,aircraft_id=aircraft_id,progress=say,
                record_tlog=self.sim_tlog.get(),
                download_bin=self.sim_bin.get(),
                log_dir=str(LOG_DIR),
                files_dir=str(FILES_DIR) if self.sim_file_evidence.get() else None)
        def done(ranked):
            self.refresh_sim(); self.refresh_predict()
            best=ranked[0] if ranked else None
            messagebox.showinfo("Sweep complete",
                (f"Best score {best['score']['outcome_score']:.4f} with "
                 + ", ".join(f"{k}={v:g}" for k,v in sorted(best["params"].items()) if k==param)
                 if best else "No runs completed."))
        self.run_bg(work,on_done=done,label=f"Running {len(candidates)} simulated flights")

    # =====================================================================
    # STANDARD BENCH — sitl_bench/'s standardized flight, filed into ATLAS
    # =====================================================================
    # The bench runs as its own process (it fans out to worker processes and
    # SITL instances, which do not belong inside the GUI process). Its output
    # streams into the log pane; its results land in this database through
    # its own ATLAS link, so the Simulation and Predict tabs pick them up.
    def build_bench(self):
        c=self.C
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Standard Bench  ")
        ttk.Label(p,text="SITL STANDARD BENCH",style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(p,text="Compare Tunes on a Standard Flight",style="Big.TLabel").pack(anchor="w",pady=(3,2))
        ttk.Label(p,text="Flies one identical test flight (hover, position step, box, climb/descend, yaw steps, speed run, attitude steps) in Mission Planner's SITL for every configuration and repeat, scores each run 0–1 from its onboard log, and files it as simulated evidence. The simulated airframe and baseline tune come from the selected aircraft, and every configuration passes the safety check before anything flies.",style="Sub.TLabel",wraplength=1200,justify="left").pack(anchor="w",pady=(0,12))

        top=ttk.Frame(p); top.pack(fill="x")
        setup=ttk.LabelFrame(top,text="AIRCRAFT & RUN",style="Card.TLabelframe"); setup.pack(side="left",fill="both",expand=True,padx=(0,10))
        self.bench_aircraft=tk.StringVar(); self.bench_weight=tk.StringVar()
        self.bench_repeats=tk.StringVar(value="2"); self.bench_workers=tk.StringVar(value="2"); self.bench_speedup=tk.StringVar(value="10")
        self.bench_save_weight=tk.BooleanVar(value=False); self.bench_file=tk.BooleanVar(value=True)
        self.bench_aircraft_box=self.field(setup,"Aircraft",self.bench_aircraft,0,0,width=40,values=[],state="readonly",required=True)
        self.bench_aircraft_box.bind("<<ComboboxSelected>>",lambda e:self._bench_load_weight())
        self.field(setup,"All-up weight (g)",self.bench_weight,0,1,width=14,required=True)
        self.field(setup,"Repeats per config",self.bench_repeats,2,0,width=10)
        self.field(setup,"Parallel runs",self.bench_workers,2,1,width=10)
        self.field(setup,"Sim speedup",self.bench_speedup,2,2,width=10)
        opts=ttk.Frame(setup); opts.grid(row=4,column=0,columnspan=3,sticky="w",padx=6,pady=(4,0))
        ttk.Checkbutton(opts,text="Save this weight to the aircraft record",variable=self.bench_save_weight).pack(side="left",padx=(0,18))
        ttk.Checkbutton(opts,text="File runs into ATLAS as simulated evidence",variable=self.bench_file).pack(side="left")
        ttk.Label(setup,text="Training needs a recorded weight; without one, runs are flown and filed but skipped by the model. Keep parallel runs at 2–3: runs whose log loses data are marked aborted, not scored.",style="Sub.TLabel",background=c["panel"],wraplength=560,justify="left").grid(row=5,column=0,columnspan=3,sticky="w",padx=6,pady=(6,0))

        cfgf=ttk.LabelFrame(top,text="CONFIGURATIONS",style="Card.TLabelframe"); cfgf.pack(side="left",fill="both",expand=True)
        self.bench_cfg_text=tk.Text(cfgf,height=6,width=56,bg=c["panel2"],fg=c["text"],insertbackground=c["text"],relief="flat",font=("Consolas",10),padx=8,pady=6)
        self.bench_cfg_text.pack(fill="both",expand=True)
        self.bench_cfg_text.insert("1.0","hardware_tune:\nrate_P_up: ATC_RAT_RLL_P=0.045, ATC_RAT_PIT_P=0.045\n")
        ttk.Label(cfgf,text="One per line:  name: PARAM=value, PARAM=value.  The first line is the baseline; leave it empty to fly the aircraft's own tune. SIM_* and LOG_* configure the simulator, not the aircraft (e.g. SIM_WIND_SPD=8).",style="Sub.TLabel",background=c["panel"],wraplength=520,justify="left").pack(anchor="w",pady=(6,0))

        bar=ttk.Frame(p); bar.pack(fill="x",pady=10)
        self.bench_run_btn=ttk.Button(bar,text="Run Bench",style="Primary.TButton",command=self.run_bench)
        self.bench_run_btn.pack(side="left",padx=(0,8))
        ttk.Button(bar,text="Stop",style="Danger.TButton",command=self.stop_bench).pack(side="left",padx=4)
        ttk.Button(bar,text="Open Results Folder",style="Secondary.TButton",command=self.open_bench_folder).pack(side="left",padx=4)
        self.bench_summary=ttk.Label(bar,text="",style="Sub.TLabel"); self.bench_summary.pack(side="right",padx=6)

        low=ttk.Frame(p); low.pack(fill="both",expand=True)
        cols=("config","repeat","score","status")
        self.bench_tree=ttk.Treeview(low,columns=cols,show="headings",height=10)
        for k,title,w in [("config","Configuration",170),("repeat","Repeat",70),("score","Score",80),("status","Status",260)]:
            self.bench_tree.heading(k,text=title); self.bench_tree.column(k,width=w,anchor="w")
        self.bench_tree.pack(side="left",fill="both",expand=True)
        self.bench_log=tk.Text(low,height=10,width=70,bg=c["panel"],fg=c["muted"],relief="flat",font=("Consolas",9),padx=8,pady=6,state="disabled",wrap="none")
        self.bench_log.pack(side="left",fill="both",expand=True,padx=(10,0))

        self.bench_proc=None; self.bench_outdir=None; self.bench_lines=queue.Queue()
        self.after(200,self._drain_bench_log)

    def refresh_bench(self):
        if not hasattr(self,"bench_aircraft_box"): return
        with db() as c:
            rows=c.execute("SELECT id,name FROM aircraft ORDER BY name").fetchall()
        values=[f"{r['id']} — {r['name']}" for r in rows]
        self.bench_aircraft_box.configure(values=values)
        if self.bench_aircraft.get() not in values:
            repo=self._selected_repo_aircraft()
            pick=next((v for v in values if repo and v.startswith(f"{repo} — ")),values[0] if values else "")
            self.bench_aircraft.set(pick); self._bench_load_weight()

    def _bench_aircraft_id(self):
        v=self.bench_aircraft.get()
        return int(v.split(" — ",1)[0]) if v else None

    def _bench_load_weight(self):
        aid=self._bench_aircraft_id()
        if aid is None: return
        with db() as c:
            row=c.execute("SELECT weight_g FROM aircraft WHERE id=?",(aid,)).fetchone()
        self.bench_weight.set(f"{row['weight_g']:g}" if row and row["weight_g"] else "")

    def _bench_parse_configs(self):
        """Ordered {name: {PARAM: value}} from the configuration box."""
        import re
        configs={}
        for n,line in enumerate(self.bench_cfg_text.get("1.0","end").splitlines(),1):
            line=line.strip()
            if not line or line.startswith("#"): continue
            name,sep,rest=line.partition(":")
            name=name.strip()
            if not sep or not re.fullmatch(r"[A-Za-z0-9_.-]+",name):
                raise ValueError(f"line {n}: start with a name made of letters, digits, _ . - followed by ':'")
            if name in configs: raise ValueError(f"line {n}: '{name}' is used twice")
            params={}
            for item in filter(None,(s.strip() for s in rest.split(","))):
                k,eq,v=item.partition("=")
                k=k.strip().upper()
                if not eq or not re.fullmatch(r"[A-Z0-9_]+",k): raise ValueError(f"line {n}: '{item}' is not PARAM=value")
                try: params[k]=float(v)
                except ValueError: raise ValueError(f"line {n}: {k} value '{v.strip()}' is not a number")
            configs[name]=params
        if not configs: raise ValueError("enter at least one configuration")
        return configs

    def run_bench(self):
        if self.bench_proc is not None and self.bench_proc.poll() is None:
            messagebox.showinfo("Bench running","A bench run is already in progress."); return
        runner=BENCH_DIR/"run_bench.py"
        if not runner.exists():
            messagebox.showerror("Bench not found",f"Expected the SITL bench at:\n{BENCH_DIR}"); return
        aid=self._bench_aircraft_id()
        if aid is None: messagebox.showwarning("No aircraft","Choose an aircraft to simulate."); return
        try:
            configs=self._bench_parse_configs()
            repeats,workers,speedup=int(self.bench_repeats.get()),int(self.bench_workers.get()),float(self.bench_speedup.get())
            if not (1<=repeats<=20 and 1<=workers<=6 and 1<=speedup<=20): raise ValueError("repeats 1–20, parallel runs 1–6, speedup 1–20")
            weight=float(self.bench_weight.get()) if self.bench_weight.get().strip() else None
        except ValueError as exc:
            messagebox.showwarning("Invalid bench setup",str(exc)); return
        if weight is None or weight<=0:
            messagebox.showwarning("Weight needed","Enter the aircraft's all-up weight in grams. The simulated airframe is built from it."); return

        # Same check the bench applies, run here first so nothing starts if a config is blocked.
        if str(BENCH_DIR) not in sys.path: sys.path.insert(0,str(BENCH_DIR))
        try:
            import atlas_link
            with db() as c:
                recorded=c.execute("SELECT weight_g FROM aircraft WHERE id=?",(aid,)).fetchone()["weight_g"]
                aircraft=atlas_link.load_aircraft(c,bridge,aid,None if recorded else weight)
            blocked=[]
            for name,overrides in configs.items():
                _,_,report=atlas_link.plan_config(bridge,safety,aircraft,overrides)
                blocked+=[f"{name}: {b['name']} = {b.get('value')} ({b.get('message') or b.get('verdict')})" for b in report["blocked"]]
        except Exception as exc:                       # noqa: BLE001
            messagebox.showerror("Cannot prepare bench",str(exc)); return
        if blocked:
            messagebox.showerror("Configuration leaves the approved range",
                "These configurations would fly values outside the approved range:\n\n"+"\n".join("  • "+b for b in blocked)); return

        nruns=len(configs)*repeats
        if not messagebox.askyesno("Run standard bench?",
                f"Fly {nruns} simulated run(s): {len(configs)} configuration(s) × {repeats} repeat(s) on {aircraft['name']}.\n\n"
                f"Roughly {max(1,round(nruns*30/workers/60))} min at {speedup:g}× speed, {workers} at a time.\n\n"
                +("Each run will be filed into ATLAS as simulated evidence." if self.bench_file.get() else "Nothing will be filed into ATLAS.")
                +"\n\nProceed?"):
            return
        if self.bench_save_weight.get() and recorded!=weight:
            with db() as c: c.execute("UPDATE aircraft SET weight_g=? WHERE id=?",(weight,aid))
            recorded=weight

        # The bench resolves paths relative to its config file, so write absolute ones.
        with open(BENCH_DIR/"bench.json") as f: cfg=json.load(f)
        for key in ("sitl_binary","copter_defaults","model_file","base_params"):
            cfg[key]=str((BENCH_DIR/os.path.expanduser(cfg[key])).resolve())
        cfg.update(repeats=repeats,workers=workers,speedup=speedup,configs=configs)
        cfg_path=LOG_DIR/f"bench_config_{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
        with open(cfg_path,"w") as f: json.dump(cfg,f,indent=2)
        cmd=[sys.executable,"-u",str(runner),str(cfg_path),"--atlas-db",str(DB_PATH),"--aircraft",str(aid),"--atlas-dir",str(APP_DIR)]
        if not recorded: cmd+=["--weight-g",f"{weight:g}"]
        if not self.bench_file.get(): cmd.append("--no-file")

        for i in self.bench_tree.get_children(): self.bench_tree.delete(i)
        self.bench_log.configure(state="normal"); self.bench_log.delete("1.0","end"); self.bench_log.configure(state="disabled")
        self.bench_summary.configure(text=""); self.bench_outdir=None
        self.bench_run_btn.state(["disabled"])
        env=dict(os.environ,PYTHONIOENCODING="utf-8",PYTHONUNBUFFERED="1")
        flags=subprocess.CREATE_NO_WINDOW if sys.platform.startswith("win") else 0

        def work(say):
            self.bench_proc=subprocess.Popen(cmd,cwd=str(BENCH_DIR),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                             text=True,encoding="utf-8",errors="replace",env=env,creationflags=flags)
            outdir=None
            for line in self.bench_proc.stdout:
                line=line.rstrip()
                self.bench_lines.put(line)
                if " -> " in line and "runs" in line: outdir=line.rsplit(" -> ",1)[1].strip()
                if line.startswith("["): say(f"Bench: {line}")
            rc=self.bench_proc.wait()
            return {"rc":rc,"outdir":outdir}
        def done(res):
            self.bench_run_btn.state(["!disabled"]); self.bench_outdir=res["outdir"]
            self._bench_show_results(res["outdir"])
            self.refresh_all()
            if res["rc"]!=0:
                messagebox.showwarning("Bench stopped",f"The bench exited with code {res['rc']}. See the log pane for details.")
        def failed(exc):
            self.bench_run_btn.state(["!disabled"])
            messagebox.showerror("Bench failed",str(exc))
        self.run_bg(work,on_done=done,on_error=failed,label=f"Running standard bench: {nruns} simulated flights")

    def _bench_show_results(self,outdir):
        if not outdir or not os.path.isdir(outdir): return
        by_config={}
        for run_json in sorted(Path(outdir).glob("*/run.json")):
            with open(run_json) as f: r=json.load(f)
            s=r.get("score") or {}
            score=s.get("outcome_score")
            ok=score is not None and not s.get("aborted")
            status=("completed" if ok else (s.get("abort_reason") or r.get("error") or "failed"))
            self.bench_tree.insert("","end",values=(r["config"],r["repeat"]+1,f"{score:.4f}" if ok else "—",status[:80]))
            if ok: by_config.setdefault(r["config"],[]).append(score)
        filed=Path(outdir)/"atlas_filing.json"
        nfiled=len(json.load(open(filed))) if filed.exists() else 0
        means="   ".join(f"{k}: {sum(v)/len(v):.3f}" for k,v in by_config.items())
        self.bench_summary.configure(text=(f"Mean score   {means}" if means else "No scored runs")+(f"   •   {nfiled} filed into ATLAS" if nfiled else ""))

    def stop_bench(self):
        proc=self.bench_proc
        if proc is None or proc.poll() is not None: return
        if not messagebox.askyesno("Stop bench?","Stop the bench now? Runs in progress are discarded and nothing from this batch is filed."): return
        if sys.platform.startswith("win"):
            # the bench's worker processes and SITL instances go with it
            subprocess.run(["taskkill","/PID",str(proc.pid),"/T","/F"],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
        else:
            proc.kill()

    def open_bench_folder(self):
        folder=Path(self.bench_outdir) if self.bench_outdir else BENCH_DIR/"runs"
        folder.mkdir(parents=True,exist_ok=True)
        try: os.startfile(str(folder))  # noqa: S606
        except Exception as exc: messagebox.showinfo("Bench results",f"{folder}\n\n(could not open a file browser: {exc})")

    def _drain_bench_log(self):
        lines=[]
        try:
            while True: lines.append(self.bench_lines.get_nowait())
        except queue.Empty:
            pass
        if lines:
            self.bench_log.configure(state="normal")
            self.bench_log.insert("end","\n".join(lines)+"\n"); self.bench_log.see("end")
            self.bench_log.configure(state="disabled")
        self.after(200,self._drain_bench_log)

    def build_browser(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Repository  "); top=ttk.Frame(p); top.pack(fill="x"); ttk.Label(top,text="Configuration Repository",style="Big.TLabel").pack(side="left"); ttk.Button(top,text="Refresh",style="Secondary.TButton",command=self.refresh_browser).pack(side="right"); ttk.Button(top,text="Edit Validation",style="Secondary.TButton",command=self.edit_validation).pack(side="right",padx=8); ttk.Button(top,text="Add Flight Evidence",style="Secondary.TButton",command=self.add_flight_evidence).pack(side="right",padx=8); ttk.Button(top,text="Add Parameter File",style="Primary.TButton",command=self.add_parameter_file).pack(side="right",padx=8)
        cols=("name","frame","validation","components","params","evidence"); self.tree=ttk.Treeview(p,columns=cols,show="headings",height=18); labels={"name":"Aircraft","frame":"Frame","validation":"Validation Status","components":"Components","params":"Parameters","evidence":"Flights (hw / sim)"}; widths={"name":220,"frame":150,"validation":175,"components":100,"params":100,"evidence":135}
        for c in cols:self.tree.heading(c,text=labels[c]); self.tree.column(c,width=widths[c],anchor="w")
        self.tree.pack(fill="both",expand=True,pady=14); self.tree.bind("<Double-1>",self.details); ttk.Label(p,text="Select a configuration to update validation. Double-click for full details.",style="Sub.TLabel").pack(anchor="w")
    def refresh_browser(self):
        for i in self.tree.get_children(): self.tree.delete(i)
        with db() as c:
            for a in c.execute("SELECT * FROM aircraft ORDER BY id DESC"):
                nc=c.execute("SELECT COUNT(*) FROM components WHERE aircraft_id=?",(a["id"],)).fetchone()[0]
                # Parameters from HARDWARE snapshots only — a simulated run files
                # the tune it flew, and lumping those in would read as more real
                # configuration evidence than the aircraft actually has.
                np=c.execute("SELECT COUNT(*) FROM parameters p JOIN parameter_files f ON p.parameter_file_id=f.id WHERE f.aircraft_id=? AND f.source='hardware'",(a["id"],)).fetchone()[0]
                nhw=c.execute("SELECT COUNT(*) FROM flight_tests WHERE aircraft_id=? AND source='hardware'",(a["id"],)).fetchone()[0]
                nsim=c.execute("SELECT COUNT(*) FROM flight_tests WHERE aircraft_id=? AND source<>'hardware'",(a["id"],)).fetchone()[0]
                self.tree.insert("","end",iid=str(a["id"]),values=(a["name"],f"{a['frame_type'] or ''} {a['frame_size'] or ''}".strip(),a["validation_status"],nc,np,f"{nhw} / {nsim}"))
    def add_parameter_file(self):
        sel=self.tree.selection()
        if not sel: messagebox.showinfo("Select a configuration","Choose an aircraft first."); return
        aid=int(sel[0]); path=filedialog.askopenfilename(title="Add ArduPilot parameter file",filetypes=[("ArduPilot parameter files","*.param *.parm *.txt"),("All files","*.*")])
        if not path:return
        ver=simpledialog.askstring("ArduPilot version","ArduPilot version (optional):",parent=self)
        try:
            src=Path(path); dest=FILES_DIR/f"{aid}_{datetime.now().strftime('%Y%m%d%H%M%S')}_{src.name}"; shutil.copy2(src,dest)
            with db() as c:
                pf=c.execute("INSERT INTO parameter_files(aircraft_id,original_name,stored_path,ardupilot_version,imported_at) VALUES(?,?,?,?,?)",(aid,src.name,str(dest),ver or "",datetime.now().isoformat(timespec="seconds"))).lastrowid
                rows=parse_param(src); c.executemany("INSERT INTO parameters(parameter_file_id,name,value) VALUES(?,?,?)",[(pf,a,b) for a,b in rows])
            self.status.set(f"Added parameter file • {src.name}"); self.refresh_all(); messagebox.showinfo("Parameter file saved",f"{src.name} is now stored with this configuration.")
        except Exception as e: messagebox.showerror("Could not save parameter file",str(e))

    def add_flight_evidence(self):
        sel=self.tree.selection()
        if not sel: messagebox.showinfo("Select a configuration","Choose an aircraft first."); return
        aid=int(sel[0])
        with db() as c: pfs=c.execute("SELECT id,original_name,imported_at FROM parameter_files WHERE aircraft_id=? ORDER BY id DESC",(aid,)).fetchall()
        if not pfs: messagebox.showwarning("No parameter file","Add the parameter file used for this flight before adding the flight log."); return
        win=tk.Toplevel(self); win.title("Add Flight Evidence"); win.geometry("560x440"); win.transient(self)
        body=ttk.Frame(win,padding=20); body.pack(fill="both",expand=True); ttk.Label(body,text="Link Flight Log to Parameter File",style="Big.TLabel").pack(anchor="w"); ttk.Label(body,text="Choose the exact .param snapshot that was loaded for this flight.",style="Sub.TLabel").pack(anchor="w",pady=(2,16))
        labels=[f"#{r['id']} • {r['original_name']} • {r['imported_at'] or ''}" for r in pfs]; by_label={label:r['id'] for label,r in zip(labels,pfs)}; pv=tk.StringVar(value=labels[0]); logv=tk.StringVar(); datev=tk.StringVar(); resultv=tk.StringVar(value="Not evaluated")
        ttk.Label(body,text="Parameter file used").pack(anchor="w"); ttk.Combobox(body,textvariable=pv,values=labels,state="readonly").pack(fill="x",pady=(4,10)); ttk.Label(body,text="Flight log").pack(anchor="w"); row=ttk.Frame(body); row.pack(fill="x",pady=(4,10)); ttk.Entry(row,textvariable=logv).pack(side="left",fill="x",expand=True); ttk.Button(row,text="Browse…",command=lambda: logv.set(filedialog.askopenfilename(title="Select flight log",filetypes=[("Flight logs","*.bin *.log *.tlog"),("All files","*.*")]) or logv.get())).pack(side="left",padx=(6,0)); ttk.Label(body,text="Test date").pack(anchor="w"); ttk.Entry(body,textvariable=datev).pack(fill="x",pady=(4,10)); ttk.Label(body,text="Result").pack(anchor="w"); ttk.Combobox(body,textvariable=resultv,values=["Not evaluated","Pass","Fail","Mixed"],state="readonly").pack(fill="x",pady=(4,10)); ttk.Label(body,text="Flight notes").pack(anchor="w"); note=tk.Text(body,height=4,wrap="word"); note.pack(fill="both",expand=True,pady=(4,10))
        def save():
            if not logv.get().strip(): messagebox.showwarning("Missing flight log","Choose a flight log file.",parent=win); return
            try:
                src=Path(logv.get()); dest=FILES_DIR/f"{aid}_{datetime.now().strftime('%Y%m%d%H%M%S')}_{src.name}"; shutil.copy2(src,dest); pfid=by_label[pv.get()]
                with db() as c: c.execute("INSERT INTO flight_tests(aircraft_id,parameter_file_id,log_name,stored_path,test_date,result,notes) VALUES(?,?,?,?,?,?,?)",(aid,pfid,src.name,str(dest),datev.get().strip(),resultv.get(),note.get("1.0","end").strip()))
                self.status.set(f"Flight evidence linked • {src.name}"); self.refresh_all(); win.destroy(); messagebox.showinfo("Flight evidence saved",f"{src.name} is linked to {pv.get().split(' • ')[1]}.")
            except Exception as e: messagebox.showerror("Could not save flight evidence",str(e),parent=win)
        ttk.Button(body,text="Save Linked Flight",style="Primary.TButton",command=save).pack(anchor="e")

    def edit_validation(self):
        sel=self.tree.selection()
        if not sel: messagebox.showinfo("Select a configuration","Choose an aircraft first."); return
        aid=int(sel[0]); win=tk.Toplevel(self); win.title("Update Validation"); win.geometry("460x310"); win.transient(self)
        with db() as c: a=c.execute("SELECT * FROM aircraft WHERE id=?",(aid,)).fetchone()
        body=ttk.Frame(win,padding=20); body.pack(fill="both",expand=True); ttk.Label(body,text=a["name"],style="Big.TLabel").pack(anchor="w"); ttk.Label(body,text="Update the status only when supported by configuration evidence.",style="Sub.TLabel").pack(anchor="w",pady=(2,16)); v=tk.StringVar(value=a["validation_status"]); ttk.Label(body,text="Validation status").pack(anchor="w"); ttk.Combobox(body,textvariable=v,values=VALIDATION,state="readonly").pack(fill="x",pady=(4,12)); ttk.Label(body,text="Validation notes").pack(anchor="w"); note=tk.Text(body,height=5,wrap="word"); note.pack(fill="both",expand=True,pady=(4,12)); note.insert("1.0",a["notes"] or "")
        def save():
            with db() as c: c.execute("UPDATE aircraft SET validation_status=?, notes=? WHERE id=?",(v.get(),note.get("1.0","end").strip(),aid))
            self.status.set(f"Updated {a['name']} → {v.get()}"); self.refresh_all(); win.destroy()
        ttk.Button(body,text="Save Validation",style="Primary.TButton",command=save).pack(anchor="e")
    def details(self,event=None):
        sel=self.tree.selection()
        if not sel:return
        aid=int(sel[0])
        with db() as c:
            a=c.execute("SELECT * FROM aircraft WHERE id=?",(aid,)).fetchone()
            comps=c.execute("SELECT * FROM components WHERE aircraft_id=?",(aid,)).fetchall()
            pfs=c.execute("SELECT * FROM parameter_files WHERE aircraft_id=? ORDER BY id",(aid,)).fetchall()
            tests=c.execute("SELECT ft.*, pf.original_name AS parameter_name FROM flight_tests ft LEFT JOIN parameter_files pf ON ft.parameter_file_id=pf.id WHERE ft.aircraft_id=? ORDER BY ft.id DESC",(aid,)).fetchall()
            outcomes=c.execute("SELECT * FROM acro_outcomes WHERE aircraft_id=? ORDER BY id DESC",(aid,)).fetchall()
            feats,prov=bridge.extract_features(c,aid)
            label,why=bridge.outcome_label(c,aid)
        win=tk.Toplevel(self); win.title(a["name"]); win.geometry("820x660"); win.configure(bg=self.C["bg"])
        wrap=ttk.Frame(win,padding=12); wrap.pack(fill="both",expand=True)
        t=tk.Text(wrap,wrap="word",font=("Consolas",10),bg=self.C["panel"],fg=self.C["text"],
                  insertbackground=self.C["text"],padx=12,pady=10)
        sb=ttk.Scrollbar(wrap,orient="vertical",command=t.yview); t.configure(yscrollcommand=sb.set)
        t.pack(side="left",fill="both",expand=True); sb.pack(side="right",fill="y")
        t.tag_configure("sim",foreground=self.C["amber"])
        t.tag_configure("hw",foreground=self.C["green"])
        t.tag_configure("muted",foreground=self.C["muted"])

        t.insert("end",f"{a['name']}\nVALIDATION: {a['validation_status']}\n"
                       f"Frame: {a['frame_type']} {a['frame_size'] or ''}\n")
        t.insert("end",f"TRAINING LABEL: {'excluded' if label is None else f'{label:.2f}'} — {why}\n","muted")

        t.insert("end","\nHARDWARE\n")
        for x in comps:
            t.insert("end",f"• {x['category']}: {x['manufacturer'] or ''} {x['model'] or ''} {x['specs'] or ''}\n")

        t.insert("end","\nMEASUREMENTS FOR PREDICTION\n")
        miss=bridge.missing_features(feats)
        for f in bridge.FEATURE_FIELDS:
            v=feats.get(f)
            t.insert("end",f"• {bridge.FEATURE_LABELS[f]:<20} "
                           f"{(f'{v:g}' if v is not None else 'not recorded')}\n",
                     "" if v is not None else "sim")
        if miss:
            t.insert("end",f"  missing {len(miss)} — this aircraft cannot train or "
                           "receive a prediction until they are filled in\n","sim")
        for line in prov:
            if "INFERRED" in line:
                t.insert("end",f"  note: {line}\n","sim")

        # Source is shown on every row. The database has always tagged simulated
        # evidence; until this view printed the tag, a SITL flight looked
        # identical on screen to one that actually flew.
        t.insert("end","\nPARAMETER FILES\n")
        for x in pfs:
            src=(x["source"] if "source" in x.keys() else "hardware") or "hardware"
            tag="sim" if src!="hardware" else "hw"
            t.insert("end",f"• [{src.upper():<8}] {x['original_name']}  "
                           f"ArduPilot {x['ardupilot_version'] or 'version not recorded'}\n",tag)
        if not pfs: t.insert("end","  (none)\n","muted")

        t.insert("end","\nFLIGHT TESTS\n")
        for x in tests:
            src=(x["source"] if "source" in x.keys() else "hardware") or "hardware"
            tag="sim" if src!="hardware" else "hw"
            t.insert("end",f"• [{src.upper():<8}] {x['test_date'] or 'date not recorded'} • "
                           f"{x['result'] or ''} • {x['log_name'] or ''}\n",tag)
            t.insert("end",f"    PARAM: {x['parameter_name'] or 'legacy / not linked'}\n","muted")
        if not tests: t.insert("end","  (none)\n","muted")

        if outcomes:
            t.insert("end","\nRECORDED SCORES\n")
            for o in outcomes:
                tag="sim" if (o["source"] or "hardware")!="hardware" else "hw"
                t.insert("end",f"• [{(o['source'] or 'hardware').upper():<8}] "
                               f"score {o['outcome_score']:.4f}"
                               + (f"  vibration {o['vibration_max']:g} m/s^2" if o["vibration_max"] is not None else "")
                               + f"  ({o['recorded_at'] or ''})\n",tag)
            t.insert("end","  SITL scores never change validation status and are excluded "
                           "from hardware training.\n","muted")

        if a["notes"]:t.insert("end",f"\nVALIDATION NOTES\n{a['notes']}\n")
        t.configure(state="disabled")

    def build_hardware(self):
        p=ttk.Frame(self.tabs,padding=18); self.tabs.add(p,text="  Hardware Catalog  "); ttk.Label(p,text="Hardware Catalog",style="Big.TLabel").pack(anchor="w"); ttk.Label(p,text="Add, edit, deactivate, or remove reusable hardware choices without changing the Python source.",style="Sub.TLabel").pack(anchor="w",pady=(2,14))
        form=ttk.LabelFrame(p,text="Catalog item",style="Card.TLabelframe"); form.pack(fill="x"); self.hw_cat=tk.StringVar(value="Flight Controller"); self.hw_make=tk.StringVar(); self.hw_model=tk.StringVar(); self.hw_specs=tk.StringVar(); self.field(form,"Category",self.hw_cat,0,0,values=CATEGORIES,state="readonly"); self.field(form,"Manufacturer",self.hw_make,0,1); self.field(form,"Model",self.hw_model,0,2); self.field(form,"Specs / notes",self.hw_specs,2,0)
        btn=ttk.Frame(form); btn.grid(row=3,column=1,columnspan=2,sticky="e",padx=6,pady=8); ttk.Button(btn,text="Clear",command=self.clear_hw_form).pack(side="left",padx=4); ttk.Button(btn,text="Save Hardware",style="Primary.TButton",command=self.save_hardware).pack(side="left",padx=4)
        cols=("category","manufacturer","model","specs","active"); hwwrap=ttk.Frame(p); self.hw_tree=ttk.Treeview(hwwrap,columns=cols,show="headings",height=15); labs={"category":"Category","manufacturer":"Manufacturer","model":"Model","specs":"Specs / Notes","active":"Active"}; widths={"category":150,"manufacturer":150,"model":300,"specs":300,"active":60}
        for c in cols:self.hw_tree.heading(c,text=labs[c]); self.hw_tree.column(c,width=widths[c],anchor="w")
        # The action row is packed first so Tk gives it space before the tree
        # takes the remainder — otherwise it falls off the bottom of the window.
        actions=ttk.Frame(p); actions.pack(side="bottom",fill="x",pady=(8,0))
        hw_sb=ttk.Scrollbar(hwwrap,orient="vertical",command=self.hw_tree.yview); self.hw_tree.configure(yscrollcommand=hw_sb.set)
        hwwrap.pack(fill="both",expand=True,pady=(14,0))
        self.hw_tree.pack(side="left",fill="both",expand=True); hw_sb.pack(side="right",fill="y")
        self.hw_tree.bind("<<TreeviewSelect>>",self.load_hw_selection)
        ttk.Button(actions,text="Toggle Active",command=self.toggle_hardware).pack(side="left"); ttk.Button(actions,text="Delete",command=self.delete_hardware).pack(side="left",padx=8); ttk.Button(actions,text="Add Missing From Aircraft",style="Secondary.TButton",command=self.catalog_from_aircraft).pack(side="left",padx=8); ttk.Label(actions,text="Inactive items stay in the catalog history but disappear from aircraft dropdowns.",style="Sub.TLabel").pack(side="right")
        self.edit_hw_id=None
    def catalog_from_aircraft(self):
        """Catalog every part recorded against an aircraft but not yet listed.

        Parts fitted to an aircraft are not automatically in the reusable
        catalog, so without this they have to be retyped by hand on the next
        build that uses them.
        """
        import import_build
        with db() as c:
            missing=c.execute(
                "SELECT DISTINCT cm.category, cm.manufacturer, cm.model FROM components cm "
                "WHERE TRIM(COALESCE(cm.model,'')) <> '' AND NOT EXISTS ("
                " SELECT 1 FROM hardware_catalog h WHERE h.category=cm.category "
                " AND h.manufacturer=COALESCE(cm.manufacturer,'') AND h.model=cm.model)"
            ).fetchall()
        if not missing:
            messagebox.showinfo("Nothing to add",
                "Every part fitted to an aircraft is already in the catalog."); return
        listing="\n".join(f"   [{r['category']}]  {r['manufacturer'] or '(no make)'}  {r['model']}"
                           for r in missing[:18])
        if len(missing)>18: listing+=f"\n   … and {len(missing)-18} more"
        if not messagebox.askyesno("Add to hardware catalog",
                f"{len(missing)} part(s) are fitted to an aircraft but not in the "
                f"reusable catalog:\n\n{listing}\n\nAdd them? They will then appear "
                "in the Add Aircraft dropdowns."):
            return
        with db() as c:
            added,already=import_build.catalog_sync_from_aircraft(c)
        self.status.set(f"Hardware catalog: {added} added"+(f", {already} already present" if already else ""))
        self.refresh_all()
        messagebox.showinfo("Hardware catalog updated",
            f"Added {added} part(s)."+(f"\n{already} were already present." if already else ""))

    def refresh_hardware(self):
        for i in self.hw_tree.get_children(): self.hw_tree.delete(i)
        with db() as c:
            for r in c.execute("SELECT * FROM hardware_catalog ORDER BY category,manufacturer,model"): self.hw_tree.insert("","end",iid=str(r["id"]),values=(r["category"],r["manufacturer"],r["model"],r["specs"] or "","Yes" if r["active"] else "No"))
    def clear_hw_form(self): self.edit_hw_id=None; self.hw_make.set(""); self.hw_model.set(""); self.hw_specs.set("")
    def load_hw_selection(self,event=None):
        sel=self.hw_tree.selection()
        if not sel:return
        self.edit_hw_id=int(sel[0]); vals=self.hw_tree.item(sel[0],"values"); self.hw_cat.set(vals[0]); self.hw_make.set(vals[1]); self.hw_model.set(vals[2]); self.hw_specs.set(vals[3])
    def save_hardware(self):
        cat,make,model,specs=self.hw_cat.get().strip(),self.hw_make.get().strip(),self.hw_model.get().strip(),self.hw_specs.get().strip()
        if not cat or not make or not model: messagebox.showwarning("Missing hardware information","Category, manufacturer, and model are required."); return
        try:
            with db() as c:
                if self.edit_hw_id: c.execute("UPDATE hardware_catalog SET category=?,manufacturer=?,model=?,specs=? WHERE id=?",(cat,make,model,specs,self.edit_hw_id))
                else: c.execute("INSERT INTO hardware_catalog(category,manufacturer,model,specs,created_at) VALUES(?,?,?,?,?)",(cat,make,model,specs,datetime.now().isoformat(timespec="seconds")))
            self.status.set(f"Hardware catalog updated • {make} {model}"); self.clear_hw_form(); self.refresh_all()
        except sqlite3.IntegrityError: messagebox.showerror("Duplicate hardware","That category / manufacturer / model already exists.")
    def toggle_hardware(self):
        sel=self.hw_tree.selection()
        if not sel:return
        hid=int(sel[0])
        with db() as c: c.execute("UPDATE hardware_catalog SET active=CASE active WHEN 1 THEN 0 ELSE 1 END WHERE id=?",(hid,))
        self.refresh_all()
    def delete_hardware(self):
        sel=self.hw_tree.selection()
        if not sel:return
        if not messagebox.askyesno("Delete hardware","Remove this item from the reusable hardware catalog? Existing aircraft records will not be changed."):return
        with db() as c:c.execute("DELETE FROM hardware_catalog WHERE id=?",(int(sel[0]),))
        self.clear_hw_form(); self.refresh_all()

    def refresh_all(self):
        self.refresh_browser(); self.refresh_hardware(); self.refresh_ingest_catalog()
        self.refresh_predict(); self.refresh_fc_pending(); self.refresh_sim(); self.refresh_bench()
        if hasattr(self,"amc"): self.amc.refresh()
        with db() as c: na=c.execute("SELECT COUNT(*) FROM aircraft").fetchone()[0]; nc=c.execute("SELECT COUNT(*) FROM components").fetchone()[0]; nh=c.execute("SELECT COUNT(*) FROM hardware_catalog WHERE active=1").fetchone()[0]; np=c.execute("SELECT COUNT(*) FROM parameters").fetchone()[0]; nt=bridge.census(c)["trainable"]
        self.stat_air.configure(text=str(na)); self.stat_comp.configure(text=str(nc)); self.stat_hw.configure(text=str(nh)); self.stat_param.configure(text=str(np))
        if hasattr(self,"stat_train"): self.stat_train.configure(text=str(nt))

if __name__=="__main__":
    init_db(); ATLAS().mainloop()
