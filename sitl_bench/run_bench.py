#!/usr/bin/env python3
"""Fly the standard profile for every config in bench.json and compare results.

Usage:
  python run_bench.py [bench.json]
  python run_bench.py [bench.json] --atlas-db ../atlas/atlas_repository.db --aircraft 2

Writes runs/<timestamp>/ with one folder per run (SITL output, .BIN log,
run.json), plus results.csv (every run) and summary.csv (mean/std per config
and % change vs the first config, which is the baseline).

With --atlas-db, the simulated airframe and baseline tune come from the ATLAS
aircraft, every config is checked by ATLAS's safety layer before it flies,
and each run is filed into ATLAS as simulated evidence (see atlas_link.py).
"""
import argparse
import csv
import hashlib
import json
import multiprocessing
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import analyze
import atlas_link
import flight_profile as fp
import sitl

HERE = os.path.dirname(os.path.abspath(__file__))


def load_config(path):
    with open(path) as f:
        cfg = json.load(f)
    base = os.path.dirname(os.path.abspath(path))
    resolve = lambda p: os.path.abspath(os.path.join(base, os.path.expanduser(p)))
    for key in ("sitl_binary", "copter_defaults", "model_file", "base_params"):
        cfg[key] = resolve(cfg[key])
    return cfg


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:12]


_slots = None


def _init_worker(slots):
    global _slots
    _slots = slots


def run_one(job):
    # Reuse a small pool of SITL instance numbers so ports stay low
    # (instance N uses 5760+10N, 5501+10N, ...; high N collides with other
    # software, e.g. Mission Planner listens on UDP 5601).
    instance = _slots.get()
    try:
        return _run_one(job, instance)
    finally:
        _slots.put(instance)


def _run_one(job, instance):
    cfg, name, repeat, rundir, params, check_keys = job
    os.makedirs(rundir, exist_ok=True)

    # Per-run parameters go in a parm file loaded at boot, so even
    # reboot-required parameters take effect.
    override_file = os.path.join(rundir, "overrides.parm")
    with open(override_file, "w") as f:
        for k, v in params.items():
            f.write(f"{k} {v}\n")
    defaults = [cfg["copter_defaults"], cfg["base_params"], override_file]

    record = {
        "config": name, "repeat": repeat, "overrides": cfg["configs"][name],
        "profile_version": fp.PROFILE_VERSION,
        "model_sha": sha256(cfg["model_file"]), "base_params_sha": sha256(cfg["base_params"]),
        "sitl_binary_sha": cfg["_binary_sha"],
    }
    started = time.time()
    proc = sitl.launch(cfg["sitl_binary"], cfg["frame"], cfg["model_file"],
                       defaults, instance, rundir, cfg["speedup"])
    v = None
    try:
        v = sitl.Vehicle(5760 + 10 * instance)
        readback = sitl.fly_standard(v, check_keys)
        mismatched = {k: (params[k], readback[k]) for k in check_keys
                      if readback[k] is None or abs(readback[k] - float(params[k])) > 1e-4}
        if mismatched:
            record["param_mismatch"] = mismatched
        time.sleep(1)  # let the log close after disarm
    except Exception as e:
        record["error"] = f"{type(e).__name__}: {e}"
    finally:
        if v:
            record["statustext"] = v.statustext
        sitl.stop(proc)

    log = sitl.newest_log(rundir)
    record["log"] = log
    if log and "error" not in record:
        try:
            record["metrics"] = analyze.analyze(log)
            record["firmware"] = record["metrics"].pop("firmware", None)
        except Exception as e:
            record["error"] = f"analysis failed: {type(e).__name__}: {e}"
    record["score"] = atlas_link.score_run(record)
    if "metrics" in record:
        record["metrics"]["outcome_score"] = record["score"]["outcome_score"]
    record["wall_s"] = round(time.time() - started, 1)

    with open(os.path.join(rundir, "run.json"), "w") as f:
        json.dump(record, f, indent=2)
    status = record.get("error", "ok")
    print(f"[{name} #{repeat}] {status} ({record['wall_s']} s)", flush=True)
    return record


def write_results(outdir, records, config_names):
    metric_keys = []
    for r in records:
        for k in r.get("metrics", {}):
            if k not in metric_keys and k != "segments_found":
                metric_keys.append(k)

    with open(os.path.join(outdir, "results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["config", "repeat", "error"] + metric_keys)
        for r in records:
            m = r.get("metrics", {})
            w.writerow([r["config"], r["repeat"], r.get("error", "")] + [m.get(k) for k in metric_keys])

    # summary: mean and std per config, % change of the mean vs baseline
    stats = {}
    for name in config_names:
        runs = [r["metrics"] for r in records if r["config"] == name and "metrics" in r]
        stats[name] = {}
        for k in metric_keys:
            vals = [m[k] for m in runs if isinstance(m.get(k), (int, float))]
            if vals:
                stats[name][k] = (statistics.mean(vals),
                                  statistics.stdev(vals) if len(vals) > 1 else 0.0, len(vals))
    baseline = config_names[0]
    with open(os.path.join(outdir, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric"] + [f"{n} {col}" for n in config_names for col in ("mean", "std", f"%vs_{baseline}")])
        for k in metric_keys:
            row = [k]
            base_mean = stats[baseline].get(k, (None,))[0]
            for n in config_names:
                if k in stats[n]:
                    mu, sd, _ = stats[n][k]
                    pct = 100.0 * (mu - base_mean) / abs(base_mean) if base_mean else None
                    row += [round(mu, 4), round(sd, 4), round(pct, 1) if pct is not None else ""]
                else:
                    row += ["", "", ""]
            w.writerow(row)
    return stats


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("config", nargs="?", default=os.path.join(HERE, "bench.json"))
    p.add_argument("--atlas-db", help="ATLAS repository database to take the aircraft from and file runs into")
    p.add_argument("--aircraft", type=int, help="ATLAS aircraft id")
    p.add_argument("--weight-g", type=float,
                   help="all-up weight to simulate if ATLAS has none recorded (not written to ATLAS)")
    p.add_argument("--atlas-dir", default=os.path.join(HERE, "..", "atlas"),
                   help="folder containing the ATLAS code (default: ../atlas)")
    p.add_argument("--no-file", action="store_true",
                   help="with --atlas-db: use the aircraft but don't file runs into ATLAS")
    args = p.parse_args()
    if bool(args.atlas_db) != (args.aircraft is not None):
        p.error("--atlas-db and --aircraft go together")
    return args


def main():
    args = parse_args()
    cfg = load_config(args.config)
    cfg["_binary_sha"] = sha256(cfg["sitl_binary"])   # identifies the ArduPilot build
    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(HERE, "runs", stamp)
    os.makedirs(outdir)
    names = list(cfg["configs"])

    # Per-config parameter sets: plain overrides, or the ATLAS aircraft's tune
    plans = {n: {"params": dict(cfg["configs"][n]), "check": list(cfg["configs"][n]),
                 "tune": None, "blocked": []} for n in names}
    atlas = None
    if args.atlas_db:
        bridge, safety, sim_logs = atlas_link.import_atlas(args.atlas_dir)
        conn = atlas_link.open_db(os.path.abspath(args.atlas_db), bridge)
        aircraft = atlas_link.load_aircraft(conn, bridge, args.aircraft, args.weight_g)
        model_path = os.path.join(outdir, f"aircraft{aircraft['id']}_model.json")
        atlas_link.build_model(cfg["model_file"], aircraft, model_path)
        cfg["model_file"] = model_path
        cfg["_atlas_aircraft"] = {k: aircraft[k] for k in
                                  ("id", "name", "features", "provenance", "parameter_file_id")}
        print(f"ATLAS aircraft #{aircraft['id']}: {aircraft['name']}")
        for line in aircraft["provenance"]:
            print(f"  {line}")
        for n in names:
            params, tune, report = atlas_link.plan_config(bridge, safety, aircraft, cfg["configs"][n])
            plans[n] = {"params": params, "check": sorted(set(tune) | set(cfg["configs"][n])),
                        "tune": tune, "blocked": report["blocked"]}
            if not report["all_valid"]:
                why = "; ".join(f"{b['name']}: {b.get('reason', b.get('verdict'))}" for b in report["blocked"])
                print(f"  BLOCKED by ATLAS safety, not flown: {n} ({why})")
        atlas = (conn, bridge, sim_logs, aircraft)

    with open(os.path.join(outdir, "bench.json"), "w") as f:
        json.dump(cfg, f, indent=2)   # exact config used, for the record

    jobs, records = [], []
    for name in names:
        plan = plans[name]
        for rep in range(cfg["repeats"]):
            idx = len(jobs) + len(records)
            if plan["blocked"]:
                records.append({"config": name, "repeat": rep, "blocked": plan["blocked"],
                                "error": "blocked by ATLAS safety check"})
                continue
            jobs.append((cfg, name, rep, os.path.join(outdir, f"{idx:03d}_{name}_r{rep}"),
                         plan["params"], plan["check"]))

    print(f"profile {fp.PROFILE_VERSION}: {len(jobs)} runs, {cfg['workers']} at a time -> {outdir}")
    with multiprocessing.Manager() as mgr:
        slots = mgr.Queue()
        for i in range(cfg["workers"]):
            slots.put(i)
        with ProcessPoolExecutor(cfg["workers"], initializer=_init_worker, initargs=(slots,)) as pool:
            records += list(pool.map(run_one, jobs))
    records.sort(key=lambda r: (names.index(r["config"]), r["repeat"]))

    stats = write_results(outdir, records, names)
    failed = [r for r in records if "error" in r]
    print(f"\n{len(records) - len(failed)}/{len(records)} runs OK. Results: {outdir}/summary.csv")
    for key in ["outcome_score"] + cfg.get("headline_metrics", []):
        line = "  ".join(f"{n}={stats[n][key][0]:.3f}±{stats[n][key][1]:.3f}"
                         for n in names if key in stats[n])
        print(f"  {key:28s} {line}")

    if atlas and not args.no_file:
        conn, bridge, sim_logs, aircraft = atlas
        for r in records:
            r["tune"] = plans[r["config"]]["tune"]
        files_dir = os.path.join(os.path.abspath(args.atlas_dir), "repository_files")
        filed = atlas_link.file_runs(conn, bridge, sim_logs, aircraft, records,
                                     files_dir, f"bench_{stamp}")
        conn.close()
        with open(os.path.join(outdir, "atlas_filing.json"), "w") as f:
            json.dump(filed, f, indent=2)
        usable = [e for e in filed if not e["aborted"] and e["outcome_score"] >= 0.7]
        print(f"\nFiled {len(filed)} runs into ATLAS as simulated evidence for "
              f"aircraft #{aircraft['id']} ({len(usable)} clear ATLAS's 0.7 training threshold).")
        if aircraft["weight_from_cli"]:
            print("  Note: weight came from --weight-g. Record it in ATLAS, or training will "
                  "skip this aircraft and its runs.")


if __name__ == "__main__":
    main()
