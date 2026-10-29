# 5" quad SITL bench

Flies an identical, standardized test flight in ArduPilot SITL for each
configuration, then scores every run from its onboard `.BIN` log.

Runs natively on Windows using Mission Planner's SITL binary
(`Documents\Mission Planner\sitl\ArduCopter.exe`) - no WSL needed.
Mission Planner downloads that binary the first time you use its Simulation tab.

## Run

```powershell
cd sitl_bench
python run_bench.py               # flies every config in bench.json
python analyze.py path\to\log.BIN # re-score one log without re-flying
```

Needs `pip install pymavlink`. One flight is ~3 min of sim time, ~28 s wall at speedup 10.

## Feeding ATLAS

```powershell
python run_bench.py --atlas-db ..\atlas\atlas_repository.db --aircraft 2
```

With `--atlas-db`, the bench works from a stored ATLAS aircraft instead of
`5inch.json`/defaults (`atlas_link.py`):

1. **Airframe**: the SITL model is scaled from `5inch.json` to the aircraft's
   weight, wheelbase, props, cells and motor count, with its learned hover
   throttle, thrust expo and battery capacity.
2. **Tune**: the aircraft's hardware snapshot (rate PIDs, filters, notches,
   mixer, Acro rates) is the baseline; each config in `bench.json` changes it.
3. **Safety**: every config's tune goes through ATLAS's `validate_for_output()`.
   A blocked config is never flown.
4. **Score**: each run gets a 0–1 `outcome_score` (`score_run()`, version
   `bench-score-v1`). Runs whose log lost data are aborted, not scored.
5. **Filing**: each run goes into ATLAS through `record_sim_run()` and
   `ingest_sim_flight()`, tagged `source='sitl'`, with its `.BIN` copied to
   `atlas/repository_files/`. The aircraft's validation status and hardware
   snapshot are never changed.

ATLAS trains on these only with **Include SITL runs in training** ticked, only
runs scoring ≥ 0.7, and only if the aircraft has every feature recorded,
including all-up weight. `--weight-g` lets you simulate without a recorded
weight, but training will skip the aircraft until it is stored in ATLAS.
`--no-file` uses the aircraft without writing anything to ATLAS.

**Model calibration:** `moment_inertia` in `5inch.json` was set so that the
Lucid H7's FULLY_VALIDATED hardware tune flies cleanly (1 deg/s rate error).
The first estimate (5x lower) made that tune oscillate. Because the model was
tuned to agree with that tune, the simulation is biased toward it; measured
inertia would remove the bias.

## Reading results

`runs/<timestamp>/summary.csv` has mean, std and % change vs the first
config (the baseline). Treat differences smaller than ~2x the std as noise.
Every run's `run.json` records the profile version, file hashes, read-back
parameters and `log_dropped` - it must be 0 for a run to be trusted.

## Files

| File | Role | Change it when |
|---|---|---|
| `bench.json` | configs to compare, repeats, workers | every experiment |
| `5inch.json` | SITL physics model of the airframe | you measure the real quad |
| `5inch.parm` | base parameters + fixed test conditions + logging | rarely (changes the baseline) |
| `flight_profile.py` | the standard flight (bump `PROFILE_VERSION` if edited) | rarely |
| `sitl.py` | launches SITL, flies the profile over MAVLink | - |
| `analyze.py` | log -> metrics | to add metrics |
| `run_bench.py` | orchestrates runs, writes CSVs | - |
| `atlas_link.py` | ATLAS aircraft → sim, safety check, scoring, filing | to change the score |

## Notes

- Logging is deliberately lean (50 Hz attitude/rate): the Windows SITL build
  writes logs slowly and full-rate logging drops most messages.
- Instance slots 0..workers-1 are reused, so ports stay at 5760/5770...;
  high instance numbers collide with Mission Planner (UDP 5601).
- Keep `workers` at 2-3 unless you confirm `log_dropped` stays 0.
