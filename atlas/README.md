# ATLAS v9 | Acro Configuration Intelligence

Run `run_atlas.bat` on Windows, or `python atlas_v9.py`. Python 3 with Tkinter
required. **No pip packages are needed to launch**, browse, retrieve cases, or
predict a tune — the random forest is implemented in the standard library.
`pymavlink` is needed only to write to a flight controller or run simulations,
and its absence is reported in those two tabs rather than stopping the program.

```
pip install pymavlink      # only for the Flight Controller and Simulation tabs
```

---

## What changed in v9

v8 was a case-retrieval repository: it stored aircraft, found the most similar
stored build, and copied that build's parameter snapshot out to a file. It had
no model, no connection to an aircraft, and no numeric check on what it
exported. v9 adds the Acro tuning layer and the glue between them.

**Three new tabs**

| Tab | What it does |
|---|---|
| **Predict Tuning** | Multi-output random forest. Predicts 8 free Acro parameters at once from 6 hardware measurements, derives 3 more, and reports tree disagreement, out-of-bag error, and feature importance. |
| **Flight Controller** | Connect over SITL / USB / ESP32 WiFi, read the current tune, stage a proposed one, see every value's safety verdict, and write it after confirming. The only screen that can change an aircraft. |
| **Simulation** | Sweep one parameter across a range in ArduPilot SITL, flying a scripted Acro sequence per candidate and scoring rate-loop tracking. Results feed back into the repository. |
| **Standard Bench** | Runs `../sitl_bench`: one identical full test flight per configuration and repeat in Mission Planner's SITL, with the airframe and baseline tune built from the chosen aircraft. Configurations pass the safety check before anything flies; each run is scored 0–1 from its onboard log and filed as `source='sitl'` evidence. Bench runs appear in the Simulation table as type *Standard bench* — their score is a different measure from an Acro sweep's. |

**Changes to existing behavior**

- `Add Aircraft` gained section 3, *Airframe measurements*: all-up weight, prop
  size, cell count, wheelbase. v8 recorded none of these, and four of the six
  model features cannot be derived without them. Aircraft without them still
  store and still work for case retrieval; they just cannot train or receive a
  prediction, and the Predict tab names them.
- `CBR Assistant` export now checks the snapshot's Acro parameters against the
  approved range and records the result in the `.review.json`. v8 exported them
  with only a prefix-based text warning.
- `Overview` gained a **Trainable cases** metric and now describes how a tune
  actually reaches an aircraft.
- Scrollbars were added to the result panes. The CBR explanation pane in v8
  could not be scrolled, so long feature lists were unreachable.

---

## The rule the whole design exists to enforce

```
retrieve or predict  →  safety bounds  →  you confirm  →  MAVLink write
```

No model output reaches an aircraft directly. Case retrieval and the forest
both hand their proposal to the same check in `safety.py`, which **fails
closed**: a parameter outside its approved range is blocked, and a parameter
that is not on the approved list at all is blocked too. Blocked values are
never clamped into range behind your back and never silently dropped — they
appear in the table in red with the reason.

`ardupilot_interface.py` is the only module that imports pymavlink, so there is
exactly one place to audit the question *can this program change my aircraft?*
`Vehicle.apply_params()` additionally refuses to write while the aircraft is
armed, and requires a confirmation callback that has no default and no bypass.

Every write is recorded in `applied_params` with a timestamp, the connection
string, what was requested, what the autopilot confirmed, and where the values
came from.

---

## Connecting to SITL on a different computer

`127.0.0.1` is **loopback** — it means "this same computer" and can never reach
another machine. Running ATLAS and SITL on separate computers works fine, but
needs a different connection string on both ends.

| | ATLAS connection string | SITL launch line |
|---|---|---|
| **Same computer** | `udp:127.0.0.1:14550` | `sim_vehicle.py -v ArduCopter -f quad --out=udp:127.0.0.1:14550` |
| **Different computers** | `udpin:0.0.0.0:14550` | `sim_vehicle.py -v ArduCopter -f quad --out=udp:ATLAS_PC_IP:14550` |

`0.0.0.0` binds every network interface, so telemetry arriving from the network
is accepted. No IP address goes in the ATLAS field — this side only listens, and
pymavlink learns where to send commands from the first packet it receives.
`ATLAS_PC_IP` is the ATLAS machine's own address (`ipconfig` on Windows,
`hostname -I` on Linux/Mac). Without `--out`, `sim_vehicle.py` only talks to its
own machine.

**Run `check_link.py` on the ATLAS computer to test the link by itself.** It
prints this machine's addresses, the exact SITL launch line to use, whether
anything already holds the port, and then listens and reports what it heard. A
UDP timeout looks identical whether the simulator is off, aimed at the wrong
address, firewalled, or on a network that will not route between hosts — this
separates those cases.

Things that cause a silent timeout even when both ends look right:

- **Windows Defender blocks inbound UDP to Python by default.** Allow it, or
  test with the firewall briefly off to confirm that is the cause.
- **Campus, guest and VPN networks commonly block host-to-host traffic.** Ping
  the ATLAS machine from the SITL machine; if that fails, nothing in ATLAS can
  work around it, and the fix is a shared network or one computer.
- **Only one program can bind a UDP port.** Mission Planner, QGroundControl or
  a second copy of ATLAS holding 14550 will stop this one from receiving. Add a
  second SITL output on another port instead: `--out=udp:ATLAS_PC_IP:14551`.

If it is fighting you, running both on one computer removes every networking
variable at once.

---

## Simulated flight logs

Each simulated run can produce real log files and be filed in the repository as
flight evidence. Controls are on the Simulation tab.

| | What it is | When to use it |
|---|---|---|
| **.tlog** | Every MAVLink packet this program received, with timestamps. Readable by Mission Planner and MAVExplorer. | Always — it costs nothing and works over any link, including a network one. On by default. |
| **.bin** | The autopilot's own high-rate dataflash log, holding data that never crosses telemetry. | When you want the authoritative log. Fetched over MAVLink at 90 bytes per packet, so it is slow. |

If SITL runs on the **same** computer, its `.bin` files are already in
`logs/` next to `sim_vehicle.py` — copying them is far faster than downloading.
The MAVLink download exists because when SITL is on **another** computer those
files are on that computer and MAVLink is the only route to them.

A partial download is never written to disk. A dataflash log with silent gaps
decodes as corrupt data rather than failing cleanly, so a download that cannot
be completed raises instead of leaving a plausible-looking bad file.

Logs land in `sim_logs/`. With *File each run as flight evidence* ticked and an
aircraft selected on the Repository tab, each run also creates:

- a `parameter_files` row — the tune that was flown, so it is recoverable
- a `flight_tests` row per log file, with the log stored in `repository_files/`
- an `acro_outcomes` row carrying the numeric score, tracking error and vibration

### Simulated evidence is kept separate, deliberately

Everything filed from a simulation is marked `source='sitl'`, and the aircraft's
`validation_status` is never changed. A simulated flight is evidence about a
tune; it is not validation of an airframe.

That separation is enforced in four places, because the first implementation
leaked in three of them:

- `outcome_label()` counts only `source='hardware'` flight tests. Without that
  filter, a simulated flight counts as a hardware pass, and an UNTESTED airframe
  that has only ever flown in SITL enters the hardware training set with a
  perfect label.
- `latest_param_file()` returns only hardware snapshots by default. A simulated
  run files the tune it flew, which would otherwise become that aircraft's
  newest snapshot — and "newest snapshot" is what target extraction reaches for.
- `cbr_engine.candidate()` exports only hardware snapshots, so a case export
  cannot hand out a simulated tune alongside that aircraft's hardware
  validation status.
- The prediction model still excludes simulated rows unless *Include SITL runs
  in training* is ticked, and simulated rows must clear a higher score
  threshold when it is.

---

## Importing an existing build

```
python import_build.py --dry-run \
    --name "Lucid H7 QAV-R 5in" \
    --params AP_FW_4_7_Lucid_H7.bin \
    --components 5in_Components.xlsx
```

`--dry-run` prints what it would store and changes nothing. Drop it to commit.

It handles both things people call a `.bin` — a CSV/text parameter export,
whatever extension it carries, and a real binary dataflash log, whose PARM
messages are read out with pymavlink. The file is **sniffed, not trusted by
extension**: parsing a binary log as text yields thousands of plausible-looking
garbage parameter names and nothing downstream would catch it.

From the parts list and the parameters it derives motor KV and stator, motor
count (from `FRAME_CLASS`), prop size, and cell count (from the ESC rating,
cross-checked against `BATT_ARM_VOLT`). It refuses to invent the rest. **All-up
weight appears in no parameter file and no parts list** — pass `--weight-g`.

It also reports disagreements between the parts list and the configuration, for
example a GPS module in the parts list on a build whose `GPS1_TYPE=0`.

### Parts go into the hardware catalog too

Importing a build also adds each part to the reusable **hardware catalog**, so
it appears in the Add Aircraft dropdowns on the next build instead of being
retyped. `--no-catalog` turns that off.

Manufacturer is worked out from a brand prefix in the part name, then the note
column, then the manufacturer's own domain in the link — retailer domains are
ignored, since `getfpv.com` says nothing about who made the part. When no brand
can be identified the manufacturer is left blank rather than promoting a model
number into it.

For parts already recorded against an aircraft but never catalogued — older
builds, or anything typed in by hand — the Hardware Catalog tab has **Add
Missing From Aircraft**, which lists them and adds them on confirmation. It
carries a product link across but not per-build specs: `Qty 4` belongs to one
build, not to the part.

v9 adds **Frame** and **GPS** categories, since the importer classifies both.

### Two things it will not decide for you

**Validation status.** A parameter file records what a tune *was*, not how it
flew, so `--validation` defaults to `UNTESTED` and is never inferred. An
UNTESTED aircraft is excluded from training entirely — not scored zero, but
excluded — so a build you know flies well must be set to
`PARTIALLY_VALIDATED` or `FULLY_VALIDATED` before it counts as a training
example.

**An outcome score.** That needs flight evidence: a log, a result, or a score
recorded against a flight. A parameter export carries none.

---

## How the modules connect

Everything except `make_demo_data.py` is reachable from the interface:

```
atlas_v9.py  (the interface)
  ├── acro_bridge.py ──── safety.py        features, targets, labels, migration
  ├── ml_model.py ─────── acro_bridge, safety    prediction
  ├── cbr_engine.py                        case retrieval
  ├── ardupilot_interface.py ─ safety      the only module that writes to an aircraft
  ├── sim_iterate.py ──── acro_bridge, safety, sim_logs   the sweep
  ├── sim_logs.py ─────── acro_bridge      telemetry capture, .bin download, filing
  ├── check_link.py                        the Test Connection diagnostic
  └── safety.py                            approved ranges, fail closed
```

`make_demo_data.py` is a one-off terminal script and is deliberately not wired
in — it writes synthetic aircraft, which nothing in the interface should be
able to do by accident.

---

## Files

| File | Role |
|---|---|
| `atlas_v9.py` | The interface. |
| `acro_bridge.py` | **The glue.** Converts ATLAS rows into model vectors and back. Also owns the schema migration. |
| `safety.py` | Approved parameter ranges, fail closed. The single definition of "allowed to leave". |
| `ml_model.py` | Multi-output random forest, standard library only. |
| `ardupilot_interface.py` | The only module that talks to a flight controller. |
| `sim_iterate.py` | SITL harness. Also runnable standalone: `python sim_iterate.py --sweep ATC_RAT_RLL_P 0.03 0.12 5` |
| `cbr_engine.py` | v8's case retrieval, unchanged. |
| `sim_logs.py` | Telemetry capture, dataflash download over MAVLink, and filing a simulated run as repository evidence. |
| `check_link.py` (also a button) | Connection diagnostic. Reachable from the Flight Controller tab as **Test Connection**, or run from a terminal. |
| `import_build.py` | Adds a build from a parameter file plus a components list. Reads CSV parameter exports **and** real binary dataflash logs. `--dry-run` shows what it would store. |
| `make_demo_data.py` | Writes 17 synthetic aircraft so the Predict tab can be demonstrated. **The data is fake.** `--remove` deletes them. |
| `atlas_repository.db` | The repository. Keep it with the scripts and `repository_files/`. |

### What the bridge actually translates

ATLAS and the tuner were built against different data models and neither could
read the other's:

| ATLAS stores | the model needs |
|---|---|
| `components.specs` = `"Stator 2207, 1700 KV, Qty 4"` (free text) | `motor_kv = 1700.0`, `motor_qty = 4` |
| `aircraft.frame_size` = `"5 inch"` | `prop_size_in = 5.0`, `wheelbase_mm = 220` |
| *(nothing)* | `weight_g`, `cell_count` |
| `parameters(name TEXT, value TEXT)` | `ATC_RAT_RLL_P = 0.03205` |
| `validation_status = 'UNTESTED'` | a label in 0–1, or exclusion |

The Predict tab's **Load** button shows where each value came from — recorded,
parsed from text, or inferred — so an inferred wheelbase is never mistaken for
a measured one.

---

## Decisions worth knowing about

**UNTESTED aircraft are excluded from training, not scored zero.** An unflown
configuration is absence of evidence, not evidence of failure. Scoring it zero
would teach the model to avoid configurations nobody has tried yet.

**Only parameters that are genuinely free are predicted.** The real Lucid H7
file shows roll P == roll I exactly and yaw I == yaw P ÷ 10. Those are
conventions, not independent choices, so they are derived from the predicted
values rather than predicted separately — otherwise the model can emit a
combination no human would write.

**Tree ensembles over deep learning, because of sample size.** The published
comparisons put the crossover around 10,000 samples; a capstone repository will
have tens of *builds*, not thousands of flights.

**A random forest cannot extrapolate.** Outside the span of its training data
it returns the nearest case it knows while still looking precise. The Predict
tab checks for this and says so in the banner.

**The schema the source declared was not the schema the database had.**
`atlas_v8.py` declares `ON DELETE CASCADE` on the child tables, but
`CREATE TABLE IF NOT EXISTS` never alters an existing table, so the shipped
`atlas_repository.db` kept an older definition with `NO ACTION` and deleting an
aircraft failed with a foreign-key error. `acro_bridge.migrate()` detects this
and rebuilds the four affected tables, verifying with `PRAGMA foreign_key_check`
before committing. It runs once and reports what it changed.

---

## Critical limits

- **Not a flight-ready generator.** A prediction is an interpolation between
  stored builds. A retrieved candidate is a copy of someone else's snapshot.
  Neither is a validated tune for your aircraft.
- **Validation belongs to the source aircraft** and never transfers
  automatically to a target.
- **Similarity scores and prediction scores are not safety scores.**
- **SITL is not your airframe.** The default simulated quad has no frame
  resonance, prop imbalance or motor lag unless you configure `SIM_*` to match.
  Simulation scores are comparable to each other within one sweep and mean
  nothing in absolute terms.
- **Never load a generated tune without bench testing, then simulation, then
  progressive flight testing.** Keep firmware and vehicle type compatible.
- The shipped database has **zero aircraft** and 22 hardware catalog entries.
  Prediction needs **12 usable configurations** minimum — and at 12–30, the
  honest description of the output is still "an interpolation between similar
  builds", which is why case retrieval remains available and works from one
  stored case.
- Back up `atlas_repository.db` before production use.

---

## Suggested capstone evaluation

Compare manual configuration against ATLAS on completion time, number of user
actions, parameter corrections required, task completion, and NASA-TLX. Three
things make a defensible experiment here:

1. **Compare against the real baseline, which is AUTOTUNE plus published
   prop-size heuristics** — not against starting from firmware defaults.
2. **Report out-of-bag error, not training error.** The Predict tab shows it.
3. **Split by build, not by row.** Two parameter files from one aircraft are
   not two independent samples, and a random split leaks between them.
