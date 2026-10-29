# DSE-26

| Folder | What it is |
|---|---|
| [`atlas/`](atlas/README.md) | **ATLAS v14**: Acro configuration intelligence. Stores builds, retrieves similar cases, predicts an Acro tune, checks it against safety bounds, and writes it to a flight controller only after confirmation. |
| [`sitl_bench/`](sitl_bench/README.md) | **Standardized SITL test bench**: flies an identical test flight in ArduPilot SITL for each parameter configuration and scores every run from its onboard log, so runs can be compared directly. |

Both run on Windows with Python 3 and `pip install pymavlink`. The bench uses
Mission Planner's SITL binary (`Documents\Mission Planner\sitl\ArduCopter.exe`).
Bench results are written to `sitl_bench/runs/<timestamp>/`.
