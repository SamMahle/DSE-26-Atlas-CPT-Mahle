"""
sim_logs.py — produce real log files from simulated flights and file them in
the repository as flight evidence.

TWO KINDS OF LOG, AND WHY BOTH EXIST
------------------------------------
**.tlog (telemetry log)** — every MAVLink packet this program received, written
as it arrives. Costs nothing, works over any link including a network one, and
is exactly what the scorer saw. Its rate is the telemetry rate, so it is
coarser than the autopilot's own record.

**.bin (dataflash log)** — the autopilot's own high-rate internal log, the file
Mission Planner and the ArduPilot log analysers expect. It holds data that
never goes over telemetry at all. Getting it needs either a file copy from the
SITL machine, or a download over MAVLink — which is what this module
implements, because when SITL runs on another computer the file is on that
computer and MAVLink is the only route to it.

The download uses LOG_REQUEST_LIST / LOG_REQUEST_DATA. It is slow — the
autopilot returns 90 bytes per packet — so a long flight takes a while. The
short Acro sequences this harness flies produce small logs.

WHAT GETS FILED, AND HOW IT IS MARKED
-------------------------------------
A simulated run becomes a parameter_files row (the tune that was flown), a
flight_tests row (the log, with source='sitl'), and an acro_outcomes row (the
numeric score, with source='sitl'). It does NOT touch the aircraft's
validation_status. A simulated flight is evidence; it is not validation, and
the prediction model keeps excluding simulated rows unless asked for them.
"""

from __future__ import annotations

import json
import shutil
import sqlite3
import struct
import time
from pathlib import Path
from typing import Any, Callable

import acro_bridge as bridge

PROGRESS = Callable[[str], None]


# ==========================================================================
# .tlog — raw telemetry capture
# ==========================================================================
class TelemetryLog:
    """Writes every received MAVLink packet to a .tlog.

    DO NOT replace this with mavutil's setup_logfile(). That looks like it
    should work and silently does not on a UDP link: mavudp overrides
    recv_msg() and its version omits the logfile write that the base class
    performs. The file is created and stays empty, with no error — and UDP is
    what SITL uses, so that is exactly the case that matters here.

    Instead the connection's recv_msg is wrapped for the life of the log and
    each parsed message is written in the same format the base class uses:
    an 8-byte big-endian microsecond timestamp followed by the raw message
    bytes. That is the standard tlog Mission Planner and MAVExplorer read, and
    it works on every transport.
    """

    def __init__(self, master, path: str | Path):
        self.master = master
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = None
        self._original_recv_msg = None
        self.messages = 0

    def start(self) -> None:
        if self._fh is not None:
            return
        self._fh = open(self.path, "wb")
        original = self.master.recv_msg

        def logging_recv_msg():
            msg = original()
            if msg is not None and self._fh is not None:
                try:
                    if msg.get_type() != "BAD_DATA":
                        usec = int(time.time() * 1.0e6) & ~3
                        self._fh.write(struct.pack(">Q", usec) + msg.get_msgbuf())
                        self.messages += 1
                except (ValueError, OSError):
                    pass            # a broken log must never break the flight
            return msg

        self._original_recv_msg = original
        self.master.recv_msg = logging_recv_msg

    def stop(self) -> int:
        """Restore the connection, close the log, return its size in bytes."""
        if self._original_recv_msg is not None:
            self.master.recv_msg = self._original_recv_msg
            self._original_recv_msg = None
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        return self.path.stat().st_size if self.path.exists() else 0

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


# ==========================================================================
# .bin — dataflash download over MAVLink
# ==========================================================================
def list_logs(master, timeout_s: float = 10.0) -> list[dict[str, Any]]:
    """Ask the autopilot what logs it holds.

    Returns a list of {id, size, time_utc}, newest id last.
    """
    mav = master.mav
    master.mav.log_request_list_send(master.target_system,
                                     master.target_component, 0, 0xFFFF)
    entries: dict[int, dict[str, Any]] = {}
    expected = None
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="LOG_ENTRY", blocking=True,
                                timeout=max(0.1, deadline - time.time()))
        if msg is None:
            continue
        if msg.num_logs == 0:
            return []
        expected = msg.num_logs
        entries[msg.id] = {"id": msg.id, "size": msg.size,
                           "time_utc": msg.time_utc,
                           "last_log_num": msg.last_log_num}
        if expected is not None and len(entries) >= expected:
            break
    return [entries[k] for k in sorted(entries)]


def download_log(master, log_id: int, dest: str | Path, *,
                 progress: PROGRESS | None = None,
                 chunk_timeout_s: float = 5.0,
                 max_stalls: int = 6) -> Path:
    """Pull one dataflash log off the autopilot and write it to `dest`.

    The protocol hands back 90 bytes per LOG_DATA packet and UDP loses
    packets, so missing ranges are re-requested rather than assumed absent —
    a log assembled with silent gaps would look valid and decode as garbage.
    """
    say = progress or (lambda _m: None)
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)

    listing = {e["id"]: e for e in list_logs(master)}
    if log_id not in listing:
        raise ValueError(f"autopilot has no log #{log_id} "
                         f"(it reports: {sorted(listing) or 'none'})")
    size = listing[log_id]["size"]
    if size == 0:
        raise ValueError(f"log #{log_id} is empty")

    say(f"Downloading log #{log_id} ({size / 1024:.0f} KiB)")
    buf = bytearray(size)
    have = bytearray(size)          # per-byte received flag
    received = 0
    stalls = 0

    def request(offset: int, count: int) -> None:
        master.mav.log_request_data_send(
            master.target_system, master.target_component,
            log_id, offset, count)

    def first_gap(start: int = 0) -> tuple[int, int] | None:
        i = start
        while i < size and have[i]:
            i += 1
        if i >= size:
            return None
        j = i
        while j < size and not have[j] and (j - i) < 0xFFFFFF:
            j += 1
        return i, j - i

    gap = first_gap()
    while gap is not None and stalls < max_stalls:
        offset, count = gap
        request(offset, count)
        got_any = False
        deadline = time.time() + chunk_timeout_s
        while time.time() < deadline:
            msg = master.recv_match(type="LOG_DATA", blocking=True,
                                    timeout=max(0.1, deadline - time.time()))
            if msg is None:
                break
            if msg.id != log_id:
                continue
            n = min(msg.count, size - msg.ofs)
            if n <= 0:
                continue
            chunk = bytes(msg.data[:n])
            buf[msg.ofs:msg.ofs + n] = chunk
            for k in range(msg.ofs, msg.ofs + n):
                if not have[k]:
                    have[k] = 1
                    received += 1
            got_any = True
            if received >= size:
                break
            # Keep reading while the stream flows; only re-request on a stall.
            deadline = time.time() + chunk_timeout_s
        if received >= size:
            break
        stalls = 0 if got_any else stalls + 1
        if stalls:
            say(f"  stalled at {received * 100 // size}% — re-requesting "
                f"(attempt {stalls}/{max_stalls})")
        gap = first_gap()
        say(f"  {received * 100 // size}% ({received}/{size} bytes)")

    try:
        master.mav.log_request_end_send(master.target_system,
                                        master.target_component)
    except Exception:
        pass

    missing = size - received
    if missing:
        raise TimeoutError(
            f"log #{log_id} incomplete: {missing} of {size} bytes never "
            f"arrived after {max_stalls} stalled re-requests. The file was NOT "
            "written — a partial dataflash log decodes as corrupt data rather "
            "than failing cleanly.\n\n"
            "If SITL is on another computer, copying the file directly from "
            "its logs/ folder is faster and more reliable than this download."
        )

    dest.write_bytes(bytes(buf))
    say(f"Saved {dest.name} ({size / 1024:.0f} KiB)")
    return dest


def newest_log_id(master) -> int | None:
    logs = list_logs(master)
    return logs[-1]["id"] if logs else None


# ==========================================================================
# Filing a simulated run as repository evidence
# ==========================================================================
def ingest_sim_flight(conn: sqlite3.Connection,
                      aircraft_id: int | None,
                      params: dict[str, float],
                      score: dict[str, Any],
                      *,
                      files_dir: str | Path,
                      tlog_path: str | Path | None = None,
                      bin_path: str | Path | None = None,
                      run_group: str = "",
                      firmware: str = "",
                      sim_run_id: int | None = None) -> dict[str, Any]:
    """File one simulated flight: the tune, the log, and the score.

    Creates a parameter_files row (so the flown tune is recoverable), a
    flight_tests row per log file, and an acro_outcomes row carrying the
    numeric score. Returns the ids it created.

    The aircraft's validation_status is deliberately NOT changed. A simulated
    flight is evidence about a tune, not validation of an airframe, and the
    moment this function starts promoting statuses the repository stops being
    able to tell the two apart.
    """
    if aircraft_id is None:
        return {"filed": False,
                "reason": "no aircraft selected — the run was scored and stored "
                          "in sim_runs, but nothing was filed as flight evidence"}

    files_dir = Path(files_dir)
    files_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d%H%M%S")
    created: dict[str, Any] = {"filed": True, "flight_test_ids": []}

    # --- the tune that was flown -----------------------------------------
    expanded = bridge.expand_derived(params)
    param_name = f"SITL_{run_group or stamp}_{stamp}.param"
    param_file = files_dir / f"{aircraft_id}_{param_name}"
    param_file.write_text(
        f"# Simulated flight — ArduPilot SITL, not hardware\n"
        f"# run group: {run_group}\n"
        f"# score: {score.get('outcome_score')}\n"
        + "".join(f"{k},{v:.8g}\n" for k, v in sorted(expanded.items())),
        encoding="utf-8")
    pf_id = conn.execute(
        "INSERT INTO parameter_files(aircraft_id, original_name, stored_path, "
        "ardupilot_version, imported_at, source) VALUES(?,?,?,?,?,?)",
        (aircraft_id, param_name, str(param_file),
         firmware or "SITL", time.strftime("%Y-%m-%dT%H:%M:%S"),
         bridge.SOURCE_SIM)).lastrowid
    conn.executemany(
        "INSERT INTO parameters(parameter_file_id, name, value) VALUES(?,?,?)",
        [(pf_id, k, f"{v:.8g}") for k, v in sorted(expanded.items())])
    created["parameter_file_id"] = pf_id

    # --- the log files ----------------------------------------------------
    result = "Fail" if score.get("aborted") else (
        "Pass" if (score.get("outcome_score") or 0) >= 0.7 else "Mixed")
    note_lines = [
        "SIMULATED FLIGHT — ArduPilot SITL. Not hardware evidence.",
        f"Run group: {run_group}" if run_group else "",
        f"Outcome score: {score.get('outcome_score')}",
        f"Aborted: {score.get('abort_reason')}" if score.get("aborted") else "",
        f"Axes scored: {', '.join(score.get('axes_scored', []))}"
        if score.get("axes_scored") else "",
        f"Relative tracking error: {json.dumps(score.get('rel_rmse', {}))}"
        if score.get("rel_rmse") else "",
        f"Vibration max: {score.get('vibration_max_m_s2')} m/s^2"
        if score.get("vibration_max_m_s2") is not None else "",
    ]
    notes = "\n".join(l for l in note_lines if l)

    for src, kind in ((bin_path, "bin"), (tlog_path, "tlog")):
        if not src:
            continue
        src = Path(src)
        if not src.exists() or src.stat().st_size == 0:
            continue
        stored = files_dir / f"{aircraft_id}_{stamp}_SITL_{src.name}"
        if src.resolve() != stored.resolve():
            shutil.copy2(src, stored)
        ft_id = conn.execute(
            "INSERT INTO flight_tests(aircraft_id, parameter_file_id, log_name, "
            "stored_path, test_date, result, notes, source) "
            "VALUES(?,?,?,?,?,?,?,?)",
            (aircraft_id, pf_id, f"SITL_{src.name}", str(stored),
             time.strftime("%Y-%m-%d"), result,
             notes + f"\nLog type: {kind}", bridge.SOURCE_SIM)).lastrowid
        created["flight_test_ids"].append(ft_id)

    # --- the score --------------------------------------------------------
    rel = score.get("rel_rmse") or {}
    outcome_id = bridge.record_outcome(
        conn, aircraft_id, score.get("outcome_score"),
        flight_test_id=(created["flight_test_ids"] or [None])[0],
        tracking_error=(sum(rel.values()) / len(rel)) if rel else None,
        vibration_max=score.get("vibration_max_m_s2"),
        oscillation_detected=bool(
            (score.get("terms", {}) or {}).get("oscillation", 1.0) < 0.5),
        source=bridge.SOURCE_SIM,
        notes=notes)
    created["acro_outcome_id"] = outcome_id

    if sim_run_id is not None:
        conn.execute("UPDATE sim_runs SET aircraft_id=? WHERE id=?",
                     (aircraft_id, sim_run_id))
    conn.commit()
    created["result"] = result
    return created


def sim_flight_summary(conn: sqlite3.Connection,
                       aircraft_id: int | None = None) -> dict[str, Any]:
    """What simulated evidence is on file, for the interface to display."""
    where = "WHERE source=?" + (" AND aircraft_id=?" if aircraft_id else "")
    args = [bridge.SOURCE_SIM] + ([aircraft_id] if aircraft_id else [])
    flights = conn.execute(
        f"SELECT COUNT(*) FROM flight_tests {where}", args).fetchone()[0]
    scored = conn.execute(
        f"SELECT COUNT(*), AVG(outcome_score) FROM acro_outcomes {where}",
        args).fetchone()
    return {"sim_flight_tests": flights,
            "sim_outcomes": scored[0],
            "mean_sim_score": round(scored[1], 4) if scored[1] is not None else None}
