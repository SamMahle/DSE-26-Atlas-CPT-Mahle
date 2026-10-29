#!/usr/bin/env python3
"""
sim_iterate.py — iterate Acro flight simulations over candidate ArduPilot
parameter sets using pymavlink + SITL.

WHAT THIS DOES
--------------
For each candidate parameter set:
    1. validate it against safety bounds (fail closed — unknown name = reject)
    2. push the parameters over MAVLink and read them back to confirm
    3. arm, climb to test altitude in GUIDED, switch to ACRO
    4. fly a scripted stick-input sequence via RC_CHANNELS_OVERRIDE
    5. record ATTITUDE / ATTITUDE_TARGET / VIBRATION telemetry throughout
    6. land, disarm, (optionally) reboot the autopilot for a clean slate
    7. score the rate-loop response and write the run to SQLite

The score is built from how well the measured body rates tracked the
commanded body rates — which is exactly what the ATC_RAT_* gains control and
exactly what Acro mode exercises. Stabilize-mode angle tracking is NOT
measured here.

REQUIRES A RUNNING SITL. Start one first, e.g.:
    sim_vehicle.py -v ArduCopter -f quad --console --out=udp:127.0.0.1:14550
then:
    python sim_iterate.py --sweep ATC_RAT_RLL_P 0.03 0.12 5

SCOPE / HONEST LIMITS
---------------------
* SITL's default quad has fixed, well-behaved simulated physics. Gains that
  score well here are a STARTING POINT for a real airframe, not a validated
  tune. SITL does not reproduce frame resonance, prop imbalance, or motor lag
  unless you configure SIM_* parameters to match.
* ACRO has no altitude hold. This harness climbs in GUIDED, then holds a fixed
  throttle in ACRO and aborts the run if altitude leaves a safe band. Keep
  maneuvers short.
* Scores are comparable to each other within one sweep. They are not an
  absolute measure of flight quality.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any

# pymavlink is imported LAZILY. Everything in this module that does not fly —
# sweep generation, safety validation, scoring, the results schema — must work
# without it, or the interface cannot even check a sweep before offering to run
# it, and an ImportError masks the real verdict.
try:
    from pymavlink import mavutil
except ImportError:                                   # pragma: no cover
    mavutil = None


def _require_mavutil():
    if mavutil is None:
        raise RuntimeError(
            "pymavlink is not installed, so simulated flights cannot run.\n\n"
            "    pip install pymavlink\n\n"
            "Sweep validation and scoring work without it."
        )
    return mavutil


# --------------------------------------------------------------------------
# Safety bounds
#
# Prefer the project's shared table so the harness and the recommender can
# never disagree about what is in range. Falls back to an embedded copy so
# this file also runs standalone.
# --------------------------------------------------------------------------
try:
    from safety import ACRO_PARAM_RANGES, validate_param_set  # type: ignore
except ImportError:
    ACRO_PARAM_RANGES: dict[str, tuple[float, float]] = {
        "ATC_RAT_RLL_P": (0.02, 0.35),
        "ATC_RAT_RLL_I": (0.02, 0.35),
        "ATC_RAT_RLL_D": (0.001, 0.02),
        "ATC_RAT_PIT_P": (0.02, 0.35),
        "ATC_RAT_PIT_I": (0.02, 0.35),
        "ATC_RAT_PIT_D": (0.001, 0.02),
        "ATC_RAT_YAW_P": (0.1, 0.6),
        "ATC_RAT_YAW_I": (0.01, 0.1),
        "ATC_RAT_YAW_D": (0.0, 0.02),
        "ACRO_RP_RATE": (180.0, 1080.0),
        "ACRO_Y_RATE": (90.0, 1080.0),
        "ACRO_RP_EXPO": (0.0, 1.0),
        "ACRO_Y_EXPO": (0.0, 1.0),
        "INS_GYRO_FILTER": (10.0, 120.0),
    }

    def validate_param_set(params: dict[str, float]) -> tuple[bool, list[str]]:
        """Fail closed: a name that is not in the table is rejected."""
        problems = []
        for name, value in params.items():
            if name not in ACRO_PARAM_RANGES:
                problems.append(f"{name}: not an approved tuning parameter")
                continue
            low, high = ACRO_PARAM_RANGES[name]
            if not (low <= float(value) <= high):
                problems.append(f"{name}={value} outside [{low}, {high}]")
        return (not problems), problems


# Copter custom-mode numbers. mavutil ships number->name; we need name->number.
# The literal fallback keeps this module importable without pymavlink; the
# values are a stable part of the Copter protocol.
_COPTER_MODE_FALLBACK = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "POSITION": 8, "LAND": 9,
    "OF_LOITER": 10, "DRIFT": 11,
}
COPTER_MODES = ({name: num for num, name in mavutil.mode_mapping_acm.items()}
                if mavutil is not None else dict(_COPTER_MODE_FALLBACK))

RC_MID = 1500
RC_MIN = 1100
RC_MAX = 1900


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
@dataclass
class Maneuver:
    """One scripted stick input.

    axis: 'roll' | 'pitch' | 'yaw' | 'center'
    deflection: -1.0 .. 1.0 as a fraction of full stick travel
    seconds: how long to hold it
    """
    axis: str
    deflection: float
    seconds: float
    label: str = ""

    def __post_init__(self):
        if self.axis not in ("roll", "pitch", "yaw", "center"):
            raise ValueError(f"unknown axis {self.axis!r}")
        if not -1.0 <= self.deflection <= 1.0:
            raise ValueError("deflection must be within -1.0..1.0")
        if not self.label:
            self.label = f"{self.axis}{self.deflection:+.2f}"


# A step input is the useful test: it excites the rate loop hard enough to
# reveal overshoot and oscillation, which is what P and D control.
DEFAULT_SEQUENCE: list[Maneuver] = [
    Maneuver("center", 0.0, 1.5, "settle"),
    Maneuver("roll", +0.6, 1.2, "roll_right"),
    Maneuver("center", 0.0, 1.0, "recover_1"),
    Maneuver("roll", -0.6, 1.2, "roll_left"),
    Maneuver("center", 0.0, 1.0, "recover_2"),
    Maneuver("pitch", +0.6, 1.2, "pitch_fwd"),
    Maneuver("center", 0.0, 1.0, "recover_3"),
    Maneuver("pitch", -0.6, 1.2, "pitch_back"),
    Maneuver("center", 0.0, 1.0, "recover_4"),
    Maneuver("yaw", +0.6, 1.2, "yaw_right"),
    Maneuver("center", 0.0, 1.5, "hold"),
]


@dataclass
class HarnessConfig:
    connection: str = "udp:127.0.0.1:14550"
    test_altitude_m: float = 30.0
    altitude_floor_m: float = 8.0       # abort the run below this
    altitude_ceiling_m: float = 90.0    # abort the run above this
    acro_throttle_pwm: int = 1500       # held through the whole Acro segment
    rc_rate_hz: float = 20.0            # overrides must be refreshed or RC fails over
    telemetry_hz: float = 50.0
    connect_timeout_s: float = 60.0
    arm_timeout_s: float = 60.0
    climb_timeout_s: float = 90.0
    reboot_between_runs: bool = True
    relax_arming_checks: bool = True    # SIMULATION ONLY — see apply_sim_setup()
    db_path: str = "sim_runs.db"


# --------------------------------------------------------------------------
# Telemetry sample
# --------------------------------------------------------------------------
@dataclass
class Sample:
    t: float                     # seconds since the Acro segment started
    phase: str                   # maneuver label
    rate_meas: tuple[float, float, float] = (0.0, 0.0, 0.0)   # rad/s, body frame
    rate_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)    # rad/s, body frame
    vibe: tuple[float, float, float] = (0.0, 0.0, 0.0)        # m/s^2
    clipping: tuple[int, int, int] = (0, 0, 0)
    alt_m: float = 0.0


# --------------------------------------------------------------------------
# The harness
# --------------------------------------------------------------------------
class SimHarness:
    def __init__(self, cfg: HarnessConfig):
        self.cfg = cfg
        self.master: Any = None

    # ---------------------------------------------------------- connection
    def connect(self) -> None:
        _require_mavutil()
        print(f"[conn] connecting to {self.cfg.connection}")
        self.master = mavutil.mavlink_connection(self.cfg.connection)
        hb = self.master.wait_heartbeat(timeout=self.cfg.connect_timeout_s)
        if hb is None:
            raise TimeoutError(
                f"no heartbeat on {self.cfg.connection} within "
                f"{self.cfg.connect_timeout_s}s — is SITL running?"
            )
        print(f"[conn] heartbeat from sys {self.master.target_system} "
              f"comp {self.master.target_component}")
        self._request_streams()

    def _request_streams(self) -> None:
        """Ask for the messages the scorer needs at a usable rate.

        SET_MESSAGE_INTERVAL is the modern mechanism; the legacy
        REQUEST_DATA_STREAM is sent as well because some builds still honor
        only that one.
        """
        interval_us = int(1_000_000 / self.cfg.telemetry_hz)
        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,
            mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        ):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, interval_us, 0, 0, 0, 0, 0,
            )
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, int(self.cfg.telemetry_hz), 1,
        )

    # ---------------------------------------------------------- parameters
    def set_param(self, name: str, value: float, retries: int = 4) -> float:
        """Set one parameter and confirm it by reading it back.

        ArduPilot sends every parameter over MAVLink as REAL32 regardless of
        its internal type, so REAL32 is correct even for integer parameters.
        """
        for attempt in range(retries):
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                name.encode("ascii"), float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            deadline = time.time() + 2.0
            while time.time() < deadline:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=True,
                                             timeout=max(0.1, deadline - time.time()))
                if msg is None:
                    break
                if msg.param_id.strip("\x00") == name:
                    got = float(msg.param_value)
                    tol = max(1e-6, abs(float(value)) * 1e-4)
                    if abs(got - float(value)) <= tol:
                        return got
                    print(f"[param] {name}: wrote {value}, autopilot reports {got} "
                          f"(attempt {attempt + 1})")
                    break
        raise RuntimeError(f"could not confirm {name}={value} after {retries} attempts")

    def apply_param_set(self, params: dict[str, float]) -> dict[str, float]:
        ok, problems = validate_param_set(params)
        if not ok:
            raise ValueError("rejected by safety bounds:\n  " + "\n  ".join(problems))
        confirmed = {}
        for name, value in params.items():
            confirmed[name] = self.set_param(name, value)
        print(f"[param] applied and confirmed {len(confirmed)} parameters")
        return confirmed

    def apply_sim_setup(self) -> None:
        """Parameters that make an unattended harness possible.

        These are NOT tuning parameters and are deliberately kept out of
        ACRO_PARAM_RANGES so they can never be recommended onto real
        hardware. Relaxing arming checks is acceptable in SITL only.
        """
        if self.cfg.relax_arming_checks:
            for name, value in (("ARMING_CHECK", 0), ("DISARM_DELAY", 0)):
                try:
                    self.set_param(name, value)
                except RuntimeError as exc:
                    print(f"[setup] {exc} (continuing)")

    # ---------------------------------------------------------------- flight
    def set_mode(self, mode_name: str, timeout_s: float = 10.0) -> None:
        if mode_name not in COPTER_MODES:
            raise ValueError(f"unknown copter mode {mode_name!r}")
        mode_num = COPTER_MODES[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num,
        )
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb and hb.custom_mode == mode_num:
                print(f"[mode] {mode_name}")
                return
            # re-send: mode changes are occasionally dropped on a lossy link
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_num,
            )
        raise TimeoutError(f"mode did not change to {mode_name} within {timeout_s}s")

    def wait_ready_to_arm(self, timeout_s: float = 120.0) -> None:
        """Wait for EKF/GPS to settle. Skipped when arming checks are off."""
        if self.cfg.relax_arming_checks:
            return
        print("[arm] waiting for prearm checks")
        self.master.wait_gps_fix()
        time.sleep(2.0)

    def arm(self) -> None:
        deadline = time.time() + self.cfg.arm_timeout_s
        while time.time() < deadline:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0,
            )
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("[arm] armed")
                    return
                print(f"[arm] rejected (result={ack.result}), retrying")
            time.sleep(1.0)
        raise TimeoutError("could not arm")

    def disarm(self, force: bool = False) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196 if force else 0, 0, 0, 0, 0, 0,
        )
        print("[arm] disarm sent")

    def altitude(self, timeout_s: float = 2.0) -> float:
        """Relative altitude in metres from GLOBAL_POSITION_INT."""
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT",
                                     blocking=True, timeout=timeout_s)
        return (msg.relative_alt / 1000.0) if msg else float("nan")

    def takeoff(self, altitude_m: float) -> None:
        """Climb in GUIDED, which holds position and altitude for us."""
        self.set_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm()
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude_m,
        )
        print(f"[fly] climbing to {altitude_m:.0f} m")
        deadline = time.time() + self.cfg.climb_timeout_s
        while time.time() < deadline:
            alt = self.altitude()
            if not math.isnan(alt) and alt >= altitude_m * 0.95:
                print(f"[fly] reached {alt:.1f} m")
                return
        raise TimeoutError(f"did not reach {altitude_m} m within "
                           f"{self.cfg.climb_timeout_s}s")

    def _send_rc(self, roll: int, pitch: int, throttle: int, yaw: int) -> None:
        """RC_CHANNELS_OVERRIDE. 0 means 'leave this channel alone';
        UINT16_MAX (65535) means 'release the override'."""
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            roll, pitch, throttle, yaw,
            0, 0, 0, 0,          # ch5..ch8 untouched
        )

    def release_rc(self) -> None:
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            *([65535] * 8),
        )

    @staticmethod
    def _pwm(deflection: float) -> int:
        span = (RC_MAX - RC_MIN) / 2.0
        return int(round(RC_MID + deflection * span))

    # --------------------------------------------------------- the Acro run
    def fly_sequence(self, sequence: list[Maneuver]) -> tuple[list[Sample], str]:
        """Fly the scripted sequence in ACRO, collecting telemetry.

        Returns (samples, abort_reason). abort_reason is "" on a clean run.
        """
        self.set_mode("ACRO")
        samples: list[Sample] = []
        t0 = time.time()
        abort = ""

        # Latest value of each message, carried forward between samples.
        rate_meas = (0.0, 0.0, 0.0)
        rate_cmd = (0.0, 0.0, 0.0)
        vibe = (0.0, 0.0, 0.0)
        clipping = (0, 0, 0)
        alt = self.cfg.test_altitude_m
        saw_attitude_target = False

        rc_period = 1.0 / self.cfg.rc_rate_hz

        for man in sequence:
            if abort:
                break
            roll = pitch = yaw = RC_MID
            d = self._pwm(man.deflection)
            if man.axis == "roll":
                roll = d
            elif man.axis == "pitch":
                pitch = d
            elif man.axis == "yaw":
                yaw = d

            seg_end = time.time() + man.seconds
            next_rc = 0.0
            while time.time() < seg_end:
                now = time.time()
                if now >= next_rc:
                    self._send_rc(roll, pitch, self.cfg.acro_throttle_pwm, yaw)
                    next_rc = now + rc_period

                msg = self.master.recv_match(
                    type=["ATTITUDE", "ATTITUDE_TARGET", "VIBRATION",
                          "GLOBAL_POSITION_INT"],
                    blocking=True, timeout=0.05,
                )
                if msg is None:
                    continue
                kind = msg.get_type()

                if kind == "ATTITUDE":
                    rate_meas = (msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                    # ATTITUDE is the metronome: one sample per ATTITUDE message
                    samples.append(Sample(
                        t=now - t0, phase=man.label,
                        rate_meas=rate_meas, rate_cmd=rate_cmd,
                        vibe=vibe, clipping=clipping, alt_m=alt,
                    ))
                elif kind == "ATTITUDE_TARGET":
                    rate_cmd = (msg.body_roll_rate, msg.body_pitch_rate,
                                msg.body_yaw_rate)
                    saw_attitude_target = True
                elif kind == "VIBRATION":
                    vibe = (msg.vibration_x, msg.vibration_y, msg.vibration_z)
                    clipping = (msg.clipping_0, msg.clipping_1, msg.clipping_2)
                elif kind == "GLOBAL_POSITION_INT":
                    alt = msg.relative_alt / 1000.0
                    if alt < self.cfg.altitude_floor_m:
                        abort = f"altitude fell to {alt:.1f} m"
                        break
                    if alt > self.cfg.altitude_ceiling_m:
                        abort = f"altitude rose to {alt:.1f} m"
                        break

        self.release_rc()
        if not saw_attitude_target and not abort:
            abort = ("no ATTITUDE_TARGET received — commanded rates unknown, "
                     "cannot score tracking")
        print(f"[fly] sequence done: {len(samples)} samples"
              + (f" — ABORTED: {abort}" if abort else ""))
        return samples, abort

    def land_and_disarm(self, timeout_s: float = 120.0) -> None:
        try:
            self.set_mode("LAND")
        except (TimeoutError, ValueError) as exc:
            print(f"[land] {exc} — forcing disarm")
            self.disarm(force=True)
            return
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            armed = bool(hb and hb.base_mode &
                         mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if hb and not armed:
                print("[land] landed and disarmed")
                return
        print("[land] still armed after timeout — forcing disarm")
        self.disarm(force=True)

    def reboot(self) -> None:
        """Reboot the autopilot so the next run starts from a known state.

        Without this, integrator windup and EKF state from the previous run
        leak into the next one and the comparison stops being fair.
        """
        print("[reset] rebooting autopilot")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        try:
            self.master.close()
        except Exception:
            pass
        time.sleep(5.0)
        self.connect()


# --------------------------------------------------------------------------
# Scoring
# --------------------------------------------------------------------------
def _rmse(errors: list[float]) -> float:
    if not errors:
        return float("nan")
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def _oscillation_hz(series: list[float], duration_s: float,
                    deadband: float) -> float:
    """Meaningful zero-crossings per second in the rate error.

    Two details matter and both were wrong in the obvious implementation:

    * A deadband is required. Sensor noise around zero error flips sign on
      almost every sample, so counting raw sign changes scores a PERFECT
      tracker as the most oscillatory run in the sweep. Only excursions
      larger than `deadband` count as real.
    * Normalize by time, not by sample count. Crossings per second is a
      property of the airframe; crossings per sample is a property of the
      telemetry rate, and would change the answer if the link slowed down.
    """
    if len(series) < 3 or duration_s <= 0 or deadband <= 0:
        return 0.0
    crossings = 0
    last_sign = 0
    for v in series:
        if abs(v) < deadband:
            continue                       # inside the deadband: no opinion
        sign = 1 if v > 0 else -1
        if last_sign != 0 and sign != last_sign:
            crossings += 1
        last_sign = sign
    return crossings / duration_s


def score_flight(samples: list[Sample], abort_reason: str = "") -> dict[str, Any]:
    """Turn a telemetry capture into comparable numbers.

    outcome_score is 0..1, higher is better, and is only meaningful relative
    to other runs in the same sweep on the same simulated airframe.
    """
    if abort_reason:
        return {
            "outcome_score": 0.0,
            "aborted": True,
            "abort_reason": abort_reason,
            "n_samples": len(samples),
        }
    if len(samples) < 20:
        return {
            "outcome_score": 0.0,
            "aborted": True,
            "abort_reason": f"only {len(samples)} samples — telemetry too sparse",
            "n_samples": len(samples),
        }

    axes = ("roll", "pitch", "yaw")
    errors = {a: [] for a in axes}
    peak_cmd = {a: 0.0 for a in axes}
    for s in samples:
        for i, a in enumerate(axes):
            errors[a].append(s.rate_cmd[i] - s.rate_meas[i])
            peak_cmd[a] = max(peak_cmd[a], abs(s.rate_cmd[i]))

    duration_s = max(1e-6, samples[-1].t - samples[0].t)

    # Only score the axes the sequence actually commanded. Averaging in an
    # untouched axis (zero command, zero error) inflates every score.
    ACTIVE_CMD_THRESHOLD = 0.5          # rad/s ~= 29 deg/s
    active = [a for a in axes if peak_cmd[a] >= ACTIVE_CMD_THRESHOLD]
    if not active:
        return {
            "outcome_score": 0.0,
            "aborted": True,
            "abort_reason": ("no axis was commanded above "
                             f"{ACTIVE_CMD_THRESHOLD} rad/s — nothing to score"),
            "n_samples": len(samples),
        }

    rmse = {a: _rmse(errors[a]) for a in axes}
    peak_err = {a: max((abs(e) for e in errors[a]), default=0.0) for a in axes}

    # Relative error makes the score scale-free: a sweep that changes
    # ACRO_RP_RATE changes the commanded rates, and an absolute rad/s
    # threshold would silently reward the slower setting.
    rel_rmse = {a: rmse[a] / peak_cmd[a] for a in active}

    # Deadband for crossing detection, per axis, tied to the command scale.
    osc_hz = {
        a: _oscillation_hz(errors[a], duration_s,
                           deadband=max(0.05, 0.05 * peak_cmd[a]))
        for a in active
    }

    vibe_max = max(max(s.vibe) for s in samples)
    clip_total = max(sum(s.clipping) for s in samples)

    # --- combine into one number -----------------------------------------
    # These constants set what counts as "bad enough to zero out that term".
    # They are judgement calls, not physics. Calibrate them once against a few
    # known-good and known-bad runs, then leave them alone — changing them
    # makes scores incomparable with every run already in the database.
    REL_RMSE_FULL_PENALTY = 0.30    # 30% mean tracking error of commanded rate
    OSC_FULL_PENALTY_HZ = 12.0      # meaningful error sign flips per second
    VIBE_FULL_PENALTY = 60.0        # m/s^2 — ArduPilot's "nearly always a problem"

    track_term = 1.0 - min(1.0, statistics.mean(rel_rmse.values())
                           / REL_RMSE_FULL_PENALTY)
    osc_term = 1.0 - min(1.0, statistics.mean(osc_hz.values())
                         / OSC_FULL_PENALTY_HZ)
    vibe_term = 1.0 - min(1.0, vibe_max / VIBE_FULL_PENALTY)

    outcome = 0.60 * track_term + 0.25 * osc_term + 0.15 * vibe_term
    if clip_total > 0:
        outcome *= 0.5   # accelerometer clipping invalidates the vibration read

    return {
        "outcome_score": round(max(0.0, min(1.0, outcome)), 4),
        "aborted": False,
        "abort_reason": "",
        "n_samples": len(samples),
        "duration_s": round(duration_s, 2),
        "axes_scored": active,
        "rmse_rad_s": {a: round(rmse[a], 5) for a in axes},
        "rel_rmse": {a: round(rel_rmse[a], 4) for a in active},
        "peak_error_rad_s": {a: round(peak_err[a], 5) for a in axes},
        "peak_cmd_rad_s": {a: round(peak_cmd[a], 4) for a in axes},
        "oscillation_hz": {a: round(osc_hz[a], 2) for a in active},
        "vibration_max_m_s2": round(vibe_max, 3),
        "clipping_total": clip_total,
        "terms": {
            "tracking": round(track_term, 4),
            "oscillation": round(osc_term, 4),
            "vibration": round(vibe_term, 4),
        },
    }


# --------------------------------------------------------------------------
# Persistence
# --------------------------------------------------------------------------
# Results land in the ATLAS repository's own sim_runs table, created by
# acro_bridge.migrate(), so simulated evidence sits beside hardware evidence
# and the prediction model can draw on both. Run standalone against a path
# with no ATLAS schema and the table is created on demand.
def open_db(path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    try:
        import acro_bridge
        acro_bridge.migrate(conn)
    except ImportError:
        conn.executescript(
            "CREATE TABLE IF NOT EXISTS sim_runs ("
            " id INTEGER PRIMARY KEY AUTOINCREMENT, aircraft_id INTEGER,"
            " run_group TEXT NOT NULL, started_utc TEXT NOT NULL,"
            " params_json TEXT NOT NULL, score_json TEXT NOT NULL,"
            " outcome_score REAL, aborted INTEGER NOT NULL DEFAULT 0,"
            " abort_reason TEXT);"
        )
        conn.commit()
    return conn


def store_run(conn, run_group, params, sequence, score, aircraft_id=None) -> int:
    # `sequence` is folded into score_json rather than given its own column, so
    # the row shape matches the schema ATLAS already owns.
    payload = dict(score)
    payload["sequence"] = [asdict(m) for m in sequence]
    cur = conn.execute(
        "INSERT INTO sim_runs (aircraft_id, run_group, started_utc, params_json, "
        "score_json, outcome_score, aborted, abort_reason) VALUES (?,?,?,?,?,?,?,?)",
        (
            aircraft_id,
            run_group,
            time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            json.dumps(params, sort_keys=True),
            json.dumps(payload),
            score["outcome_score"],
            int(score["aborted"]),
            score["abort_reason"],
        ),
    )
    conn.commit()
    return cur.lastrowid


# --------------------------------------------------------------------------
# The iteration loop
# --------------------------------------------------------------------------
def iterate(candidates: list[dict[str, float]],
            cfg: HarnessConfig,
            sequence: list[Maneuver] | None = None,
            run_group: str | None = None,
            aircraft_id: int | None = None,
            progress=None,
            record_tlog: bool = True,
            download_bin: bool = False,
            log_dir: str | None = None,
            files_dir: str | None = None) -> list[dict[str, Any]]:
    sequence = sequence or DEFAULT_SEQUENCE
    run_group = run_group or time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    say = progress or (lambda _m: None)
    log_dir = Path(log_dir or "sim_logs")
    if record_tlog or download_bin:
        log_dir.mkdir(parents=True, exist_ok=True)
    try:
        import sim_logs
    except ImportError:
        sim_logs = None
        if record_tlog or download_bin:
            say("sim_logs.py not found — flights will be scored but not logged")
            record_tlog = download_bin = False

    # Reject everything out of bounds up front, before burning flight time.
    for idx, cand in enumerate(candidates):
        ok, problems = validate_param_set(cand)
        if not ok:
            raise ValueError(f"candidate {idx} rejected:\n  " + "\n  ".join(problems))

    conn = open_db(cfg.db_path)
    harness = SimHarness(cfg)
    harness.connect()
    results: list[dict[str, Any]] = []

    for idx, cand in enumerate(candidates, start=1):
        print(f"\n===== run {idx}/{len(candidates)} =====")
        print("  " + ", ".join(f"{k}={v}" for k, v in sorted(cand.items())))
        say(f"Run {idx}/{len(candidates)}: "
            + ", ".join(f"{k}={v:g}" for k, v in sorted(cand.items())
                        if k in ("ATC_RAT_RLL_P", "ATC_RAT_PIT_P", "ACRO_RP_RATE"))[:90])
        score: dict[str, Any]
        tlog_path = bin_path = None
        tlog = None
        run_tag = f"{run_group}_run{idx:02d}"
        try:
            harness.apply_sim_setup()
            harness.apply_param_set(cand)
            if record_tlog:
                # Start the telemetry log AFTER the parameters are set, so the
                # log covers the flight rather than the setup chatter.
                tlog_path = log_dir / f"{run_tag}.tlog"
                tlog = sim_logs.TelemetryLog(harness.master, tlog_path)
                tlog.start()
            harness.takeoff(cfg.test_altitude_m)
            samples, abort = harness.fly_sequence(sequence)
            score = score_flight(samples, abort)
        except (TimeoutError, RuntimeError, ValueError) as exc:
            print(f"[run] failed: {exc}")
            score = {"outcome_score": 0.0, "aborted": True,
                     "abort_reason": str(exc), "n_samples": 0}
        finally:
            try:
                harness.land_and_disarm()
            except Exception as exc:          # never let cleanup kill the sweep
                print(f"[land] cleanup failed: {exc}")
            if tlog is not None:
                size = tlog.stop()
                print(f"[log] {tlog_path.name}: {size / 1024:.1f} KiB")
                if size == 0:
                    tlog_path = None

        # The dataflash log must be fetched BEFORE the reboot below, or the
        # autopilot closes the log and starts a new one for the next run.
        if download_bin:
            try:
                log_id = sim_logs.newest_log_id(harness.master)
                if log_id is None:
                    say("autopilot reports no dataflash logs — is LOG_BITMASK set?")
                else:
                    bin_path = sim_logs.download_log(
                        harness.master, log_id, log_dir / f"{run_tag}.bin",
                        progress=say)
            except (TimeoutError, ValueError, RuntimeError) as exc:
                print(f"[log] dataflash download failed: {exc}")
                say(f"dataflash download failed: {str(exc).splitlines()[0]}")
                bin_path = None

        run_id = store_run(conn, run_group, cand, sequence, score, aircraft_id)
        results.append({"run_id": run_id, "params": cand, "score": score,
                        "tlog": str(tlog_path) if tlog_path else None,
                        "bin": str(bin_path) if bin_path else None})
        print(f"[score] {score['outcome_score']:.4f}"
              + (f"  ({score['abort_reason']})" if score["aborted"] else ""))
        say(f"Run {idx}/{len(candidates)} scored {score['outcome_score']:.4f}")

        # File it as repository evidence, clearly marked as simulated.
        if aircraft_id is not None and sim_logs is not None and files_dir:
            try:
                filed = sim_logs.ingest_sim_flight(
                    conn, aircraft_id, cand, score,
                    files_dir=files_dir, tlog_path=tlog_path, bin_path=bin_path,
                    run_group=run_group, firmware="SITL", sim_run_id=run_id)
                results[-1]["filed"] = filed
                if filed.get("filed"):
                    say(f"Filed run {idx} as flight evidence "
                        f"({len(filed['flight_test_ids'])} log file(s))")
            except Exception as exc:                         # noqa: BLE001
                print(f"[file] could not file run {idx}: {exc}")

        if cfg.reboot_between_runs and idx < len(candidates):
            try:
                harness.reboot()
            except Exception as exc:
                print(f"[reset] reboot failed: {exc} — reconnecting")
                harness.connect()

    conn.close()

    ranked = sorted(results, key=lambda r: r["score"]["outcome_score"], reverse=True)
    print(f"\n===== ranking (group {run_group}) =====")
    for rank, r in enumerate(ranked, start=1):
        label = ", ".join(f"{k}={v}" for k, v in sorted(r["params"].items()))
        print(f"{rank:>2}. {r['score']['outcome_score']:.4f}  {label}")
    return ranked


# --------------------------------------------------------------------------
# Candidate generation
# --------------------------------------------------------------------------
BASELINE: dict[str, float] = {
    # From the real TBS Lucid H7 / Copter 4.7 parameter file.
    "ATC_RAT_RLL_P": 0.03205,
    "ATC_RAT_RLL_I": 0.03205,
    "ATC_RAT_RLL_D": 0.001393,
    "ATC_RAT_PIT_P": 0.03205,
    "ATC_RAT_PIT_I": 0.03205,
    "ATC_RAT_PIT_D": 0.001465,
    "ATC_RAT_YAW_P": 0.229,
    "ATC_RAT_YAW_I": 0.0229,
    "ACRO_RP_RATE": 400.0,
    "ACRO_Y_RATE": 300.0,
    "INS_GYRO_FILTER": 100.0,
}


def sweep(param: str, low: float, high: float, steps: int,
          baseline: dict[str, float] | None = None) -> list[dict[str, float]]:
    """One parameter varied across a range, everything else held at baseline."""
    base = dict(baseline or BASELINE)
    if steps < 2:
        raise ValueError("steps must be at least 2")
    out = []
    for i in range(steps):
        value = low + (high - low) * i / (steps - 1)
        cand = dict(base)
        cand[param] = round(value, 6)
        # Roll/pitch I tracks P on this airframe; keep the relationship intact
        # so the sweep changes one thing instead of two.
        if param == "ATC_RAT_RLL_P":
            cand["ATC_RAT_RLL_I"] = cand[param]
        elif param == "ATC_RAT_PIT_P":
            cand["ATC_RAT_PIT_I"] = cand[param]
        elif param == "ATC_RAT_YAW_P":
            cand["ATC_RAT_YAW_I"] = round(cand[param] / 10.0, 6)
        out.append(cand)
    return out


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Iterate Acro flight simulations over ArduPilot parameter sets.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Start SITL first:\n"
               "  sim_vehicle.py -v ArduCopter -f quad --out=udp:127.0.0.1:14550",
    )
    ap.add_argument("--connection", default=HarnessConfig.connection,
                    help="MAVLink connection string (default: %(default)s)")
    ap.add_argument("--altitude", type=float, default=HarnessConfig.test_altitude_m,
                    help="test altitude in metres (default: %(default)s)")
    ap.add_argument("--db", default=HarnessConfig.db_path,
                    help="SQLite results file (default: %(default)s). Point this "
                         "at atlas_repository.db to feed the prediction model.")
    ap.add_argument("--aircraft-id", type=int, default=None,
                    help="link these runs to an aircraft in the ATLAS repository")
    ap.add_argument("--no-reboot", action="store_true",
                    help="skip the autopilot reboot between runs (faster, less clean)")
    ap.add_argument("--group", default=None, help="label for this sweep")
    ap.add_argument("--no-tlog", action="store_true",
                    help="do not record a telemetry log per run")
    ap.add_argument("--download-bin", action="store_true",
                    help="also pull each run's dataflash .bin off the autopilot "
                         "over MAVLink (slow; needed when SITL is on another "
                         "computer, otherwise just copy from its logs/ folder)")
    ap.add_argument("--log-dir", default="sim_logs",
                    help="where to write flight logs (default: %(default)s)")
    ap.add_argument("--files-dir", default=None,
                    help="repository_files directory; set it to file each run as "
                         "flight evidence (needs --aircraft-id)")

    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sweep", nargs=4, metavar=("PARAM", "LOW", "HIGH", "STEPS"),
                     help="vary one parameter across a range")
    src.add_argument("--candidates", metavar="FILE",
                     help="JSON file: a list of {param: value} objects")
    src.add_argument("--baseline-only", action="store_true",
                     help="fly the baseline parameter set once")

    args = ap.parse_args(argv)

    cfg = HarnessConfig(
        connection=args.connection,
        test_altitude_m=args.altitude,
        db_path=args.db,
        reboot_between_runs=not args.no_reboot,
    )

    if args.sweep:
        param, low, high, steps = args.sweep
        candidates = sweep(param, float(low), float(high), int(steps))
    elif args.candidates:
        with open(args.candidates) as fh:
            candidates = json.load(fh)
        if not isinstance(candidates, list):
            ap.error("--candidates file must contain a JSON list of objects")
    else:
        candidates = [dict(BASELINE)]

    print(f"{len(candidates)} candidate parameter set(s) to fly")
    try:
        iterate(candidates, cfg, run_group=args.group,
                aircraft_id=args.aircraft_id,
                record_tlog=not args.no_tlog,
                download_bin=args.download_bin,
                log_dir=args.log_dir,
                files_dir=args.files_dir)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
