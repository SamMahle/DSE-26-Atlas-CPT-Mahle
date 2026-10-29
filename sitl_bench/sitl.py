"""Launch ArduPilot SITL and fly the standard profile over MAVLink.

Nothing in ArduPilot is modified: everything is driven from here through
command-line options, parameter files and MAVLink. All waits use simulated
time (time_boot_ms), so results don't depend on speedup or CPU load.
"""
import glob
import math
import os
import shutil
import subprocess
import time

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

import flight_profile as fp

L = mavutil.mavlink


class FlightError(RuntimeError):
    pass


# ---------------------------------------------------------------- SITL process

def launch(binary, frame, model_file, defaults, instance, rundir, speedup):
    """Start the SITL binary directly (no sim_vehicle.py, no MAVProxy).
    `defaults` is a list of .parm files; later files override earlier ones.
    The model and parm files are copied into rundir and passed by bare name,
    because SITL splits paths containing spaces."""
    if not os.path.exists(binary):
        raise FileNotFoundError(f"SITL binary not found: {binary}")
    names = []
    for src in [model_file] + defaults:
        name = os.path.basename(src)
        if os.path.abspath(src) != os.path.abspath(os.path.join(rundir, name)):
            shutil.copyfile(src, os.path.join(rundir, name))
        names.append(name)
    home = ",".join(str(v) for v in fp.HOME)
    cmd = [binary, "--model", f"{frame}:{names[0]}", "--speedup", str(speedup),
           "-I", str(instance), "-w", "--home", home,
           "--defaults", ",".join(names[1:])]
    out = open(os.path.join(rundir, "sitl.log"), "w")
    return subprocess.Popen(cmd, cwd=rundir, stdout=out, stderr=subprocess.STDOUT)


def stop(proc):
    proc.terminate()
    try:
        proc.wait(10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def newest_log(rundir):
    logs = glob.glob(os.path.join(rundir, "logs", "*.BIN"))
    return max(logs, key=os.path.getmtime) if logs else None


# ---------------------------------------------------------------- MAVLink link

class Vehicle:
    def __init__(self, port, timeout=60):
        deadline = time.time() + timeout
        self.m = None
        while time.time() < deadline:
            try:
                m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255)
                if m.wait_heartbeat(timeout=5):
                    self.m = m
                    break
            except OSError:
                time.sleep(1)
        if self.m is None:
            raise FlightError(f"no heartbeat on port {port}")
        self.sim_t = 0.0
        self._last_clock = time.time()
        self.statustext = []
        self._request_streams()

    def _request_streams(self):
        m = self.m
        # Without MAVProxy nobody else asks SITL for telemetry, so ask explicitly.
        m.mav.request_data_stream_send(m.target_system, m.target_component,
                                       L.MAV_DATA_STREAM_ALL, 10, 1)
        self.command(L.MAV_CMD_SET_MESSAGE_INTERVAL, L.MAVLINK_MSG_ID_ATTITUDE, 20000)  # 50 Hz

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     cmd, 0, *p)

    def recv(self, types=None, timeout=10):
        """Receive one message, keeping track of sim time and status text."""
        msg = self.m.recv_match(type=types, blocking=True, timeout=timeout)
        stalled = time.time() - self._last_clock
        if stalled > 30:
            raise FlightError("no timestamped telemetry for 30 s")
        if stalled > 5:
            self._request_streams()
        if msg is None:
            return None
        if hasattr(msg, "time_boot_ms"):
            self.sim_t = msg.time_boot_ms / 1000.0
            self._last_clock = time.time()
        if msg.get_type() == "STATUSTEXT":
            self.statustext.append((round(self.sim_t, 2), msg.text))
        return msg

    def sim_sleep(self, secs, on_tick=None):
        t_end = self.sim_t + secs
        while self.sim_t < t_end:
            msg = self.recv(["ATTITUDE", "STATUSTEXT", "HEARTBEAT"])
            if msg is None:
                raise FlightError("telemetry stopped")
            if on_tick and msg.get_type() == "ATTITUDE":
                on_tick(msg)

    def wait_for(self, predicate, timeout_sim, what):
        t_end = self.sim_t + timeout_sim
        while self.sim_t < t_end:
            msg = self.recv()
            if msg is None:
                raise FlightError(f"telemetry stopped waiting for {what}")
            if predicate(msg):
                return msg
        raise FlightError(f"timed out waiting for {what}")

    def set_mode(self, mode, timeout_sim=10):
        self.m.set_mode(mode)
        self.wait_for(lambda _: self.m.flightmode == mode, timeout_sim, f"mode {mode}")

    # ---------------------------------------------------------- parameters

    def get_param(self, name, timeout=5):
        m = self.m
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = self.recv("PARAM_VALUE", timeout=1)
            if msg and msg.param_id == name:
                return msg.param_value
        return None

    # ---------------------------------------------------------- mission

    def upload_mission(self, items):
        m = self.m
        m.mav.mission_count_send(m.target_system, m.target_component, len(items))
        while True:
            msg = self.recv(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], timeout=10)
            if msg is None:
                raise FlightError("mission upload timed out")
            if msg.get_type() == "MISSION_ACK":
                if msg.type != L.MAV_MISSION_ACCEPTED:
                    raise FlightError(f"mission rejected (code {msg.type})")
                return
            frame, cmd, p1, p2, p3, p4, lat, lon, alt = items[msg.seq]
            m.mav.mission_item_int_send(
                m.target_system, m.target_component, msg.seq, frame, cmd,
                0, 1, p1, p2, p3, p4, int(lat * 1e7), int(lon * 1e7), alt)


# ---------------------------------------------------------------- the flight

def arm_in_auto(v, timeout_sim=180):
    """Retry until pre-arm checks pass (EKF/GPS take a while after boot).
    AUTO_OPTIONS=3 lets Copter arm in AUTO and start the mission immediately."""
    t_end = v.sim_t + timeout_sim
    while not v.m.motors_armed():
        if v.sim_t > t_end:
            raise FlightError("could not arm (pre-arm checks never passed)")
        v.m.set_mode("AUTO")
        v.m.arducopter_arm()
        v.sim_sleep(2)


def fly_mission(v, timeout_sim=600):
    last_seq = len(fp.MISSION)

    def reached_end(msg):
        if msg.get_type() == "HEARTBEAT" and not v.m.motors_armed():
            raise FlightError("disarmed during mission (crash?)")
        return msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq == last_seq

    v.wait_for(reached_end, timeout_sim, "end of mission")
    v.sim_sleep(fp.MISSION[-1][2].get("hold", 0))


def attitude_steps(v):
    v.set_mode("GUIDED")
    v.sim_sleep(3)  # settle in position hold
    msg = v.recv("ATTITUDE")
    yaw = msg.yaw

    def hold(roll_deg, pitch_deg, secs):
        q = QuaternionBase([math.radians(roll_deg), math.radians(pitch_deg), yaw]).q
        m = v.m

        def send(_msg):
            # type_mask 7: ignore body rates. thrust 0.5 = hold altitude (climb-rate mode)
            m.mav.set_attitude_target_send(0, m.target_system, m.target_component,
                                           7, q, 0, 0, 0, 0.5)
        send(None)
        v.sim_sleep(secs, on_tick=send)

    for axis, deg in fp.ATTITUDE_STEPS:
        if axis == "roll":
            hold(deg, 0, fp.STEP_HOLD_S)
        else:
            hold(0, deg, fp.STEP_HOLD_S)
        hold(0, 0, fp.STEP_REST_S)


def land(v, timeout_sim=120):
    v.set_mode("LAND")
    v.wait_for(lambda msg: msg.get_type() == "HEARTBEAT" and not v.m.motors_armed(),
               timeout_sim, "landing/disarm")


def fly_standard(v, overrides):
    """Fly the full standard profile. Returns parameter values read back from the vehicle."""
    # SITL accepts the TCP connection before it has finished booting
    v.wait_for(lambda msg: msg.get_type() == "STATUSTEXT" and "ArduPilot Ready" in msg.text,
               60, "ArduPilot Ready")
    readback = {k: v.get_param(k) for k in overrides}
    v.upload_mission(fp.mission_items())
    arm_in_auto(v)
    fly_mission(v)
    attitude_steps(v)
    land(v)
    return readback
