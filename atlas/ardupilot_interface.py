"""
ardupilot_interface.py — the ONLY module in this application that talks to a
flight controller.

Every other module proposes, scores, retrieves or validates. Nothing else
imports pymavlink, so there is exactly one place to audit the question "can
this program change my aircraft?" — and the answer is: only through
apply_params(), only after safety.check_param_set() passed, and only with a
callback that a human has to satisfy.

pymavlink is imported LAZILY so ATLAS still launches with no pip packages
installed. The flight-controller tab reports the missing dependency instead of
the whole application failing to start.
"""

from __future__ import annotations

import time
from typing import Any, Callable

import safety

DEFAULT_CONNECTIONS = [
    # 127.0.0.1 is LOOPBACK — it means "this same computer". It can only ever
    # reach a simulator running on the machine the interface is running on.
    ("SITL — same computer", "udp:127.0.0.1:14550"),
    # 0.0.0.0 binds every network interface, so telemetry arriving from another
    # machine on the network is accepted. This is the one to use when the
    # simulator runs elsewhere; no IP goes here, because this side only listens.
    ("SITL — another computer", "udpin:0.0.0.0:14550"),
    ("USB serial (Windows)", "COM3"),
    ("USB serial (Linux/Mac)", "/dev/ttyACM0"),
    ("ESP32 WiFi bridge", "tcp:192.168.4.1:5760"),
    ("Telemetry radio (Windows)", "COM4"),
]

#: Baud is ignored for udp:/tcp: connection strings.
DEFAULT_BAUD = 115200


class MavlinkUnavailable(RuntimeError):
    """pymavlink is not installed."""


def _mavutil():
    try:
        from pymavlink import mavutil        # noqa: PLC0415 — deliberately lazy
    except ImportError as exc:
        raise MavlinkUnavailable(
            "pymavlink is not installed. Install it with:\n\n"
            "    pip install pymavlink\n\n"
            "Everything else in ATLAS works without it — only writing to a "
            "flight controller and running simulations need it."
        ) from exc
    return mavutil


class Vehicle:
    """A connection to one autopilot.

    Usage is always: connect -> read -> propose -> validate -> confirm ->
    apply. The class will not let you skip the validation step; apply_params()
    re-checks everything it is given regardless of what the caller claims to
    have done already.
    """

    def __init__(self, connection: str, baud: int = DEFAULT_BAUD):
        self.connection = connection
        self.baud = baud
        self.master: Any = None
        self.mavutil = None

    # ------------------------------------------------------------ connect
    def connect(self, timeout_s: float = 30.0,
                progress: Callable[[str], None] | None = None) -> dict[str, Any]:
        say = progress or (lambda _m: None)
        self.mavutil = _mavutil()
        say(f"Opening {self.connection} …")
        kwargs = {}
        if not self.connection.startswith(("udp:", "tcp:", "udpin:", "udpout:")):
            kwargs["baud"] = self.baud
        self.master = self.mavutil.mavlink_connection(self.connection, **kwargs)

        say("Waiting for heartbeat …")
        hb = self.master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise TimeoutError(self._no_heartbeat_help(timeout_s))

        info = {
            "system_id": self.master.target_system,
            "component_id": self.master.target_component,
            "autopilot": hb.autopilot,
            "vehicle_type": hb.type,
            "armed": bool(hb.base_mode &
                          self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
        }
        say(f"Connected to system {info['system_id']}.")
        for name in ("FRAME_CLASS", "FRAME_TYPE"):
            try:
                info[name] = self.get_param(name, timeout_s=4.0)
            except (TimeoutError, RuntimeError):
                info[name] = None
        return info

    def _no_heartbeat_help(self, timeout_s: float) -> str:
        """Explain a silent timeout in terms of the connection string used.

        A UDP listener that receives nothing looks identical whether the
        simulator is off, on another machine, or behind a firewall — so the
        message has to name the likely cause rather than list every cause.
        """
        conn = self.connection
        head = f"No heartbeat from {conn} within {timeout_s:.0f}s.\n\n"

        if "127.0.0.1" in conn or "localhost" in conn:
            return head + (
                "127.0.0.1 is LOOPBACK — it only reaches a simulator running on "
                "THIS computer.\n\n"
                "If ArduPilot SITL is on another computer:\n"
                "  1. Here, use the connection string:   udpin:0.0.0.0:14550\n"
                "     (no IP address — this side only listens)\n"
                "  2. On the SITL computer, tell it where to send:\n"
                "     sim_vehicle.py -v ArduCopter -f quad --out=udp:THIS_PC_IP:14550\n"
                "     replacing THIS_PC_IP with this computer's address\n"
                "     (Windows: ipconfig    Linux/Mac: hostname -I)\n"
                "  3. Allow Python through the firewall on this computer, and make\n"
                "     sure both machines are on the same network.\n\n"
                "If SITL is on this computer, check that it is actually running and "
                "that its output line reads --out=udp:127.0.0.1:14550."
            )

        if conn.startswith(("udpin:", "udp:")):
            return head + (
                "This side is listening and nothing arrived. In order:\n\n"
                "  1. Is SITL sending HERE? Its launch line needs\n"
                "     --out=udp:THIS_PC_IP:14550 with this computer's real address.\n"
                "     Without --out, sim_vehicle only talks to its own machine.\n"
                "  2. Firewall: Windows Defender blocks inbound UDP to Python by\n"
                "     default. Allow it, or test with the firewall briefly off.\n"
                "  3. Can the machines see each other at all? Ping this computer\n"
                "     from the SITL machine. Campus, guest and VPN networks often\n"
                "     block traffic between hosts, which no setting here can fix.\n"
                "  4. Is something else already holding port 14550 on this computer\n"
                "     (Mission Planner, QGroundControl, another copy of ATLAS)?\n"
                "     Only one program can bind a UDP port. Use a second output on\n"
                "     a different port instead: --out=udp:THIS_PC_IP:14551\n\n"
                "Run check_link.py on this computer to test the link on its own."
            )

        if conn.startswith("tcp:"):
            return head + (
                "Check the device is powered and reachable at that address.\n"
                "For an ESP32 bridge, confirm you are joined to its WiFi network "
                "and that the IP and port match its configuration."
            )

        return head + (
            f"'{conn}' looks like a serial port.\n\n"
            "  1. Is the cable connected and the board powered?\n"
            "  2. Is the port name right? (Windows: Device Manager -> Ports)\n"
            "  3. Is another program holding it — Mission Planner, QGroundControl, "
            "or an open Arduino serial monitor? Close it and try again.\n"
            f"  4. Is the baud rate right? Currently {self.baud}."
        )

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def _require(self):
        if self.master is None:
            raise RuntimeError("not connected — call connect() first")

    # --------------------------------------------------------------- read
    def get_param(self, name: str, timeout_s: float = 6.0) -> float:
        self._require()
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode("ascii"), -1)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True,
                                         timeout=max(0.1, deadline - time.time()))
            if msg is None:
                break
            if msg.param_id.strip("\x00") == name:
                return float(msg.param_value)
        raise TimeoutError(f"no response reading {name}")

    def read_acro_params(self, progress=None) -> dict[str, float | None]:
        """Read back everything this tool considers tunable, for comparison."""
        say = progress or (lambda _m: None)
        out: dict[str, float | None] = {}
        names = list(safety.ACRO_PARAM_RANGES)
        for i, name in enumerate(names, 1):
            say(f"Reading {name} ({i}/{len(names)}) …")
            try:
                out[name] = self.get_param(name)
            except TimeoutError:
                out[name] = None
        return out

    # -------------------------------------------------------------- write
    def set_param(self, name: str, value: float, retries: int = 4) -> float:
        """Write one parameter and confirm it by reading it back.

        ArduPilot transports every parameter as REAL32 regardless of its
        internal type, so REAL32 is correct even for integer parameters.
        """
        self._require()
        mav = self.mavutil.mavlink
        for attempt in range(1, retries + 1):
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                name.encode("ascii"), float(value), mav.MAV_PARAM_TYPE_REAL32)
            deadline = time.time() + 2.5
            while time.time() < deadline:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=True,
                                             timeout=max(0.1, deadline - time.time()))
                if msg is None:
                    break
                if msg.param_id.strip("\x00") != name:
                    continue
                got = float(msg.param_value)
                tol = max(1e-6, abs(float(value)) * 1e-4)
                if abs(got - float(value)) <= tol:
                    return got
                # The autopilot accepted a different value than requested —
                # usually a type coercion. Report it rather than retrying
                # forever on a value it will never store exactly.
                if attempt == retries:
                    raise RuntimeError(
                        f"{name}: wrote {value:g}, autopilot stored {got:g}")
                break
        raise TimeoutError(f"{name}: no confirmation after {retries} attempts")

    def apply_params(self, params: dict[str, float], *,
                     confirm: Callable[[dict], bool],
                     progress: Callable[[str], None] | None = None,
                     ) -> dict[str, Any]:
        """Validate, ask, then write. The only path that changes an aircraft.

        `confirm` receives the validation report and must return True. There is
        no default and no bypass: a caller that does not supply a confirmation
        function cannot write parameters.
        """
        self._require()
        say = progress or (lambda _m: None)

        report = safety.check_param_set(params)
        if report["blocked"]:
            return {"applied": False,
                    "reason": "blocked by safety bounds",
                    "validation": report, "confirmed": {}}
        if not report["writable"]:
            return {"applied": False, "reason": "nothing to write",
                    "validation": report, "confirmed": {}}

        # Refuse to write to an armed aircraft. Rate gains changing mid-flight
        # is exactly the failure this tool exists to prevent.
        if self.is_armed():
            return {"applied": False,
                    "reason": "aircraft is ARMED — disarm before changing tuning",
                    "validation": report, "confirmed": {}}

        if not confirm(report):
            return {"applied": False, "reason": "cancelled by operator",
                    "validation": report, "confirmed": {}}

        confirmed, failures = {}, []
        for i, (name, value) in enumerate(sorted(report["writable"].items()), 1):
            say(f"Writing {name} = {value:g}  ({i}/{len(report['writable'])})")
            try:
                confirmed[name] = self.set_param(name, value)
            except (TimeoutError, RuntimeError) as exc:
                failures.append(str(exc))

        return {
            "applied": bool(confirmed),
            "reason": "" if not failures else "; ".join(failures),
            "validation": report,
            "confirmed": confirmed,
            "failures": failures,
        }

    # ------------------------------------------------------------- status
    def is_armed(self, timeout_s: float = 3.0) -> bool:
        self._require()
        hb = self.master.recv_match(type="HEARTBEAT", blocking=True,
                                    timeout=timeout_s)
        if hb is None:
            return False
        return bool(hb.base_mode &
                    self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def telemetry_snapshot(self, timeout_s: float = 3.0) -> dict[str, Any]:
        """A one-shot read of the values worth showing next to a tune."""
        self._require()
        snap: dict[str, Any] = {}
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(
                type=["ATTITUDE", "VIBRATION", "SYS_STATUS", "HEARTBEAT"],
                blocking=True, timeout=max(0.1, deadline - time.time()))
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "ATTITUDE":
                snap["roll_rate"] = msg.rollspeed
                snap["pitch_rate"] = msg.pitchspeed
                snap["yaw_rate"] = msg.yawspeed
            elif kind == "VIBRATION":
                snap["vibe"] = (msg.vibration_x, msg.vibration_y, msg.vibration_z)
                snap["clipping"] = (msg.clipping_0, msg.clipping_1, msg.clipping_2)
            elif kind == "SYS_STATUS":
                snap["voltage_v"] = msg.voltage_battery / 1000.0
            elif kind == "HEARTBEAT":
                snap["armed"] = bool(
                    msg.base_mode &
                    self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                snap["mode"] = msg.custom_mode
        return snap


def available() -> tuple[bool, str]:
    """Is pymavlink importable? For the GUI to show status without crashing."""
    try:
        _mavutil()
        return True, "pymavlink available"
    except MavlinkUnavailable as exc:
        return False, str(exc)
