#!/usr/bin/env python3
"""
check_link.py — diagnose a MAVLink connection without involving the interface.

Run this on the computer where ATLAS runs. It reports this machine's network
addresses, tells you the exact line to launch SITL with, then listens and says
what it heard.

    python check_link.py                         # listen on all interfaces, port 14550
    python check_link.py --port 14551            # a different port
    python check_link.py --conn udp:127.0.0.1:14550   # test one specific string
    python check_link.py --probe 192.168.1.42    # can this machine reach that one?

Why a UDP timeout is ambiguous
------------------------------
A listener that receives nothing looks the same whether the simulator is off,
sending to the wrong address, blocked by a firewall, or on a network that will
not route between hosts. This script separates those cases.
"""
from __future__ import annotations

import argparse
from typing import Callable

PROGRESS = Callable[..., None]

import socket
import sys
import time

# Imported by the interface as well as run from a terminal, so a missing
# pymavlink must NOT kill the process — exiting here took the whole GUI down
# with it. main() checks and reports; diagnose() raises for the caller.
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

NO_PYMAVLINK = ("pymavlink is not installed, so the link cannot be tested.\n\n"
                "    pip install pymavlink")


def _require_mavutil():
    if mavutil is None:
        raise RuntimeError(NO_PYMAVLINK)
    return mavutil


def local_addresses() -> list[str]:
    """Every non-loopback IPv4 address on this machine."""
    found = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                found.add(ip)
    except socket.gaierror:
        pass
    # Routing trick: no packet is sent, but the OS picks the interface it would
    # use to reach the internet, which is normally the one the other machine
    # should target.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        if not ip.startswith("127."):
            found.add(ip)
    except OSError:
        pass
    finally:
        s.close()
    return sorted(found)


def port_in_use(port: int) -> str | None:
    """Is something already bound to this UDP port? Only one program can be."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", port))
        return None
    except OSError as exc:
        return str(exc)
    finally:
        s.close()


def probe(host: str, port: int, timeout: float = 3.0) -> None:
    """Send a heartbeat TO a host, to test the outbound direction."""
    print(f"\nProbing {host}:{port} — sending heartbeats for {timeout:.0f}s")
    try:
        m = mavutil.mavlink_connection(f"udpout:{host}:{port}")
    except Exception as exc:
        print(f"  could not open: {exc}")
        return
    deadline = time.time() + timeout
    sent = 0
    while time.time() < deadline:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                             mavutil.mavlink.MAV_STATE_ACTIVE)
        sent += 1
        time.sleep(0.25)
    m.close()
    print(f"  sent {sent} heartbeats. UDP gives no delivery confirmation, so this "
          "only proves\n  this machine could address that host — not that it arrived.")


def listen(conn: str, seconds: float) -> bool:
    print(f"\nListening on {conn} for {seconds:.0f}s …")
    if "127.0.0.1" in conn or "localhost" in conn:
        print("  NOTE: 127.0.0.1 is loopback. This will only ever hear a simulator")
        print("        running on THIS computer, never one on another machine.")
    try:
        m = mavutil.mavlink_connection(conn)
    except Exception as exc:
        print(f"  could not bind: {exc}")
        print("  Something else is probably holding that port.")
        return False

    deadline = time.time() + seconds
    seen: dict[tuple[int, int], int] = {}
    kinds: dict[str, int] = {}
    first_hb = None
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=max(0.2, deadline - time.time()))
        if msg is None:
            continue
        kinds[msg.get_type()] = kinds.get(msg.get_type(), 0) + 1
        key = (msg.get_srcSystem(), msg.get_srcComponent())
        seen[key] = seen.get(key, 0) + 1
        if msg.get_type() == "HEARTBEAT" and first_hb is None:
            first_hb = msg
            print(f"  HEARTBEAT from system {msg.get_srcSystem()} "
                  f"component {msg.get_srcComponent()}")
            print(f"    autopilot={msg.autopilot}  type={msg.type}  "
                  f"armed={bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")
    m.close()

    if not seen:
        print("  Nothing received.")
        return False
    print(f"  Received {sum(seen.values())} messages from {len(seen)} source(s).")
    top = sorted(kinds.items(), key=lambda kv: -kv[1])[:8]
    print("  Message types: " + ", ".join(f"{k}×{v}" for k, v in top))
    if first_hb is None:
        print("  But NO HEARTBEAT. Traffic is arriving from something that is not "
              "an autopilot,\n  or the stream is partial — check the port number.")
        return False
    return True


def diagnose(port: int = 14550, conn: str | None = None,
             seconds: float = 6.0, emit: PROGRESS | None = None) -> dict:
    """Run the whole check and return a structured report.

    Used both by main() below and by the Flight Controller tab, so the
    diagnostic is reachable from the interface where the failure is actually
    seen, not only from a terminal.
    """
    _require_mavutil()
    out: list[str] = []
    def say(line: str = "") -> None:
        out.append(line)
        if emit: emit(line)

    addrs = local_addresses()
    say("This computer's network addresses:")
    for a in addrs or []:
        say(f"  {a}")
    if not addrs:
        say("  (none found — not on a network?)")
    say("  127.0.0.1    (loopback — this machine only)")
    say()

    busy = port_in_use(port)
    say(f"UDP port {port}: " + ("free" if busy is None else f"ALREADY IN USE — {busy}"))
    if busy:
        say("  Only one program can bind a UDP port. Close Mission Planner,")
        say("  QGroundControl or another copy of ATLAS, or give SITL a second")
        say(f"  output on another port: --out=udp:<this pc>:{port + 1}")
    say()

    if addrs:
        say("On the computer running ArduPilot, launch SITL with:")
        for a in addrs:
            say(f"  sim_vehicle.py -v ArduCopter -f quad --out=udp:{a}:{port}")
        say()

    target = conn or f"udpin:0.0.0.0:{port}"
    say(f"Listening on {target} for {seconds:.0f}s …")
    heard = _listen_quiet(target, seconds, say)
    working = target
    if not heard and not conn:
        say()
        say("Nothing arrived from the network. Checking loopback, in case SITL")
        say("is in fact running on this computer …")
        heard = _listen_quiet(f"udp:127.0.0.1:{port}", min(4.0, seconds), say)
        if heard:
            working = f"udp:127.0.0.1:{port}"
            say()
            say(">> SITL IS on this computer.")
    return {"ok": bool(heard), "lines": out, "connection": working,
            "addresses": addrs, "port_busy": busy}


def _listen_quiet(conn: str, seconds: float, say: PROGRESS) -> bool:
    """listen() but reporting through `say` instead of print()."""
    if "127.0.0.1" in conn or "localhost" in conn:
        say("  NOTE: 127.0.0.1 is loopback — it can only hear a simulator on")
        say("        THIS computer, never one on another machine.")
    try:
        m = mavutil.mavlink_connection(conn)
    except Exception as exc:
        say(f"  could not bind: {exc}")
        say("  Something else is probably holding that port.")
        return False
    deadline = time.time() + seconds
    kinds: dict[str, int] = {}
    first_hb = None
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=max(0.2, deadline - time.time()))
        if msg is None:
            continue
        kinds[msg.get_type()] = kinds.get(msg.get_type(), 0) + 1
        if msg.get_type() == "HEARTBEAT" and first_hb is None:
            first_hb = msg
            say(f"  HEARTBEAT from system {msg.get_srcSystem()} "
                f"component {msg.get_srcComponent()}")
    m.close()
    if not kinds:
        say("  Nothing received.")
        return False
    top = sorted(kinds.items(), key=lambda kv: -kv[1])[:6]
    say("  Message types: " + ", ".join(f"{k}x{v}" for k, v in top))
    if first_hb is None:
        say("  But NO HEARTBEAT — traffic is arriving from something that is not")
        say("  an autopilot, or the port number is wrong.")
        return False
    return True


FAILURE_STEPS = [
    "Is SITL actually running on the other computer?",
    "Does its launch line include --out=udp:<this pc>:14550 with one of the "
    "addresses above? Without --out it only talks to its own machine.",
    "Firewall on THIS computer — Windows Defender blocks inbound UDP to Python "
    "by default. Allow it, or test with it briefly off.",
    "Can the machines reach each other? Ping this computer from the SITL "
    "machine. Campus, guest and VPN networks commonly block host-to-host "
    "traffic, and nothing in ATLAS can work around that.",
    "Still stuck? Run both on one computer — it removes every networking "
    "variable at once.",
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--conn", default=None,
                    help="test one connection string instead of the default sweep")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--probe", default=None, metavar="HOST",
                    help="also send heartbeats TO this host")
    args = ap.parse_args()

    if mavutil is None:
        print(NO_PYMAVLINK)
        return 1

    print("=" * 68)
    print("MAVLink link check")
    print("=" * 68)

    addrs = local_addresses()
    print("\nThis computer's network addresses:")
    if addrs:
        for a in addrs:
            print(f"  {a}")
    else:
        print("  (none found — not on a network?)")
    print("  127.0.0.1    (loopback — this machine only)")

    busy = port_in_use(args.port)
    print(f"\nUDP port {args.port}: ", end="")
    print("free" if busy is None else f"ALREADY IN USE — {busy}")
    if busy:
        print("  Only one program can bind a UDP port. Close Mission Planner,")
        print("  QGroundControl or another copy of ATLAS, or use a second SITL")
        print(f"  output on a different port: --out=udp:<this pc>:{args.port + 1}")

    if addrs:
        print("\nOn the computer running ArduPilot, launch SITL with:")
        for a in addrs:
            print(f"  sim_vehicle.py -v ArduCopter -f quad --out=udp:{a}:{args.port}")
        if len(addrs) > 1:
            print("  (try each address if you are unsure which network is shared)")

    if args.probe:
        probe(args.probe, args.port, timeout=3.0)

    ok = False
    if args.conn:
        ok = listen(args.conn, args.seconds)
    else:
        ok = listen(f"udpin:0.0.0.0:{args.port}", args.seconds)
        if not ok:
            print("\nNothing arrived on any interface. Checking loopback separately,")
            print("in case SITL is in fact running on this computer …")
            ok = listen(f"udp:127.0.0.1:{args.port}", 4.0)
            if ok:
                print("\n  >> SITL IS on this computer. Use  udp:127.0.0.1:14550")

    print("\n" + "=" * 68)
    if ok:
        print("LINK IS WORKING. Use this connection string in ATLAS:")
        print(f"  {args.conn or f'udpin:0.0.0.0:{args.port}'}")
    else:
        print("NO LINK. Work through these in order:")
        print("  1. Is SITL actually running on the other computer?")
        print(f"  2. Does its launch line include --out=udp:<this pc>:{args.port}")
        print("     with one of the addresses listed above? Without --out it only")
        print("     talks to its own machine.")
        print("  3. Firewall on THIS computer — Windows Defender blocks inbound UDP")
        print("     to Python by default. Allow it, or test with it briefly off.")
        print("  4. Can the machines reach each other? Ping this computer from the")
        print("     SITL machine. Campus, guest and VPN networks commonly block")
        print("     host-to-host traffic, and nothing in ATLAS can work around that.")
        print("  5. Still stuck? Run both on one computer — it removes every")
        print("     networking variable at once.")
    print("=" * 68)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
