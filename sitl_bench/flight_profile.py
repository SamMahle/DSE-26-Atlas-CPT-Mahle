"""The standard test flight.

Every run flies exactly this profile, so logs are comparable run to run.
Bump PROFILE_VERSION whenever anything in this file changes; results from
different profile versions must not be compared against each other.
"""
import math

from pymavlink import mavutil

PROFILE_VERSION = "5in-std-v1"

# Fixed home: lat, lon, alt AMSL (m), heading (deg). Same field every run.
HOME = (-35.363261, 149.165230, 584.0, 0.0)

ALT = 10.0      # standard test altitude (m, relative to home)
CRUISE = 10.0   # m/s
FAST = 15.0     # m/s for the speed run

# Phase A: autonomous mission (flown onboard in AUTO, so timing is deterministic).
# (segment name, kind, args). Consecutive items with the same name form one segment.
#   takeoff(alt)  loiter(n, e, alt, secs)  wp(n, e, alt, hold)  speed(mps)  yaw(deg)
# n/e are metres north/east of home.
MISSION = [
    ("takeoff",    "takeoff", dict(alt=ALT)),
    ("hover",      "loiter",  dict(n=0,   e=0,  alt=ALT, secs=10)),
    ("step_north", "wp",      dict(n=30,  e=0,  alt=ALT, hold=5)),
    ("box",        "wp",      dict(n=30,  e=30, alt=ALT, hold=0)),
    ("box",        "wp",      dict(n=0,   e=30, alt=ALT, hold=0)),
    ("box",        "wp",      dict(n=0,   e=0,  alt=ALT, hold=3)),
    ("climb",      "wp",      dict(n=0,   e=0,  alt=20,  hold=5)),
    ("descend",    "wp",      dict(n=0,   e=0,  alt=ALT, hold=5)),
    ("yaw_90",     "yaw",     dict(deg=90)),
    ("yaw_90",     "loiter",  dict(n=0,   e=0,  alt=ALT, secs=6)),
    ("yaw_0",      "yaw",     dict(deg=0)),
    ("yaw_0",      "loiter",  dict(n=0,   e=0,  alt=ALT, secs=6)),
    ("speed_run",  "speed",   dict(mps=FAST)),
    ("speed_run",  "wp",      dict(n=100, e=0,  alt=ALT, hold=0)),
    ("speed_run",  "wp",      dict(n=0,   e=0,  alt=ALT, hold=3)),
    ("settle",     "speed",   dict(mps=CRUISE)),
    ("settle",     "wp",      dict(n=0,   e=0,  alt=ALT, hold=3)),
]

# Box corners in flight order, for cross-track error.
BOX_PATH = [(30, 0), (30, 30), (0, 30), (0, 0)]

# Phase B: attitude step tests, flown in GUIDED from Python after the mission.
# Metrics are measured against the desired attitude in the onboard log,
# so Python's timing jitter doesn't affect the results.
ATTITUDE_STEPS = [("roll", 10), ("roll", -10), ("pitch", 10), ("pitch", -10)]  # deg
STEP_HOLD_S = 1.5   # sim seconds held at the commanded angle
STEP_REST_S = 2.0   # sim seconds level between steps

M_PER_DEG_LAT = 111319.5


def ne_to_latlon(n, e):
    lat = HOME[0] + n / M_PER_DEG_LAT
    lon = HOME[1] + e / (M_PER_DEG_LAT * math.cos(math.radians(HOME[0])))
    return lat, lon


def latlon_to_ne(lat, lon):
    n = (lat - HOME[0]) * M_PER_DEG_LAT
    e = (lon - HOME[1]) * M_PER_DEG_LAT * math.cos(math.radians(HOME[0]))
    return n, e


def mission_items():
    """MAVLink mission items (seq 0 = home). Each item is
    (frame, command, p1, p2, p3, p4, lat, lon, alt)."""
    L = mavutil.mavlink
    rel = L.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    items = [(L.MAV_FRAME_GLOBAL_INT, L.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, HOME[0], HOME[1], 0)]
    for _, kind, a in MISSION:
        if kind == "takeoff":
            items.append((rel, L.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, HOME[0], HOME[1], a["alt"]))
        elif kind == "loiter":
            lat, lon = ne_to_latlon(a["n"], a["e"])
            items.append((rel, L.MAV_CMD_NAV_LOITER_TIME, a["secs"], 0, 0, 0, lat, lon, a["alt"]))
        elif kind == "wp":
            lat, lon = ne_to_latlon(a["n"], a["e"])
            items.append((rel, L.MAV_CMD_NAV_WAYPOINT, a["hold"], 0, 0, 0, lat, lon, a["alt"]))
        elif kind == "speed":
            items.append((L.MAV_FRAME_MISSION, L.MAV_CMD_DO_CHANGE_SPEED, 1, a["mps"], -1, 0, 0, 0, 0))
        elif kind == "yaw":
            # absolute heading, default rate, shortest direction
            items.append((L.MAV_FRAME_MISSION, L.MAV_CMD_CONDITION_YAW, a["deg"], 0, 0, 0, 0, 0, 0))
        else:
            raise ValueError(kind)
    return items


def segment_names():
    """Mission seq number -> segment name (matches CMD.CNum in the log)."""
    return {i + 1: name for i, (name, _, _) in enumerate(MISSION)}
