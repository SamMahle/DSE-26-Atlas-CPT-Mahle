"""
Deterministic safety bounds for ArduPilot parameters relevant to Acro mode.

Intentionally simple and explicit rather than clever - the whole point of
this layer is that its behavior is predictable and auditable, unlike the
LLM's suggestions. Expand ACRO_PARAM_RANGES as you learn more about what's
safe for your specific airframes. Anything not listed here is rejected by
default (fail closed).
"""

# name -> (min, max) safe range. These are conservative starting points,
# not tuned values - adjust based on your own airframe testing.
ACRO_PARAM_RANGES = {
    # Rate controller PIDs (shared by every flight mode, tuned in Acro).
    # P/I floor widened to 0.02 (from 0.05) after real-world data from a
    # small/light build showed working values of ~0.032 - the original
    # floor was eyeballed for larger frames and would have wrongly
    # rejected a real, currently-flying small-quad configuration.
    "ATC_RAT_RLL_P": (0.02, 0.35),
    "ATC_RAT_RLL_I": (0.02, 0.35),
    "ATC_RAT_RLL_D": (0.001, 0.02),
    "ATC_RAT_PIT_P": (0.02, 0.35),
    "ATC_RAT_PIT_I": (0.02, 0.35),
    "ATC_RAT_PIT_D": (0.001, 0.02),
    "ATC_RAT_YAW_P": (0.1, 0.6),
    "ATC_RAT_YAW_I": (0.01, 0.1),
    "ATC_RAT_YAW_D": (0.0, 0.02),
    # Acro-specific stick feel. ACRO_RP_P was dropped from current
    # ArduPilot firmware - Copter 4.7 uses ACRO_RP_RATE/ACRO_Y_RATE
    # (max rotation rate at full stick, deg/s) instead.
    "ACRO_RP_RATE": (180.0, 1080.0),
    "ACRO_Y_RATE": (90.0, 1080.0),
    "ACRO_Y_EXPO": (0.0, 1.0),
    "ACRO_RP_EXPO": (0.0, 1.0),
    # Gyro filtering. Ceiling raised slightly (100 -> 120) since a real
    # working value sat exactly on the old boundary.
    "INS_GYRO_FILTER": (10.0, 120.0),
}


def is_safe_value(param_name: str, value: float) -> bool:
    """True if value is within the known-safe range for param_name."""
    bounds = ACRO_PARAM_RANGES.get(param_name)
    if bounds is None:
        return False
    low, high = bounds
    return low <= value <= high


def validate_param_set(params: dict) -> tuple[bool, list[str]]:
    """Validate a whole {param_name: value} dict. Returns (all_valid, problems)."""
    problems = []
    for name, value in params.items():
        if not is_safe_value(name, value):
            problems.append(f"{name}={value} is out of range or unrecognized")
    return (len(problems) == 0, problems)


# ---------------------------------------------------------------------------
# Structured checking, for interfaces that need to explain the verdict rather
# than just print it. validate_param_set() above is kept unchanged so existing
# callers keep working.
# ---------------------------------------------------------------------------

#: Parameters not in ACRO_PARAM_RANGES that this tool may legitimately read and
#: display but must never recommend or write. Keeping them out of the ranges
#: table is what makes the fail-closed rule meaningful: an unknown name is
#: rejected, and these are explicitly unknown *on purpose*.
NEVER_WRITE_PREFIXES = (
    "SERVO", "MOT_", "FRAME_", "BRD_", "RC", "ARMING_", "FS_",
    "BATT_", "GPS_", "COMPASS_", "EK3_", "EK2_", "SIM_",
)

OK = "ok"
OUT_OF_RANGE = "out_of_range"
NOT_TUNABLE = "not_tunable"
NOT_A_NUMBER = "not_a_number"


def check_param(name: str, value) -> dict:
    """Check one parameter and say exactly why it passed or failed.

    Returns a dict with: name, value, verdict, low, high, message.
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return {"name": name, "value": value, "verdict": NOT_A_NUMBER,
                "low": None, "high": None,
                "message": f"{name}: {value!r} is not a number"}

    bounds = ACRO_PARAM_RANGES.get(name)
    if bounds is None:
        why = "not an approved Acro tuning parameter"
        if name.startswith(NEVER_WRITE_PREFIXES):
            why = ("hardware/safety parameter — this tool never writes it "
                   "(set it in Mission Planner)")
        return {"name": name, "value": v, "verdict": NOT_TUNABLE,
                "low": None, "high": None, "message": f"{name}: {why}"}

    low, high = bounds
    if low <= v <= high:
        return {"name": name, "value": v, "verdict": OK,
                "low": low, "high": high, "message": f"{name}={v:g} in range"}
    return {"name": name, "value": v, "verdict": OUT_OF_RANGE,
            "low": low, "high": high,
            "message": f"{name}={v:g} outside approved range [{low:g}, {high:g}]"}


def check_param_set(params: dict) -> dict:
    """Check a whole set. Returns {'all_valid', 'results', 'writable', 'blocked'}.

    'writable' is the subset safe to send to a flight controller. Everything
    else lands in 'blocked' with a reason — nothing is silently dropped.
    """
    results = [check_param(n, v) for n, v in params.items()]
    writable = {r["name"]: r["value"] for r in results if r["verdict"] == OK}
    blocked = [r for r in results if r["verdict"] != OK]
    return {
        "all_valid": not blocked,
        "results": results,
        "writable": writable,
        "blocked": blocked,
    }


def clamp_to_range(name: str, value: float) -> tuple[float, bool]:
    """Clamp a value into its approved range. Returns (value, was_clamped).

    Used only where a model's output is deliberately pulled back to the
    boundary instead of rejected. Unknown parameters are never clamped — an
    unknown name has no boundary to clamp to, so it stays rejected.
    """
    bounds = ACRO_PARAM_RANGES.get(name)
    if bounds is None:
        raise KeyError(f"{name} is not an approved Acro tuning parameter")
    low, high = bounds
    v = float(value)
    if v < low:
        return low, True
    if v > high:
        return high, True
    return v, False
