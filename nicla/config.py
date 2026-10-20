"""Single knob registry for the Nicla side of the robot.

Every threshold, scale, timeout and enable flag that the link, ToF, IMU and
vision modules read lives here, so a calibration is one edit and no magic number
reaches a runtime module. Defaults are the values documented in
``docs/mega-nicla-protocol.md``; ``apply_push`` is the CONFIG_PUSH receiver and
works on the same live values, which are never persisted across a power cycle.
"""

# UART (protocol 2.4).
UART_ID = 0
UART_BAUD = 115200

# Link cadence, per-tick budgets and the re-sync rule (protocol 5.1-5.5).
HEARTBEAT_PERIOD_MS = 250
HEARTBEAT_TOLERANCE_MS = 50
RX_BYTE_BUDGET = 64
RX_FRAMES_PER_PUMP = 1
TX_FRAMES_PER_PUMP = 2
TX_FRAME_SLOTS = 8
TX_SLOT_BYTES = 14   # widest frame in either direction: GOAL_BLOB (PL 9)
BAD_FRAME_LIMIT = 3
LIVENESS_FRESH_MS = 1000
LIVENESS_LOST_MS = 2500
MEGA_SILENCE_MS = 1000
LINK_READY_TIMEOUT_MS = 5000

# Per-type freshness windows (protocol 5.2). The host mirrors the same table.
TTL_BLOB_MS = 500
TTL_TOF_MS = 1000
TTL_IMU_MS = 1000
TTL_RADAR_MS = 2500
CACHE_TTL_NONE = 0
CACHE_TTL_STICKY = 0xFFFFFFFF

# Radar sweep and the RADAR_RESULT mapping (protocol 4.3). The class bytes are
# the reconciliation the Underboss handed this work package: enclosed -> 1,
# open -> 0. The protocol text lists the opposite order, so one edit here plus
# the matching edit on the host realigns the two ends.
RADAR_CLOSE_MM = 1200
RADAR_SECTOR_COUNT = 18
RADAR_DEFAULT_STEP_DEG = 20
RADAR_ZONE_NO_RESULT = 0xFF
RADAR_CLASS_ENCLOSED = 1
RADAR_CLASS_OPEN = 0

# Payload scales and windows (protocol 4.3).
COORD_MAX = 1000
CONFIDENCE_MAX_PCT = 100
TOF_RANGE_MAX_MM = 4000
TOF_VALID_MAX_MM = 3500
BLOB_MIN_SIZE = 20
GOAL_CONF_MIN = 50
ANGLE_CENTI_DEG_MAX = 18000
ANGLE_CENTI_DEG_PER_DEG = 100
YAW_OFFSET_DEG100 = 0

# CONFIG_PUSH key registry (protocol 4.4).
KEY_TOF_VALID_MAX_MM = 0x0001
KEY_BLOB_MIN_SIZE = 0x0002
KEY_GOAL_CONF_MIN = 0x0003
KEY_YAW_OFFSET_DEG100 = 0x0004
TAG_INT32 = 1
TAG_UINT32 = 2

# Stream enables read by the main loop.
ENABLE_TOF_STREAM = True
ENABLE_IMU_STREAM = True
ENABLE_VISION_STREAM = True
ENABLE_RADAR_STREAM = True
ENABLE_HEARTBEAT = True

CAL_OK = 0
CAL_CANCELED = 1
CAL_REJECTED = 2
CAL_ERROR = 3

# (key, module attribute, tag, low, high) for every pushed knob.
_PUSHED_KNOBS = (
    (KEY_TOF_VALID_MAX_MM, "TOF_VALID_MAX_MM", TAG_UINT32, 0, TOF_RANGE_MAX_MM),
    (KEY_BLOB_MIN_SIZE, "BLOB_MIN_SIZE", TAG_UINT32, 0, COORD_MAX),
    (KEY_GOAL_CONF_MIN, "GOAL_CONF_MIN", TAG_UINT32, 0, CONFIDENCE_MAX_PCT),
    (KEY_YAW_OFFSET_DEG100, "YAW_OFFSET_DEG100", TAG_INT32,
     -ANGLE_CENTI_DEG_MAX, ANGLE_CENTI_DEG_MAX),
)

# Documented defaults; CONFIG_PUSH changes these only in memory.
_DEFAULTS = {
    "TOF_VALID_MAX_MM": 3500,
    "BLOB_MIN_SIZE": 20,
    "GOAL_CONF_MIN": 50,
    "YAW_OFFSET_DEG100": 0,
}


def _knob(attribute):
    """Current value of a registry attribute."""
    return globals()[attribute]


def _entry(key):
    """The registry row for ``key``, or None when the key is not ours."""
    for row in _PUSHED_KNOBS:
        if row[0] == key:
            return row
    return None


def value_of(key):
    """Live value of the CONFIG_PUSH key, or None for a key we do not know."""
    entry = _entry(key)
    return None if entry is None else _knob(entry[1])


def apply_push(tag, key, value):
    """Apply a CONFIG_PUSH value when the key, tag and range all match."""
    entry = _entry(key)
    if entry is None or entry[2] != tag:
        return False
    if not entry[3] <= value <= entry[4]:
        return False
    globals()[entry[1]] = value
    return True


def echo_entries():
    """Every pushed knob as (tag, key, live value) for the config echo."""
    return tuple((tag, key, _knob(name))
                 for key, name, tag, _low, _high in _PUSHED_KNOBS)


def yaw_offset_centi_deg():
    """The configured IMU_FRAME yaw bias in the wire's 0.01 deg/LSB units."""
    return YAW_OFFSET_DEG100


def reset():
    """Restore the pushed knobs to their documented defaults."""
    for _key, name, _tag, _low, _high in _PUSHED_KNOBS:
        globals()[name] = _DEFAULTS[name]
