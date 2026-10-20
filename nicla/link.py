"""Nicla-side mirror of the Mega <-> Nicla host-link protocol (v1.0).

Framing, CRC-8, the re-sync machine, the last-good cache, liveness, the
heartbeat, the host-silence suspension rule and typed send/receive over the
message table of ``docs/mega-nicla-protocol.md``. Every tunable lives in
``nicla.config`` and the UART is reached only through the injected ``port``
seam, so the module runs on a host with an in-memory port.
"""

from . import config

PROTO_MAJOR = 1
PROTO_MINOR = 0

# Frame format (protocol 3).
SYNC0 = 0xAA
SYNC1 = 0x55
CRC8_POLY = 0x07
CRC8_MSB = 0x80
CRC8_BIT_COUNT = 8
LEN_MIN = 2
BODY_OVERHEAD = 2

# Liveness states and first-transition events (protocol 5.1, 5.3).
LIVENESS_FRESH = 0
LIVENESS_STALE = 1
LIVENESS_LOST = 2
EVENT_NONE = 0
EVENT_STALE = 1
EVENT_LOST = 2
EVENT_REVIVED = 3

# Modes the host may command, and the TOF_RANGE status codes (protocol 4.2/4.3).
MODE_IDLE = 0
MODE_RUN = 1
MODE_CAL = 2
MODE_DIAG = 3
MODES = (MODE_IDLE, MODE_RUN, MODE_CAL, MODE_DIAG)
TOF_STATUS_VALID = 0
TOF_STATUS_OUT_OF_RANGE = 1
TOF_STATUS_ERROR = 2
IMU_FLAG_VALID = 0x01

# SENSOR_GATE mask bits (protocol 4.2).
GATE_YAW = 0x0001
GATE_PITCH = 0x0002
GATE_TOF = 0x0004
GATE_VISION = 0x0008

_WIDTH = {"u8": 1, "u16": 2, "i16": 2, "u32": 4}
_SIGNED = ("i16",)
_UP, _DOWN, _BOTH = "up", "down", "both"

# name -> (id, direction, "field:kind ..."). The single source of truth for the
# encoder, the parser and the named ids: "up" is daughter -> host.
MESSAGE_TABLE = {
    "HEARTBEAT": (0x00, _BOTH, ""),
    "READY": (0x01, _UP, "proto_major:u8 proto_minor:u8 cal_cached:u8"
                        " sensor_ok:u8 impl_version:u16"),
    "MODE_ACK": (0x02, _UP, "mode:u8 accepted:u8"),
    "GOAL_BLOB": (0x03, _UP, "cx:u16 cy:u16 width:u16 height:u16"
                           " confidence:u8"),
    "TRIANGLE_BLOB": (0x04, _UP, "color:u8 cx:u16 cy:u16 size:u8"),
    "TOF_RANGE": (0x05, _UP, "mm:u16 status:u8"),
    "RADAR_RESULT": (0x06, _UP, "zone:u8 class:u8"),
    "IMU_FRAME": (0x07, _UP, "yaw:i16 pitch:i16 flags:u8"),
    "TAPE_BLOB": (0x08, _UP, "type:u8 cx:u16 cy:u16 size:u8"),
    "DIAG_REPORT": (0x09, _UP, "target:u8 data:u32"),
    "CAL_COMPLETE": (0x0A, _UP, "status:u8"),
    "ERROR": (0x0B, _UP, "code:u8 info:u16"),
    "MODE_SET": (0x10, _DOWN, "mode:u8 arg:u8"),
    "RUN_START": (0x11, _DOWN, "flags:u16"),
    "DIAG_GET": (0x12, _DOWN, "target:u8"),
    "CAL_START": (0x13, _DOWN, "channel:u8"),
    "CAL_ABORT": (0x14, _DOWN, ""),
    "REQUEST_BLOB": (0x15, _DOWN, "mask:u16"),
    "REQUEST_RANGE": (0x16, _DOWN, ""),
    "REQUEST_RADAR": (0x17, _DOWN, "step_deg:u8 sectors:u8"),
    "CONFIG_PUSH": (0x18, _DOWN, "tag:u8 key:u16 value:u32"),
    "SENSOR_GATE": (0x19, _DOWN, "mask:u16"),
    "VERSION": (0x1A, _BOTH, "proto_major:u8 proto_minor:u8"
                            " impl_major:u8 impl_minor:u8"),
}


def _field_list(spec):
    """Ordered ((name, kind), ...) parsed from a payload spec string."""
    return tuple(tuple(part.split(":")) for part in spec.split())


def _payload_width(fields):
    """Payload length in bytes for an ordered field list; LEN is PL + 2."""
    return sum(_WIDTH[kind] for _name, kind in fields)


MESSAGE_IDS = dict((name, row[0]) for name, row in MESSAGE_TABLE.items())
_BY_ID = dict((row[0], (name, row[1], _field_list(row[2])))
              for name, row in MESSAGE_TABLE.items())


class MsgIds:
    """Named ids, derived from the table so there is one source of truth."""

    def __init__(self, ids):
        for name in ids:
            setattr(self, name, ids[name])


MSG = MsgIds(MESSAGE_IDS)


def crc8(data):
    """CRC-8 over ``data``: poly 0x07, init 0x00, no final xor, MSB-first."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(CRC8_BIT_COUNT):
            if crc & CRC8_MSB:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _encode_field(kind, value):
    """One field as big-endian bytes, two's complement when signed."""
    width = _WIDTH[kind]
    limit = 1 << (8 * width - 1)
    if not (-limit if kind in _SIGNED else 0) <= value < 2 * limit:
        return None
    unsigned = value & ((1 << (8 * width)) - 1)
    return bytes([(unsigned >> shift) & 0xFF
                  for shift in range((width - 1) * 8, -8, -8)])


def _decode_field(kind, data, offset):
    """One field read back as a Python int, sign-extended when signed."""
    raw = 0
    for index in range(_WIDTH[kind]):
        raw = (raw << 8) | data[offset + index]
    return raw - (1 << (8 * _WIDTH[kind])) if kind in _SIGNED and raw >> (
        8 * _WIDTH[kind] - 1) else raw


def encode_frame(msg_id, values=None):
    """``AA 55 LEN ID payload CRC``, or None when the request is malformed.

    LEN is authoritative: the payload comes from the table's field list, so the
    frame length and the field values can never disagree.
    """
    entry = _BY_ID.get(msg_id)
    if entry is None:
        return None
    _name, _direction, fields = entry
    values = {} if values is None else values
    if sorted(values) != sorted(field[0] for field in fields):
        return None
    body = bytearray((_payload_width(fields) + BODY_OVERHEAD, msg_id))
    for name, kind in fields:
        encoded = _encode_field(kind, values[name])
        if encoded is None:
            return None
        body += encoded
    return bytes((SYNC0, SYNC1)) + bytes(body) + bytes((crc8(body),))


def decode_fields(msg_id, payload):
    """Field dict for ``msg_id``, or None when the payload is the wrong width."""
    entry = _BY_ID.get(msg_id)
    if entry is None or len(payload) != _payload_width(entry[2]):
        return None
    values, offset = {}, 0
    for name, kind in entry[2]:
        values[name] = _decode_field(kind, payload, offset)
        offset += _WIDTH[kind]
    return values


# Freshness window per id (protocol 5.2). Commands are sticky: they hold until
# the host supersedes them. The heartbeat ages out exactly when the link does.
_CACHE_TTL_MS = {
    MSG.HEARTBEAT: config.LIVENESS_FRESH_MS,
    MSG.DIAG_GET: config.CACHE_TTL_NONE,
    MSG.CAL_START: config.CACHE_TTL_STICKY,
    MSG.CAL_ABORT: config.CACHE_TTL_NONE,
    MSG.REQUEST_RANGE: config.CACHE_TTL_NONE,
    MSG.REQUEST_RADAR: config.CACHE_TTL_STICKY,
}


def ttl_ms_for(msg_id):
    """Freshness window for ``msg_id``; CACHE_TTL_NONE = the link does not hold it."""
    return _CACHE_TTL_MS.get(msg_id, config.CACHE_TTL_STICKY)


def _age_since(stamp_ms, now_ms):
    """Unsigned millisecond age that survives the microsecond-timer wrap."""
    return (now_ms - stamp_ms) & 0x3FFFFFFF


def _clamp_centi(centi_deg100):
    """Fold a 0.01 deg value into the documented +-180.00 deg window."""
    return max(-config.ANGLE_CENTI_DEG_MAX,
               min(config.ANGLE_CENTI_DEG_MAX, centi_deg100))


def yaw_centi_deg(yaw_deg):
    """Yaw in 0.01 deg/LSB, folded into the window with the configured bias."""
    folded = ((yaw_deg + 180.0) % 360.0) - 180.0
    return _clamp_centi(int(round(folded * config.ANGLE_CENTI_DEG_PER_DEG))
                        + config.yaw_offset_centi_deg())


def tof_status(mm):
    """TOF_RANGE status for a reading, from the configured validity window."""
    if mm is None or mm < 0 or mm > config.TOF_RANGE_MAX_MM:
        return TOF_STATUS_ERROR
    return (TOF_STATUS_OUT_OF_RANGE if mm > config.TOF_VALID_MAX_MM
            else TOF_STATUS_VALID)


def radar_zone_from_profile(bins_mm):
    """Sector toward the most open run, or ZONE_NO_RESULT when there is none."""
    first = count = best_first = best_count = 0
    # The trailing sentinel closes a run that ends on the last sector.
    close = [1 if bin_mm <= config.RADAR_CLOSE_MM else 0 for bin_mm in bins_mm]
    for index, bit in enumerate(close + [1]):
        if bit == 0:
            first = index if count == 0 else first
            count += 1
        else:
            best_first, best_count = (first, count) if count > best_count else (
                best_first, best_count)
            count = 0
    if best_count == 0:
        return config.RADAR_ZONE_NO_RESULT
    return (2 * best_first + best_count - 1) // 2


def radar_result_from_profile(profile):
    """``(zone, class)`` for a ToF sweep profile; class follows its outcome."""
    enclosed = profile["outcome"] != "open"
    klass = (config.RADAR_CLASS_OPEN if not enclosed
             else config.RADAR_CLASS_ENCLOSED)
    return radar_zone_from_profile(profile["bins_mm"]), klass


class FrameDecoder:
    """HUNT/FRAME re-sync machine (protocol 5.4).

    A CRC mismatch or an out-of-range LEN is one bad frame; the third discards
    the buffer and re-hunts the preamble. Unknown ids are not violations, so a
    fully decoded frame always clears the bad-frame count.
    """

    HUNT = 0
    PREAMBLE = 1
    BODY = 2

    def __init__(self):
        self.state = self.HUNT
        self.bad_frames = 0
        self.resyncs = 0
        self._body = bytearray()

    def feed(self, data, max_frames=1):
        """Decode up to ``max_frames`` frames; returns (frames, resynced)."""
        frames, resynced = [], False
        for byte in data:
            if len(frames) >= max_frames:
                break
            if self.state == self.HUNT:
                self.state = self.PREAMBLE if byte == SYNC0 else self.HUNT
            elif self.state == self.PREAMBLE:
                self._read_preamble(byte)
            else:
                frame, dropped = self._read_body(byte)
                resynced = resynced or dropped
                if frame is not None:
                    frames.append(frame)
        return frames, resynced

    def _read_preamble(self, byte):
        """A lone 0xAA is dropped; the mismatch byte is re-examined."""
        if byte == SYNC1:
            self.state = self.BODY
            self._body = bytearray()
        elif byte != SYNC0:
            self.state = self.HUNT

    def _reject(self):
        """Count one bad frame; the third discards the buffer and re-hunts."""
        self.state = self.HUNT
        self._body = bytearray()
        self.bad_frames += 1
        if self.bad_frames < config.BAD_FRAME_LIMIT:
            return False
        self.bad_frames = 0
        self.resyncs += 1
        return True

    def _read_body(self, byte):
        """Consume one body byte: LEN, then id and payload, then the CRC."""
        self._body.append(byte)
        if len(self._body) == 1 and self._body[0] < LEN_MIN:
            return None, self._reject()
        if len(self._body) <= self._body[0]:
            return None, False
        body = self._body
        if crc8(body[:body[0]]) != body[body[0]]:
            return None, self._reject()
        self.bad_frames = 0
        self.state = self.HUNT
        self._body = bytearray()
        return (body[1], bytes(body[BODY_OVERHEAD:body[0]])), False  # id, payload


class MachineUartPort:
    """Default delivery seam: the board's UART at the documented baud.

    ``machine`` is imported here so the module stays importable on a host. The
    seam only moves bytes; every protocol decision lives in the codec.
    """

    def __init__(self, uart_id=config.UART_ID):
        import machine
        self._uart = machine.UART(uart_id, config.UART_BAUD)

    def available(self):
        """Bytes waiting in the receive buffer."""
        return self._uart.any()

    def read(self, count):
        """Take up to ``count`` received bytes."""
        return self._uart.read(count)

    def write(self, data):
        """Hand one queued frame to the transmit register."""
        return self._uart.write(data)


def _host_millis():
    """Millisecond clock: the microsecond timer on device, monotonic on host."""
    import time
    ticks = getattr(time, "ticks_ms", None)
    return ticks() if ticks else int(time.monotonic() * 1000)


class NiclaLink:
    """Responder on the daughter side of the host link.

    ``port`` is the delivery seam (anything with ``available()``, ``read(n)``
    and ``write(data)``); ``clock`` returns milliseconds. The link holds the
    last-good value of every command the host sends, ages the link, answers the
    one-shot requests, and gates the streaming frames on the documented
    suspension rule.
    """

    def __init__(self, port=None, clock=None):
        self._port = MachineUartPort() if port is None else port
        self._clock = _host_millis if clock is None else clock
        self._decoder = FrameDecoder()
        self._tx = []
        self._cache = {}
        self._liveness = LIVENESS_LOST
        self._event = EVENT_NONE
        self._decoded_frames = 0
        self._tx_overflows = 0
        self._last_mega_frame_ms = None
        self._seen = False
        self._last_heartbeat_ms = self._clock()
        self._mode = MODE_IDLE
        self._range_request = None
        self._radar_request = None

    # --- clock, liveness and the suspension rule ---

    def _now(self):
        return self._clock()

    def _age_since(self, stamp_ms):
        return _age_since(stamp_ms, self._now())

    def liveness(self):
        """FRESH / STALE / LOST from the age of the last frame of any type."""
        if self._last_mega_frame_ms is None:
            liveness = LIVENESS_LOST
        else:
            age_ms = self._age_since(self._last_mega_frame_ms)
            liveness = (LIVENESS_FRESH if age_ms < config.LIVENESS_FRESH_MS
                        else LIVENESS_STALE if age_ms < config.LIVENESS_LOST_MS
                        else LIVENESS_LOST)
        self._advance_liveness(liveness)
        return self._liveness

    def _advance_liveness(self, liveness):
        """Record the first transition into another liveness state (5.3)."""
        if liveness == self._liveness:
            return
        if liveness == LIVENESS_FRESH:
            self._event = (EVENT_REVIVED
                           if self._liveness == LIVENESS_LOST and self._seen
                           else EVENT_NONE)
        else:
            self._event = (EVENT_STALE if liveness == LIVENESS_STALE
                           else EVENT_LOST)
        self._liveness = liveness

    def take_event(self):
        """Consume the first-transition event since the last call."""
        event, self._event = self._event, EVENT_NONE
        return event

    def suspended(self):
        """True once the host has been silent for more than the window.

        Before the first frame ever arrives the link is not suspended: READY
        is gated only by the daughter self-check, never by host state.
        """
        if self._last_mega_frame_ms is None:
            return False
        return self._age_since(self._last_mega_frame_ms) > config.MEGA_SILENCE_MS

    def effective_mode(self):
        """IDLE while the host is silent, otherwise the last accepted mode."""
        return MODE_IDLE if self.suspended() else self._mode

    def mode(self):
        """The last accepted mode, whatever the host has done since."""
        return self._mode

    def streams_allowed(self, gate_bit):
        """Gated streams are off while suspended or with their bit clear."""
        return not self.suspended() and bool(self.gate_mask() & gate_bit)

    # --- last-good cache and decoded host state ---

    def last_good(self, msg_id):
        """Fresh last-good fields for ``msg_id``, or None once they expired."""
        entry = self._cache.get(msg_id)
        if entry is None:
            return None
        stamp_ms, values = entry
        ttl_ms = ttl_ms_for(msg_id)
        if ttl_ms != config.CACHE_TTL_STICKY and self._age_since(stamp_ms) >= ttl_ms:
            return None
        return dict(values)

    def _last_decoded(self, msg_id):
        """Fields as last decoded, without the freshness gate."""
        entry = self._cache.get(msg_id)
        return None if entry is None else entry[1]

    def gate_mask(self):
        """The SENSOR_GATE mask the host last commanded."""
        values = self._last_decoded(MSG.SENSOR_GATE)
        return 0 if values is None else values["mask"]

    def peer_heartbeat_fresh(self):
        """True while the host's last-good heartbeat is inside its window."""
        return self.last_good(MSG.HEARTBEAT) is not None

    def radar_request(self):
        """One-shot: the sweep the host asked for, 0 read as the default."""
        requested, self._radar_request = self._radar_request, None
        if requested is None:
            return None
        return (requested["step_deg"] or config.RADAR_DEFAULT_STEP_DEG,
                requested["sectors"] or config.RADAR_SECTOR_COUNT)

    def take_range_request(self):
        """One-shot: True once after a REQUEST_RANGE, then cleared."""
        requested, self._range_request = self._range_request, None
        return requested is not None

    # --- counters ---

    def bad_frames(self):
        """Consecutive bad frames seen since the last good one."""
        return self._decoder.bad_frames

    def resyncs(self):
        """Re-syncs performed after the bad-frame limit was reached."""
        return self._decoder.resyncs

    def decoded_frames(self):
        """Fully decoded frames since the link came up."""
        return self._decoded_frames

    def tx_overflows(self):
        """Frames dropped because the transmit FIFO was full."""
        return self._tx_overflows

    def diag_counters(self):
        """Packed diag word: bad frames, TX overflows, frames decoded."""
        return ((self._decoder.bad_frames & 0xFF)
                | ((self._tx_overflows & 0xFF) << 8)
                | ((self._decoded_frames & 0xFFFF) << 16))

    # --- receive and transmit ---

    def pump(self):
        """One non-blocking tick: receive, age, transmit, heartbeat if due."""
        now = self._now()
        self._receive(now)
        self._pump_tx()
        if self._age_since(self._last_heartbeat_ms) >= config.HEARTBEAT_PERIOD_MS:
            self._last_heartbeat_ms = now
            self.send_heartbeat()
        return now

    def _receive(self, now_ms):
        pending = self._port.available()
        if not pending:
            return
        data = self._port.read(min(pending, config.RX_BYTE_BUDGET))
        frames, _resynced = self._decoder.feed(data, config.RX_FRAMES_PER_PUMP)
        for msg_id, payload in frames:
            self._on_frame(msg_id, payload, now_ms)

    def _on_frame(self, msg_id, payload, now_ms):
        """Cache the value, run the command, then refresh liveness.

        Handling before the liveness refresh is deliberate: a frame that ends a
        silence window is answered under the state the host left behind, and it
        is that frame which resumes the gated streams.
        """
        self._decoded_frames += 1
        entry = _BY_ID.get(msg_id)
        values = None if entry is None else decode_fields(msg_id, payload)
        if values is not None and entry[1] != _UP:
            if ttl_ms_for(msg_id) != config.CACHE_TTL_NONE:
                self._cache[msg_id] = (now_ms, values)
            _COMMANDS.get(entry[0], _no_command)(self, values)
        self._note_frame(now_ms)

    def _note_frame(self, now_ms):
        """Any decoded frame proves liveness and resumes the gated streams."""
        self._advance_liveness(LIVENESS_FRESH)
        self._last_mega_frame_ms = now_ms
        self._seen = True

    def _send(self, msg_id, values):
        """Queue one frame, dropping the oldest when the FIFO is full."""
        frame = encode_frame(msg_id, values)
        if frame is None or len(frame) > config.TX_SLOT_BYTES:
            return None
        if len(self._tx) >= config.TX_FRAME_SLOTS:
            del self._tx[0]
            self._tx_overflows += 1
        self._tx.append(frame)
        return frame

    def _pump_tx(self):
        budget = config.TX_FRAMES_PER_PUMP
        while budget > 0 and self._tx:
            self._port.write(self._tx.pop(0))
            budget -= 1

    # --- typed sends ---

    def send_heartbeat(self):
        """HEARTBEAT; never gated, the link proves liveness on its own."""
        return self._send(MSG.HEARTBEAT, {})

    def send_ready(self, cal_cached, sensor_ok, impl_version=0):
        """READY; sent on power-up and after every reset, host state aside."""
        return self._send(MSG.READY, {"proto_major": PROTO_MAJOR,
                                      "proto_minor": PROTO_MINOR,
                                      "cal_cached": 1 if cal_cached else 0,
                                      "sensor_ok": sensor_ok,
                                      "impl_version": impl_version})

    def send_mode_ack(self, mode, accepted):
        """MODE_ACK: the accepted mode, or the current one when rejected."""
        return self._send(MSG.MODE_ACK, {"mode": mode,
                                         "accepted": 1 if accepted else 0})

    def send_goal_blob(self, cx, cy, width, height, confidence):
        """GOAL_BLOB from a vision blob, when the vision stream is gated on."""
        if not self.streams_allowed(GATE_VISION):
            return None
        return self._send(MSG.GOAL_BLOB, {"cx": cx, "cy": cy, "width": width,
                                          "height": height,
                                          "confidence": confidence})

    def send_triangle_blob(self, color, cx, cy, size):
        """TRIANGLE_BLOB; ``color`` is 0 red, 1 green."""
        if not self.streams_allowed(GATE_VISION):
            return None
        return self._send(MSG.TRIANGLE_BLOB, {"color": color, "cx": cx,
                                               "cy": cy, "size": size})

    def send_tape_blob(self, tape_type, cx, cy, size):
        """TAPE_BLOB; ``tape_type`` is 0 silver, 1 black."""
        if not self.streams_allowed(GATE_VISION):
            return None
        return self._send(MSG.TAPE_BLOB, {"type": tape_type, "cx": cx,
                                          "cy": cy, "size": size})

    def send_tof_range(self, mm):
        """TOF_RANGE with the status the configured validity window implies."""
        if not self.streams_allowed(GATE_TOF):
            return None
        return self._send(MSG.TOF_RANGE, {"mm": max(0, mm),
                                          "status": tof_status(mm)})

    def send_radar_result(self, zone, klass):
        """RADAR_RESULT; a one-shot answer, not a gated stream."""
        if self.suspended():
            return None
        return self._send(MSG.RADAR_RESULT, {"zone": zone, "class": klass})

    def send_radar_result_from_profile(self, profile):
        """RADAR_RESULT for a ToF sweep profile; returns ``(zone, class)``."""
        zone, klass = radar_result_from_profile(profile)
        self.send_radar_result(zone, klass)
        return (zone, klass)

    def send_imu_frame(self, yaw_deg, pitch_deg, valid=True):
        """IMU_FRAME in 0.01 deg/LSB; the yaw stream gate covers both angles."""
        if not self.streams_allowed(GATE_YAW):
            return None
        pitch = _clamp_centi(int(round(pitch_deg
                                       * config.ANGLE_CENTI_DEG_PER_DEG)))
        return self._send(MSG.IMU_FRAME, {"yaw": yaw_centi_deg(yaw_deg),
                                          "pitch": pitch,
                                          "flags": IMU_FLAG_VALID if valid
                                          else 0})

    def send_diag_report(self, target, data):
        """DIAG_REPORT answering a DIAG_GET target."""
        return self._send(MSG.DIAG_REPORT, {"target": target, "data": data})

    def send_cal_complete(self, status):
        """CAL_COMPLETE with one of the registry's status codes."""
        return self._send(MSG.CAL_COMPLETE, {"status": status})

    def send_error(self, code, info):
        """ERROR: the control code plus its information word."""
        return self._send(MSG.ERROR, {"code": code, "info": info})


def _no_command(link, values):
    """Default command handler: the cache and liveness did the work."""
    return None


def _on_mode_set(link, values):
    """MODE_SET: accept a known mode, else answer with the current one."""
    mode = values["mode"]
    if mode in MODES:
        link._mode = mode
        link.send_mode_ack(mode, 1)
    else:
        link.send_mode_ack(link.effective_mode(), 0)


def _on_diag_get(link, values):
    """DIAG_GET: answer the target with the link counters."""
    link.send_diag_report(values["target"], link.diag_counters())


def _on_cal_abort(link, values):
    """CAL_ABORT: the last CAL_START is void, so report the cancellation."""
    link._cache.pop(MSG.CAL_START, None)
    link.send_cal_complete(config.CAL_CANCELED)


def _on_request_range(link, values):
    """REQUEST_RANGE: latch the one-shot so a range is answered once."""
    link._range_request = link._now()


def _on_request_radar(link, values):
    """REQUEST_RADAR: latch the one-shot sweep request."""
    link._radar_request = values


def _on_config_push(link, values):
    """CONFIG_PUSH: apply at once, never persist, ignore what is not ours."""
    config.apply_push(values["tag"], values["key"], values["value"])


_COMMANDS = {
    "MODE_SET": _on_mode_set,
    "DIAG_GET": _on_diag_get,
    "CAL_ABORT": _on_cal_abort,
    "REQUEST_RANGE": _on_request_range,
    "REQUEST_RADAR": _on_request_radar,
    "CONFIG_PUSH": _on_config_push,
}
