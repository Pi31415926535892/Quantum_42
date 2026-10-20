"""Host tests for the Nicla-side mirror of the Mega <-> Nicla host link.

Pure Python and stdlib only: the link is driven through an in-memory port and a
hand-advanced clock, so the wire format, the re-sync machine, the freshness
contract, the Mega-silence suspension rule and the radar mapping are all
asserted without hardware. The CRC reference below is transcribed from the
protocol's own C routine so the codec is never checked against itself.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicla import config, link, tof

HEARTBEAT_VECTOR = bytes((0xAA, 0x55, 0x02, 0x00, 0x2A))
RUN_START_VECTOR = bytes((0xAA, 0x55, 0x04, 0x11, 0x00, 0x03, 0x98))
HOST_COMMAND_IDS = (0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                    0x19, 0x1A)
DAUGHTER_REPLY_IDS = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x09, 0x0A, 0x0B)


def doc_crc8(data):
    """Reference CRC-8: poly 0x07, init 0x00, no final xor, MSB-first."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def corrupt(frame, index=4):
    """Flip one bit of ``frame`` so its CRC no longer matches."""
    broken = bytearray(frame)
    broken[index] ^= 0x01
    return bytes(broken)


def radar_vector(zone, klass):
    """The RADAR_RESULT frame for a zone/class pair, CRC from the reference."""
    scope = bytes((0x04, link.MSG.RADAR_RESULT, zone, klass))
    return bytes((0xAA, 0x55)) + scope + bytes((doc_crc8(scope),))


def decode_written(port):
    """Every frame the link has written so far, decoded to field dicts."""
    written = bytes(port.written)
    frames, _resynced = link.FrameDecoder().feed(written, len(written))
    return {msg_id: link.decode_fields(msg_id, payload)
            for msg_id, payload in frames}


class InMemoryPort:
    """Delivery seam: what the link writes is what the peer reads."""

    def __init__(self):
        self._inbound = bytearray()
        self.written = bytearray()

    def available(self):
        return len(self._inbound)

    def read(self, count):
        chunk = bytes(self._inbound[:count])
        del self._inbound[:count]
        return chunk

    def write(self, data):
        self.written += data
        return len(data)

    def feed(self, data):
        """Put bytes on the link's receive path."""
        self._inbound += data


class FakeClock:
    """Monotonic millisecond clock the tests drive by hand."""

    def __init__(self, start_ms=0):
        self._now_ms = start_ms

    def __call__(self):
        return self._now_ms

    def advance(self, delta_ms):
        self._now_ms += delta_ms
        return self._now_ms


class FakeUart:
    """Stand-in for machine.UART: records how the default port built it."""

    def __init__(self, uart_id, baud):
        self.built_with = (uart_id, baud)
        self.inbound = bytearray()
        self.outbound = bytearray()

    def any(self):
        return len(self.inbound)

    def read(self, count):
        chunk = bytes(self.inbound[:count])
        del self.inbound[:count]
        return chunk

    def write(self, data):
        self.outbound += data
        return len(data)


class PortDeliveryTest(unittest.TestCase):
    """The default seam talks to machine.UART at the documented baud."""

    def setUp(self):
        self.fake = FakeUart(0, 0)
        self.built_with = []
        module = type(sys)("machine")
        module.UART = lambda uart_id, baud: (self.built_with.append(
            (uart_id, baud)) or self.fake)
        sys.modules["machine"] = module
        self.addCleanup(sys.modules.pop, "machine", None)

    def test_default_port_moves_bytes_at_the_documented_baud(self):
        port = link.MachineUartPort()
        self.assertEqual(self.built_with,
                         [(config.UART_ID, config.UART_BAUD)])
        self.fake.inbound += b"\x01\x02"
        self.assertEqual(port.available(), 2)
        self.assertEqual(port.read(2), b"\x01\x02")
        self.assertEqual(port.write(b"ok"), 2)
        self.assertEqual(bytes(self.fake.outbound), b"ok")

    def test_link_falls_back_to_a_host_clock(self):
        nicla = link.NiclaLink(port=InMemoryPort())
        self.assertGreaterEqual(nicla.pump(), nicla.pump())


class LinkFixture(unittest.TestCase):
    """A Nicla link on an in-memory port with a hand-driven clock."""

    def setUp(self):
        config.reset()
        self.clock = FakeClock()
        self.port = InMemoryPort()
        self.link = link.NiclaLink(port=self.port, clock=self.clock)

    def tearDown(self):
        config.reset()

    def open_streams(self, mask=0x0F):
        """Send the SENSOR_GATE frame that opens the streaming bits."""
        self.port.feed(link.encode_frame(link.MSG.SENSOR_GATE, {"mask": mask}))
        self.link.pump()

    def drain(self, rounds=8):
        """Pump enough ticks to push every queued frame out of the FIFO."""
        for _ in range(rounds):
            self.link.pump()

    def emit(self, send, *args):
        """Call one typed send, then push its frame out of the FIFO."""
        frame = send(*args)
        self.drain(1)
        return frame

    def feed(self, msg_id, values):
        """Hand one host frame to the link and run one receive tick."""
        self.port.feed(link.encode_frame(msg_id, values))
        self.link.pump()


class CrcVectorTest(unittest.TestCase):
    """The protocol's two worked examples, asserted byte for byte."""

    def test_heartbeat_vector(self):
        self.assertEqual(link.encode_frame(link.MSG.HEARTBEAT), HEARTBEAT_VECTOR)

    def test_run_start_vector(self):
        frame = link.encode_frame(link.MSG.RUN_START, {"flags": 0x0003})
        self.assertEqual(frame, RUN_START_VECTOR)

    def test_crc_of_empty_scope_is_the_init_value(self):
        self.assertEqual(link.crc8(b""), 0x00)

    def test_crc_agrees_with_the_reference_routine(self):
        scope = bytes((0x04, 0x06, 0x0B, 0x01, 0x02, 0x03))
        self.assertEqual(link.crc8(scope), doc_crc8(scope))


class MessageTableTest(LinkFixture):
    """Every message id in the table builds and parses to the same values."""

    def test_host_commands_decode_back_to_the_values_sent(self):
        commands = [
            (link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0}),
            (link.MSG.RUN_START, {"flags": 0x0001}),
            (link.MSG.DIAG_GET, {"target": 0}),
            (link.MSG.CAL_START, {"channel": 0}),
            (link.MSG.CAL_ABORT, {}),
            (link.MSG.REQUEST_BLOB, {"mask": 0x0003}),
            (link.MSG.REQUEST_RANGE, {}),
            (link.MSG.REQUEST_RADAR, {"step_deg": 0, "sectors": 0}),
            (link.MSG.SENSOR_GATE, {"mask": 0x000F}),
            (link.MSG.CAL_START, {"channel": 0}),
            (link.MSG.VERSION, {"proto_major": 1, "proto_minor": 0,
                                "impl_major": 2, "impl_minor": 5}),
        ]
        for msg_id, values in commands:
            frame = link.encode_frame(msg_id, values)
            self.assertIsNotNone(frame, msg_id)
            self.assertEqual(link.decode_fields(msg_id, frame[4:-1]), values)
            self.port.feed(frame)
            self.link.pump()
        self.assertEqual(self.link.mode(), link.MODE_RUN)
        self.assertEqual(self.link.gate_mask(), 0x000F)
        self.assertEqual(self.link.last_good(link.MSG.RUN_START)["flags"], 1)
        self.assertEqual(self.link.last_good(link.MSG.VERSION)["proto_major"], 1)
        self.assertEqual(self.link.last_good(link.MSG.REQUEST_BLOB)["mask"], 3)
        self.assertEqual(self.link.radar_request(),
                         (config.RADAR_DEFAULT_STEP_DEG,
                          config.RADAR_SECTOR_COUNT))
        self.assertIsNone(self.link.radar_request())
        self.assertTrue(self.link.take_range_request())
        self.assertFalse(self.link.take_range_request())
        self.assertEqual(self.link.last_good(link.MSG.CAL_START)["channel"], 0)
        self.feed(link.MSG.CAL_ABORT, {})
        self.assertIsNone(self.link.last_good(link.MSG.CAL_START))
        self.assertEqual(decode_written(self.port)[link.MSG.CAL_COMPLETE],
                         {"status": config.CAL_CANCELED})

    def test_daughter_replies_round_trip_through_the_port(self):
        self.open_streams()
        emit = self.emit
        emit(self.link.send_ready, 1, 0x0F, 0x0102)
        emit(self.link.send_mode_ack, link.MODE_RUN, 1)
        emit(self.link.send_goal_blob, 500, 400, 120, 90, 80)
        emit(self.link.send_triangle_blob, 1, 300, 250, 60)
        emit(self.link.send_tape_blob, 0, 800, 100, 40)
        emit(self.link.send_tof_range, 1234)
        emit(self.link.send_radar_result, 11, config.RADAR_CLASS_ENCLOSED)
        emit(self.link.send_imu_frame, 350.0, -12.5)
        emit(self.link.send_diag_report, 0, 0x0A0B0C)
        emit(self.link.send_cal_complete, config.CAL_OK)
        emit(self.link.send_error, 0, 0x0201)
        decoded = decode_written(self.port)
        self.assertEqual(decoded, {
            link.MSG.READY: {"proto_major": 1, "proto_minor": 0,
                             "cal_cached": 1, "sensor_ok": 0x0F,
                             "impl_version": 0x0102},
            link.MSG.MODE_ACK: {"mode": link.MODE_RUN, "accepted": 1},
            link.MSG.GOAL_BLOB: {"cx": 500, "cy": 400, "width": 120,
                                 "height": 90, "confidence": 80},
            link.MSG.TRIANGLE_BLOB: {"color": 1, "cx": 300, "cy": 250,
                                     "size": 60},
            link.MSG.TAPE_BLOB: {"type": 0, "cx": 800, "cy": 100, "size": 40},
            link.MSG.TOF_RANGE: {"mm": 1234, "status": link.TOF_STATUS_VALID},
            link.MSG.RADAR_RESULT: {"zone": 11,
                                    "class": config.RADAR_CLASS_ENCLOSED},
            link.MSG.IMU_FRAME: {"yaw": -1000, "pitch": -1250, "flags": 1},
            link.MSG.DIAG_REPORT: {"target": 0, "data": 0x0A0B0C},
            link.MSG.CAL_COMPLETE: {"status": config.CAL_OK},
            link.MSG.ERROR: {"code": 0, "info": 0x0201},
        })

    def test_table_covers_every_documented_id(self):
        self.assertEqual(sorted(link.MESSAGE_IDS.values()),
                         sorted(DAUGHTER_REPLY_IDS + HOST_COMMAND_IDS + (0x00,)))

    def test_transmit_fifo_drops_the_oldest_and_reports_counters(self):
        self.open_streams(0x000F)
        for _ in range(config.TX_FRAME_SLOTS):
            self.link.send_goal_blob(1, 1, 1, 1, 1)
        self.link.send_goal_blob(2, 2, 2, 2, 2)
        self.assertEqual(self.link.tx_overflows(), 1)
        self.feed(link.MSG.DIAG_GET, {"target": 0})
        self.drain()
        data = decode_written(self.port)[link.MSG.DIAG_REPORT]["data"]
        self.assertEqual(data & 0xFF, 0)
        self.assertEqual((data >> 8) & 0xFF, 1)
        self.assertEqual((data >> 16) & 0xFFFF, self.link.decoded_frames())

    def test_config_push_frame_retunes_the_tof_window(self):
        self.feed(link.MSG.CONFIG_PUSH, {"tag": config.TAG_UINT32,
                                          "key": config.KEY_TOF_VALID_MAX_MM,
                                          "value": 1200})
        self.assertEqual(config.value_of(config.KEY_TOF_VALID_MAX_MM), 1200)
        self.assertEqual(link.tof_status(1300), link.TOF_STATUS_OUT_OF_RANGE)

    def test_heartbeat_is_emitted_on_its_cadence(self):
        self.link.pump()
        self.assertNotIn(link.MSG.HEARTBEAT, decode_written(self.port))
        self.clock.advance(config.HEARTBEAT_PERIOD_MS)
        self.link.pump()
        self.link.pump()
        self.assertEqual(decode_written(self.port),
                         {link.MSG.HEARTBEAT: {}})


class CorruptedFrameTest(LinkFixture):
    """CRC, framing and reserved-id handling (protocol 1, 5.4)."""

    def test_corrupted_payload_is_rejected_and_state_stands(self):
        self.port.feed(link.encode_frame(link.MSG.MODE_SET,
                                          {"mode": link.MODE_RUN, "arg": 0}))
        self.link.pump()
        self.assertEqual(self.link.mode(), link.MODE_RUN)
        before = bytes(self.port.written)
        self.port.feed(corrupt(link.encode_frame(link.MSG.MODE_SET,
                                                 {"mode": link.MODE_DIAG,
                                                  "arg": 0})))
        self.link.pump()
        self.assertEqual(self.link.mode(), link.MODE_RUN)
        self.assertEqual(self.link.last_good(link.MSG.MODE_SET)["mode"],
                         link.MODE_RUN)
        self.assertEqual(bytes(self.port.written), before)
        self.assertEqual(self.link.decoded_frames(), 1)
        self.assertEqual(self.link.bad_frames(), 1)

    def test_three_bad_frames_resync_on_the_next_preamble(self):
        bad = corrupt(link.encode_frame(link.MSG.HEARTBEAT), index=-1)
        for expected_bad in (1, 2):
            self.port.feed(bad)
            self.link.pump()
            self.assertEqual(self.link.bad_frames(), expected_bad)
        self.assertEqual(self.link.resyncs(), 0)
        self.port.feed(bad)
        self.link.pump()
        self.assertEqual(self.link.resyncs(), 1)
        self.assertEqual(self.link.bad_frames(), 0)
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.assertEqual(self.link.liveness(), link.LIVENESS_FRESH)
        self.assertEqual(self.link.decoded_frames(), 1)
        self.assertEqual(self.link.bad_frames(), 0)

    def test_hunt_scan_recovers_from_a_lone_preamble_byte(self):
        noise = bytes((0x00, 0xAA, 0x13, 0xAA, 0x13, 0x00))
        self.port.feed(noise + link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.assertEqual(self.link.liveness(), link.LIVENESS_FRESH)
        self.assertEqual(self.link.bad_frames(), 0)
        self.assertEqual(self.link.decoded_frames(), 1)

    def test_out_of_range_len_is_a_framing_error(self):
        self.port.feed(bytes((0xAA, 0x55, 0x01, 0x00, 0x00)))
        self.link.pump()
        self.assertEqual(self.link.bad_frames(), 1)
        self.assertEqual(self.link.decoded_frames(), 0)

    def test_reserved_id_is_ignored_without_a_frame_error(self):
        scope = bytes((0x03, 0x77, 0x05))
        reserved = bytes((0xAA, 0x55)) + scope + bytes((doc_crc8(scope),))
        self.port.feed(reserved)
        self.link.pump()
        self.assertEqual(self.link.bad_frames(), 0)
        self.assertEqual(self.link.resyncs(), 0)
        self.assertEqual(self.link.decoded_frames(), 1)
        self.assertIsNone(self.link.last_good(0x77))
        self.assertEqual(self.link.liveness(), link.LIVENESS_FRESH)


class LivenessTest(LinkFixture):
    """Liveness ages, its first transitions and the per-type windows."""

    def test_ages_walk_fresh_to_stale_to_lost(self):
        self.assertEqual(self.link.liveness(), link.LIVENESS_LOST)
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.assertEqual(self.link.liveness(), link.LIVENESS_FRESH)
        self.clock.advance(config.LIVENESS_FRESH_MS - 1)
        self.assertEqual(self.link.liveness(), link.LIVENESS_FRESH)
        self.assertEqual(self.link.take_event(), link.EVENT_NONE)
        self.clock.advance(1)
        self.assertEqual(self.link.liveness(), link.LIVENESS_STALE)
        self.assertEqual(self.link.take_event(), link.EVENT_STALE)
        self.assertFalse(self.link.peer_heartbeat_fresh())
        self.clock.advance(config.LIVENESS_LOST_MS - config.LIVENESS_FRESH_MS)
        self.assertEqual(self.link.liveness(), link.LIVENESS_LOST)
        self.assertEqual(self.link.take_event(), link.EVENT_LOST)

    def test_first_frame_after_lost_reports_revived(self):
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.clock.advance(config.LIVENESS_LOST_MS)
        self.assertEqual(self.link.liveness(), link.LIVENESS_LOST)
        self.assertEqual(self.link.take_event(), link.EVENT_LOST)
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.assertEqual(self.link.take_event(), link.EVENT_REVIVED)

    def test_sticky_commands_survive_the_heartbeat_gap(self):
        self.open_streams(0x000F)
        self.assertEqual(self.link.gate_mask(), 0x000F)
        self.clock.advance(config.LIVENESS_LOST_MS)
        self.assertEqual(self.link.last_good(link.MSG.SENSOR_GATE)["mask"],
                         0x000F)
        self.assertEqual(self.link.gate_mask(), 0x000F)


class SuspensionTest(LinkFixture):
    """The Mega-silence rule: gated streams stop, heartbeats do not."""

    def test_streams_suspend_only_after_more_than_the_silence_window(self):
        self.open_streams(0x000F)
        self.assertFalse(self.link.suspended())
        self.assertIsNotNone(self.link.send_tof_range(900))
        self.clock.advance(config.MEGA_SILENCE_MS)
        self.assertFalse(self.link.suspended())
        self.clock.advance(1)
        self.assertTrue(self.link.suspended())
        self.assertFalse(self.link.streams_allowed(link.GATE_TOF))
        self.assertIsNone(self.link.send_tof_range(900))
        self.assertIsNone(self.link.send_imu_frame(90.0, 0.0))
        self.assertIsNotNone(self.link.send_heartbeat())
        self.assertIsNotNone(self.link.send_ready(cal_cached=1, sensor_ok=0x0F))
        self.drain()
        self.assertIn(link.MSG.HEARTBEAT, decode_written(self.port))
        self.assertIn(link.MSG.READY, decode_written(self.port))
        self.assertEqual(self.link.gate_mask(), 0x000F)

    def test_first_host_frame_resumes_the_streams(self):
        self.open_streams(0x000F)
        self.clock.advance(config.MEGA_SILENCE_MS + 1)
        self.assertTrue(self.link.suspended())
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.assertFalse(self.link.suspended())
        self.assertIsNotNone(self.link.send_tof_range(900))

    def test_responder_answers_idle_while_the_host_is_silent(self):
        self.port.feed(link.encode_frame(link.MSG.MODE_SET,
                                          {"mode": link.MODE_RUN, "arg": 0}))
        self.link.pump()
        self.assertEqual(self.link.effective_mode(), link.MODE_RUN)
        self.clock.advance(config.MEGA_SILENCE_MS + 1)
        self.assertEqual(self.link.effective_mode(), link.MODE_IDLE)
        self.port.feed(link.encode_frame(link.MSG.MODE_SET,
                                          {"mode": 0x7F, "arg": 0}))
        self.link.pump()
        self.drain()
        self.assertEqual(decode_written(self.port)[link.MSG.MODE_ACK],
                         {"mode": link.MODE_IDLE, "accepted": 0})
        self.assertEqual(self.link.mode(), link.MODE_RUN)

    def test_gated_streams_need_their_gate_bit(self):
        self.assertIsNone(self.link.send_tof_range(900))
        self.assertIsNone(self.link.send_goal_blob(1, 1, 1, 1, 1))
        self.assertFalse(self.link.streams_allowed(link.GATE_VISION))


class RadarMappingTest(LinkFixture):
    """tof.py's sweep profile becomes the documented zone + class frame."""

    @staticmethod
    def sweep_profile(close_bins):
        """A tof.py-shaped profile: 900 mm on the listed sectors, else no return."""
        bins = [tof.OUT_OF_RANGE_MM] * tof.SWEEP_SAMPLE_COUNT
        for index in close_bins:
            bins[index] = 900
        outcome, retry = tof.classify(bins)
        return {"bins_mm": bins, "outcome": outcome, "retry": retry}

    def test_enclosed_sweep_maps_to_the_enclosed_class(self):
        profile = self.sweep_profile(range(6))
        self.assertEqual(profile["outcome"], tof.CLASS_ENCLOSED)
        self.assertEqual(self.link.send_radar_result_from_profile(profile),
                         (11, config.RADAR_CLASS_ENCLOSED))
        self.link.pump()
        self.link.pump()
        self.assertEqual(bytes(self.port.written),
                         radar_vector(11, config.RADAR_CLASS_ENCLOSED))

    def test_open_sweep_maps_to_the_open_class(self):
        profile = self.sweep_profile((2, 3))
        self.assertEqual(profile["outcome"], tof.CLASS_OPEN)
        self.assertEqual(self.link.send_radar_result_from_profile(profile),
                         (10, config.RADAR_CLASS_OPEN))
        self.link.pump()
        self.link.pump()
        self.assertEqual(bytes(self.port.written),
                         radar_vector(10, config.RADAR_CLASS_OPEN))

    def test_ambiguous_sweep_keeps_the_conservative_open_class(self):
        profile = self.sweep_profile((0, 1, 2, 3, 6, 7, 8, 9))
        self.assertTrue(profile["retry"])
        self.assertEqual(profile["outcome"], tof.CLASS_OPEN)
        self.assertEqual(self.link.send_radar_result_from_profile(profile),
                         (13, config.RADAR_CLASS_OPEN))

    def test_sweep_with_no_open_run_reports_no_useful_sector(self):
        profile = self.sweep_profile(range(tof.SWEEP_SAMPLE_COUNT))
        self.assertEqual(self.link.send_radar_result_from_profile(profile),
                         (config.RADAR_ZONE_NO_RESULT,
                          config.RADAR_CLASS_ENCLOSED))

    def test_sweep_is_withheld_while_the_host_is_silent(self):
        self.port.feed(link.encode_frame(link.MSG.HEARTBEAT))
        self.link.pump()
        self.clock.advance(config.MEGA_SILENCE_MS + 1)
        self.link.send_radar_result_from_profile(self.sweep_profile((2, 3)))
        self.drain()
        self.assertNotIn(link.MSG.RADAR_RESULT, decode_written(self.port))


class ConfigRegistryTest(LinkFixture):
    """The knob registry: documented defaults, tags, ranges and echo."""

    def test_documented_defaults(self):
        self.assertEqual(config.value_of(config.KEY_TOF_VALID_MAX_MM), 3500)
        self.assertEqual(config.value_of(config.KEY_BLOB_MIN_SIZE), 20)
        self.assertEqual(config.value_of(config.KEY_GOAL_CONF_MIN), 50)
        self.assertEqual(config.value_of(config.KEY_YAW_OFFSET_DEG100), 0)

    def test_push_applies_immediately_and_never_persists(self):
        self.assertTrue(config.apply_push(config.TAG_UINT32,
                                          config.KEY_TOF_VALID_MAX_MM, 3200))
        self.assertEqual(config.value_of(config.KEY_TOF_VALID_MAX_MM), 3200)
        config.reset()
        self.assertEqual(config.value_of(config.KEY_TOF_VALID_MAX_MM), 3500)

    def test_unknown_key_and_mismatched_tag_are_ignored(self):
        self.assertFalse(config.apply_push(config.TAG_UINT32, 0x00FF, 1))
        self.assertFalse(config.apply_push(9, config.KEY_GOAL_CONF_MIN, 60))
        self.assertIsNone(config.value_of(0x00FF))

    def test_out_of_range_value_is_refused(self):
        self.assertFalse(config.apply_push(config.TAG_UINT32,
                                           config.KEY_TOF_VALID_MAX_MM, 9999))
        self.assertFalse(config.apply_push(config.TAG_UINT32,
                                           config.KEY_BLOB_MIN_SIZE, 2000))
        self.assertFalse(config.apply_push(config.TAG_INT32,
                                           config.KEY_YAW_OFFSET_DEG100,
                                           1 << 31))
        self.assertEqual(config.value_of(config.KEY_TOF_VALID_MAX_MM), 3500)

    def test_echo_entries_carry_the_documented_keys_and_tags(self):
        self.assertEqual([(tag, key) for tag, key, _value
                          in config.echo_entries()],
                         [(2, 0x0001), (2, 0x0002), (2, 0x0003), (1, 0x0004)])

    def test_tof_status_follows_the_pushed_window(self):
        self.assertEqual(link.tof_status(3500), link.TOF_STATUS_VALID)
        self.assertEqual(link.tof_status(3501), link.TOF_STATUS_OUT_OF_RANGE)
        self.assertEqual(link.tof_status(-1), link.TOF_STATUS_ERROR)
        self.assertEqual(link.tof_status(config.TOF_RANGE_MAX_MM + 1),
                         link.TOF_STATUS_ERROR)
        self.assertEqual(config.yaw_offset_centi_deg(), 0)


if __name__ == "__main__":
    unittest.main()
