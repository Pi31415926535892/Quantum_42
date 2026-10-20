"""Host tests for nicla/main.py - the board orchestrator, no hardware needed.

    python3 -m unittest test.main_test
    python3 -m pytest test/main_test.py

The real link codec and the real vision run against stand-in camera, flash, ToF
and IMU seams, so the frames asserted here are the frames the wire would carry.
"""

import os
import sys
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicla import config, link, main, vision

FRAME_W = 320
FRAME_H = 240
CARD_MEANS = {"red": (188, 30, 24), "green": (30, 170, 48)}
CACHED_CAL = vision.encode_cache(
    [vision.ColorThreshold("red", 188, 30, 24, 12),
     vision.ColorThreshold("green", 30, 170, 48, 12)])
# A red patch big enough to read as the goal; small patches read as markers.
GOAL_RED = vision.Detection(160, 120, 200, 180, 200 * 180)
MARKER_RED = vision.Detection(40, 60, 40, 40, 40 * 40)
MARKER_GREEN = vision.Detection(260, 60, 40, 40, 40 * 40)
ALL_GATES = link.GATE_TOF | link.GATE_YAW | link.GATE_VISION


def sent(port):
    """Every frame the link wrote, in order, as (message id, field dict)."""
    written = bytes(port.written)
    frames, _resynced = link.FrameDecoder().feed(written, len(written))
    return [(msg_id, link.decode_fields(msg_id, payload))
            for msg_id, payload in frames]


def fields_of(port, msg_id):
    """Field dicts of every frame of ``msg_id`` the link wrote."""
    return [fields for sent_id, fields in sent(port) if sent_id == msg_id]


class InMemoryPort:
    """Delivery seam: what the link writes is what the host would read."""

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
        """Put host frames on the link's receive path."""
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


class StubCamera:
    """Camera seam double: scripted card means and detections."""

    def __init__(self, means=None, red=(), green=(), size=(FRAME_W, FRAME_H)):
        self._means = CARD_MEANS if means is None else means
        self._red, self._green, self._size = list(red), list(green), size
        self.start_calls = 0
        self.grabs = 0

    def start(self):
        self.start_calls += 1

    def size(self):
        return self._size

    def grab(self):
        self.grabs += 1

    def blobs(self, threshold):
        return list(self._red if threshold.name == "red" else self._green)

    def patch_mean(self, rect):
        return self._means["red" if rect == vision.CARD_PATCHES["red"]
                           else "green"]


class StubStorage:
    """Flash seam double: serves a stamped record, or fails on demand."""

    def __init__(self, record=CACHED_CAL, read_error=None, write_error=None):
        self._record, self._read_error, self._write_error = (
            record, read_error, write_error)
        self.writes = 0

    def read(self):
        if self._read_error is not None:
            raise self._read_error
        return self._record

    def write(self, data):
        if self._write_error is not None:
            raise self._write_error
        self.writes += 1
        self._record = data
        return True


class FakeToF:
    """ToF seam double: scripted reading and sweep, failure on demand."""

    def __init__(self, clock=None, error=None, burn_ms=0, millis=1234,
                 bins=(500,) * 9 + (2000,) * 9):
        self._clock, self._error, self._burn_ms = clock, error, burn_ms
        self._millis, self._bins = millis, list(bins)
        self.reads = 0

    def init(self):
        if self._burn_ms:
            self._clock.advance(self._burn_ms)
        if self._error is not None:
            raise self._error

    def distance_mm(self):
        self.reads += 1
        return self._millis

    def sweep(self, rotator=None):
        return {"bins_mm": list(self._bins), "outcome": "enclosed",
                "retry": False}


class FakeImu:
    """IMU seam double: a fixed attitude, failure on demand."""

    def __init__(self, error=None, heading_deg=12.5, pitch_deg=-3.25):
        self._error, self._heading, self._pitch = error, heading_deg, pitch_deg
        self.steps = 0

    def step(self, dt_s):
        self.steps += 1
        if self._error is not None:
            raise self._error

    def heading_deg(self):
        return self._heading

    def pitch_deg(self):
        return self._pitch


class ScriptedVision(vision.Vision):
    """The real vision with the calibration verdict and its cost scripted.

    ``answer`` is what the calibration returns, ``burn_ms`` what one attempt
    costs the board's clock: a slice that spends its whole slice was cut off
    before it could reach a verdict.
    """

    def __init__(self, clock, answer, burn_ms=0, **seams):
        vision.Vision.__init__(self, clock=clock, **seams)
        self._answer, self._burn_ms = answer, burn_ms
        self.attempts = 0

    def calibrate(self, deadline_ms=vision.CAL_DEADLINE_MS):
        self.attempts += 1
        self._clock.advance(self._burn_ms)
        return self._answer


class NiclaMainTest(unittest.TestCase):
    """The board orchestrator over the real link and vision, faked sensors."""

    def setUp(self):
        self.port = InMemoryPort()
        self.clock = FakeClock()

    def build(self, tof=None, imu=None, storage=None, vision_module=None,
              red=None, green=None):
        """A board on its seams: real link and vision, faked sensors.

        The camera scene is the frame the vision pass reads, so each test
        states the field it means: the goal card in the goal test, a single
        evacuation marker in the triangle tests.
        """
        if red is None and green is None:
            red, green = [GOAL_RED], [MARKER_GREEN]
        self.camera = StubCamera(red=[] if red is None else red,
                                 green=[] if green is None else green)
        self.storage = StubStorage() if storage is None else storage
        self.tof = FakeToF(clock=self.clock) if tof is None else tof
        self.imu = FakeImu() if imu is None else imu
        self.sense = (vision.Vision(camera=self.camera, storage=self.storage,
                                    clock=self.clock)
                      if vision_module is None else vision_module)
        self.wire = link.NiclaLink(self.port, self.clock)
        self.board = main.NiclaMain(link=self.wire, vision=self.sense,
                                    tof=self.tof, imu=self.imu,
                                    clock=self.clock)
        return self.board

    def sent_ids(self, msg_id):
        """Every ``msg_id`` frame the board has written so far."""
        return [fields for frame_id, fields in sent(self.port)
                if frame_id == msg_id]

    def on_wire(self, msg_id):
        """The frames of ``msg_id`` on the wire, once the FIFO has drained.

        The link hands out a couple of frames per pump, so draining takes as
        many pumps as the board queued frames.
        """
        while self.wire._tx:
            self.wire.pump()
        return fields_of(self.port, msg_id)

    def script_calibration(self, answer, burn_ms=0):
        """Rebuild the board over a vision whose calibration is scripted."""
        return self.build(vision_module=ScriptedVision(
            clock=self.clock, answer=answer, burn_ms=burn_ms,
            camera=self.camera, storage=self.storage))

    def host(self, msg_id, values=None):
        """One host frame on the wire, decoded by a pump.

        The link decodes a single frame per pump (protocol 5.5), so the host is
        fed one frame at a time: a burst would be truncated at the budget.
        """
        self.port.feed(link.encode_frame(msg_id, values or {}))
        self.wire.pump()

    def host_run(self, mask=vision.SEEK_GOAL, gate=ALL_GATES):
        """Boot, then a host that opened every stream and declared a run."""
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": gate})
        self.host(link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0})
        self.host(link.MSG.RUN_START, {"flags": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": mask})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()

    # --- boot, the self-check and READY ---

    def test_boot_announces_ready_inside_the_timeout(self):
        self.build()
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_READY)
        ready = self.on_wire(link.MSG.READY)
        self.assertEqual(ready[0]["sensor_ok"], main.SENSOR_OK_ALL)
        self.assertEqual(ready[0]["cal_cached"], 1)
        self.assertLess(self.clock(), config.LINK_READY_TIMEOUT_MS)

    def test_ready_needs_no_host_frame_at_all(self):
        self.build()
        self.board.boot()
        self.assertEqual(len(self.on_wire(link.MSG.READY)), 1)

    def test_boot_stays_idle_when_the_tof_probe_fails(self):
        self.build(tof=FakeToF(clock=self.clock, error=OSError("no tof")))
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_IDLE)
        self.assertEqual(self.on_wire(link.MSG.READY), [])
        self.assertEqual(self.on_wire(link.MSG.ERROR)[0]["info"],
                         main.FAIL_TOF)

    def test_idle_board_heartbeats_without_a_host(self):
        self.build(imu=FakeImu(error=OSError("no imu")))
        self.board.boot()
        self.assertEqual(self.on_wire(link.MSG.READY), [])
        for _ in range(3):
            self.clock.advance(config.HEARTBEAT_PERIOD_MS)
            self.board.tick()
        # One heartbeat per period, from a board the host never answered.
        self.assertEqual(len(self.on_wire(link.MSG.HEARTBEAT)), 3)

    def test_boot_stays_idle_when_the_cal_cache_is_unreadable(self):
        self.build(storage=StubStorage(read_error=OSError("no qspi")))
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_IDLE)
        self.assertEqual(self.on_wire(link.MSG.READY), [])
        self.assertEqual(self.on_wire(link.MSG.ERROR)[0]["info"],
                         main.FAIL_QSPI)

    def test_empty_cal_cache_still_readies_with_cal_cached_zero(self):
        self.build(storage=StubStorage(record=None))
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_READY)
        self.assertEqual(self.on_wire(link.MSG.READY)[0]["cal_cached"], 0)

    def test_no_blob_is_emitted_without_a_held_calibration(self):
        self.build(storage=StubStorage(record=None))
        self.host_run(mask=vision.SEEK_ALL)
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB), [])
        self.assertEqual(self.on_wire(link.MSG.TRIANGLE_BLOB), [])

    def test_boot_stays_idle_when_the_self_check_overruns_the_timeout(self):
        self.build(tof=FakeToF(clock=self.clock,
                               burn_ms=config.LINK_READY_TIMEOUT_MS))
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_IDLE)
        self.assertEqual(self.on_wire(link.MSG.READY), [])
        self.assertEqual(self.on_wire(link.MSG.ERROR)[0]["info"],
                         main.FAIL_TIMEOUT)

    # --- the match-window radio guard ---

    def test_radio_energy_is_off_in_a_clean_interpreter(self):
        self.assertIs(main.MATCH_WINDOW, True)
        self.assertTrue(main.radio_energy_off())

    def test_loaded_radio_stack_withholds_ready(self):
        self.build()
        sys.modules["network"] = types.ModuleType("network")
        self.addCleanup(sys.modules.pop, "network", None)
        self.assertFalse(main.radio_energy_off())
        self.board.boot()
        self.assertEqual(self.board.state, main.STATE_IDLE)
        self.assertEqual(self.on_wire(link.MSG.READY), [])
        self.assertEqual(self.on_wire(link.MSG.ERROR)[0]["info"],
                         main.FAIL_RADIO_ON)

    # --- calibration ---

    def test_cal_start_answers_ok_and_caches_once(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.board.tick()
        self.assertEqual([fields["status"]
                          for fields in self.on_wire(link.MSG.CAL_COMPLETE)],
                         [config.CAL_OK])
        self.assertEqual(self.storage.writes, 1)

    def test_sticky_cal_start_is_answered_only_once(self):
        self.build()
        self.script_calibration(answer=True)
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.board.tick()
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.sense.attempts, 1)
        self.assertEqual(len(self.on_wire(link.MSG.CAL_COMPLETE)), 1)

    def test_cal_abort_cancels_and_frees_the_next_request(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.board.tick()
        self.host(link.MSG.CAL_ABORT)
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual([fields["status"]
                          for fields in self.on_wire(link.MSG.CAL_COMPLETE)],
                         [config.CAL_OK, config.CAL_CANCELED, config.CAL_OK])

    def test_card_the_camera_refuses_is_reported_rejected(self):
        self.build()
        self.script_calibration(answer=False)
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.CAL_COMPLETE)[0]["status"],
                         config.CAL_REJECTED)

    def test_calibration_that_never_verdicts_times_out_as_error(self):
        self.build()
        self.script_calibration(answer=False, burn_ms=main.TICK_PERIOD_MS)
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        while not self.sent_ids(link.MSG.CAL_COMPLETE) and (
                self.clock() <= vision.CAL_DEADLINE_MS):
            self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.CAL_COMPLETE)[0]["status"],
                         config.CAL_ERROR)
        self.assertLessEqual(self.clock(), vision.CAL_DEADLINE_MS)

    def test_heartbeat_holds_its_cadence_through_a_slow_calibration(self):
        self.build()
        self.script_calibration(answer=False, burn_ms=main.TICK_PERIOD_MS)
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        while not self.sent_ids(link.MSG.CAL_COMPLETE) and (
                self.clock() <= vision.CAL_DEADLINE_MS):
            self.board.tick()
        self.assertGreaterEqual(len(self.on_wire(link.MSG.HEARTBEAT)),
                                self.clock() // config.HEARTBEAT_PERIOD_MS)

    def test_cal_flash_write_failure_is_reported(self):
        self.build(storage=StubStorage(write_error=OSError("qspi full")))
        self.board.boot()
        self.host(link.MSG.CAL_START, {"channel": 0})
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.ERROR)[0]["code"],
                         main.ERROR_CODE_CAL_FLASH)
        self.assertEqual(self.storage.writes, 0)

    # --- the blob stream ---

    def test_goal_mask_reports_the_goal_and_nothing_else(self):
        self.build()
        self.host_run(mask=vision.SEEK_GOAL)
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB)[0]["cx"], 500)
        self.assertEqual(self.on_wire(link.MSG.TRIANGLE_BLOB), [])

    def test_green_mask_reports_the_green_marker(self):
        self.build(red=[MARKER_RED], green=[MARKER_GREEN])
        self.host_run(mask=vision.SEEK_TRIANGLE_GREEN)
        triangles = self.on_wire(link.MSG.TRIANGLE_BLOB)
        self.assertEqual(triangles[0]["color"], vision.COLOR_GREEN)
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB), [])

    def test_red_mask_reports_the_red_marker(self):
        self.build(red=[MARKER_RED], green=[MARKER_GREEN])
        self.host_run(mask=vision.SEEK_TRIANGLE_RED)
        markers = self.on_wire(link.MSG.TRIANGLE_BLOB)
        self.assertEqual(markers[0]["color"], vision.COLOR_RED)
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB), [])

    def test_no_blob_before_the_run_is_declared(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES})
        self.host(link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": vision.SEEK_GOAL})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB), [])

    def test_no_blob_while_the_run_mode_is_not_set(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES})
        self.host(link.MSG.RUN_START, {"flags": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": vision.SEEK_GOAL})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB), [])

    def test_blob_stream_stops_when_the_host_goes_silent(self):
        self.build()
        self.host_run(mask=vision.SEEK_GOAL)
        seen = len(self.on_wire(link.MSG.GOAL_BLOB))
        self.clock.advance(config.MEGA_SILENCE_MS + main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(len(self.on_wire(link.MSG.GOAL_BLOB)), seen)

    # --- telemetry and the one-shot requests ---

    def test_streams_follow_their_gate_bits(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": 0})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.TOF_RANGE), [])
        self.assertEqual(self.on_wire(link.MSG.IMU_FRAME), [])
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.on_wire(link.MSG.TOF_RANGE)[0]["mm"], 1234)
        self.assertEqual(self.on_wire(link.MSG.IMU_FRAME)[0]["flags"],
                         link.IMU_FLAG_VALID)

    def test_streams_pace_their_seams_to_the_cadence(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES})
        self.host(link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0})
        self.host(link.MSG.RUN_START, {"flags": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": vision.SEEK_GOAL})
        # Counted from here, past the self-check the boot pass already ran.
        self.tof.reads, self.imu.steps, self.camera.grabs = 0, 0, 0
        # Short of a period, no seam is touched: the streams are paced, not
        # polled, so a tick can never cost more than one period of sensors.
        for _ in range(3):
            self.board.tick()
        self.assertEqual((self.tof.reads, self.imu.steps, self.camera.grabs),
                         (0, 0, 0))
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual((self.tof.reads, self.imu.steps, self.camera.grabs),
                         (1, 1, 1))

    def test_stream_gates_spare_the_seams_they_shut(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": link.GATE_VISION})
        self.host(link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0})
        self.host(link.MSG.RUN_START, {"flags": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": vision.SEEK_GOAL})
        self.tof.reads, self.imu.steps, self.camera.grabs = 0, 0, 0
        for _ in range(2):
            self.clock.advance(main.TICK_PERIOD_MS)
            self.board.tick()
        # A shut stream costs the board nothing: the ToF and the IMU are not
        # read at all, while the vision pass the host asked for still runs.
        self.assertEqual((self.tof.reads, self.imu.steps), (0, 0))
        self.assertEqual(self.camera.grabs, 2)

    def test_blob_stream_spares_the_camera_while_its_gate_is_shut(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES & ~link.GATE_VISION})
        self.host(link.MSG.MODE_SET, {"mode": link.MODE_RUN, "arg": 0})
        self.host(link.MSG.RUN_START, {"flags": 0})
        self.host(link.MSG.REQUEST_BLOB, {"mask": vision.SEEK_GOAL})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.camera.grabs, 0)
        self.host(link.MSG.SENSOR_GATE, {"mask": ALL_GATES})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertEqual(self.camera.grabs, 1)
        self.assertEqual(self.on_wire(link.MSG.GOAL_BLOB)[0]["cx"], 500)

    def test_request_range_is_answered_once(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.SENSOR_GATE, {"mask": link.GATE_TOF})
        self.host(link.MSG.REQUEST_RANGE)
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        # One reading for the stream's turn, one for the host's request.
        self.assertEqual(len(self.on_wire(link.MSG.TOF_RANGE)), 2)
        # Off the stream cadence, a spent request asks nothing more.
        self.board.tick()
        self.assertEqual(self.tof.reads, 2)

    def test_request_radar_answers_one_sweep(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.REQUEST_RADAR, {"step_deg": 0, "sectors": 0})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        # The nine close bins are the tail of the sweep, whose best sector is
        # the midpoint of that run.
        self.assertEqual(self.on_wire(link.MSG.RADAR_RESULT)[0],
                         {"zone": 13, "class": config.RADAR_CLASS_ENCLOSED})

    def test_lop_radar_assist_is_remembered_from_run_start(self):
        self.build()
        self.board.boot()
        self.host(link.MSG.RUN_START, {"flags": 0x0001})
        self.clock.advance(main.TICK_PERIOD_MS)
        self.board.tick()
        self.assertTrue(self.board.resume_lop())

    # --- the main loop ---

    def test_run_forever_ticks_and_yields_until_the_device_stops_it(self):
        self.build()
        self.board.boot()
        yields = []

        def idle():
            yields.append(self.clock())
            if len(yields) == 3:
                raise StopIteration

        with self.assertRaises(StopIteration):
            main.run_forever(self.board, idle)
        self.assertEqual(len(yields), 3)


if __name__ == "__main__":
    unittest.main()
