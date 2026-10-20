"""Nicla board entrypoint: self-check to READY, then the main-loop pass.

The board is a responder: it never waits for the host, so this module is
orchestration only. Every camera, flash, ToF and IMU seam is injected, so the
device script builds the hardware adapters and the host tests build stand-ins:
nothing here imports a device module and nothing here decides sensor policy --
the link gates the streams, the vision module gates the classes, the message
tables gate the shapes.

READY is the one thing the board decides alone: it is sent once the camera, the
ToF, the IMU and the QSPI calibration cache have all answered inside the ready
timeout, whatever the host has or has not said. A self-check that fails or
overruns leaves the board IDLE, reporting the reason and heartbeating, so the
host times out on its own schedule.
"""

import sys

from . import config
from .link import GATE_TOF, GATE_VISION, GATE_YAW, MSG, MODE_RUN
from .vision import CAL_DEADLINE_MS, GOAL_BLOB, TRIANGLE_BLOB

STATE_BOOT = "boot"
STATE_IDLE = "idle"
STATE_READY = "ready"

# READY.sensor_ok bits (protocol 4.3): one bit per self-check step.
SENSOR_OK_CAMERA = 0x01
SENSOR_OK_TOF = 0x02
SENSOR_OK_IMU = 0x04
SENSOR_OK_QSPI = 0x08
SENSOR_OK_ALL = SENSOR_OK_CAMERA | SENSOR_OK_TOF | SENSOR_OK_IMU | SENSOR_OK_QSPI
# ERROR codes (protocol 4.3) and the self-check step each info word names.
ERROR_CODE_INTERNAL = 4
ERROR_CODE_CAL_FLASH = 3
FAIL_CAMERA = 1
FAIL_TOF = 2
FAIL_IMU = 3
FAIL_QSPI = 4
FAIL_TIMEOUT = 5
FAIL_RADIO_ON = 6
# One period of main-loop work, taken from the link's heartbeat period: the
# sensor cadence and the calibration slice both ride it, so a camera busy with
# the calibration card cannot cost the link a heartbeat.
TICK_PERIOD_MS = config.HEARTBEAT_PERIOD_MS

# Frozen for the match window: the 2.4 GHz radios stay off. The flag is a one-way
# door -- clearing it fails the guard instead of re-enabling anything, and no
# module here loads the stacks it names.
MATCH_WINDOW = True
RADIO_MODULES = ("network", "bluetooth", "wifi")

# RUN_START bit0 (protocol 4.2): the host declared a run that expects the radar
# assist, which is what makes a sweep answerable after a LoP power cut.
RUN_FLAG_RESUME_LOP = 0x0001
def _millis():
    """Board milliseconds: the microsecond timer on device, monotonic on host."""
    import time
    ticks = getattr(time, "ticks_ms", None)
    return ticks() if ticks else int(time.monotonic() * 1000)


def radio_energy_off():
    """True while the match window holds and no radio stack is loaded."""
    return MATCH_WINDOW and not any(name in sys.modules for name in RADIO_MODULES)


def run_forever(controller, idle):
    """The board main loop: tick, yield, repeat. ``idle`` is the device sleep."""
    while True:
        controller.tick()
        idle()


def _reaches(step):
    """Run a self-check probe; True when the seam answered without raising.

    A seam that answers is present: an empty calibration cache is a readable
    flash with nothing in it, which readies with ``cal_cached`` clear, while a
    flash that raises is a board that cannot pass its self-check.
    """
    try:
        step()
    except Exception:            # an absent or wedged seam is not an answer
        return False
    return True


class NiclaMain:
    """The board-side orchestration: self-check, host commands, sensor streams."""

    def __init__(self, link, vision, tof, imu, clock=None):
        self._link = link
        self._vision = vision
        self._tof = tof
        self._imu = imu
        self._clock = _millis if clock is None else clock
        self.state = STATE_BOOT
        self._resume_lop = False
        self._run_started = False
        self._cal_started_ms = None
        self._cal_answered = False
        self._tof_ms = self._imu_ms = self._blobs_ms = self._clock()

    # --- boot: the self-check the handshake waits on ---

    def boot(self):
        """Run the self-check and, inside the ready timeout, announce READY."""
        if not radio_energy_off():
            self._stay_idle(FAIL_RADIO_ON)
        else:
            started = self._clock()
            sensor_ok, reason = self._self_check(started)
            if sensor_ok == SENSOR_OK_ALL and self._budget_left(started):
                self.state = STATE_READY
                self._link.send_ready(self._vision.calibrated(), sensor_ok)
            else:
                self._stay_idle(reason if reason is not None else FAIL_TIMEOUT)
        self._link.pump()
        return self.state

    def _self_check(self, started_ms):
        """Camera, ToF, IMU and QSPI in turn; (sensor_ok, failure reason)."""
        sensor_ok, reason = 0, None
        for bit, failure, step in self._check_steps():
            if reason is not None or not self._budget_left(started_ms):
                reason = reason if reason is not None else FAIL_TIMEOUT
            elif _reaches(step):
                sensor_ok |= bit
            else:
                reason = failure
            self._link.pump()
        return sensor_ok, reason

    def _check_steps(self):
        """The self-check in order: sensor bit, its failure code, what it runs."""
        return ((SENSOR_OK_CAMERA, FAIL_CAMERA, self._vision.start),
                (SENSOR_OK_TOF, FAIL_TOF, self._tof.init),
                (SENSOR_OK_IMU, FAIL_IMU, lambda: self._imu.step(0.0)),
                (SENSOR_OK_QSPI, FAIL_QSPI, self._vision.load_cache))

    def _budget_left(self, started_ms):
        """True while the ready timeout the host is armed with has not gone."""
        return self._clock() - started_ms < config.LINK_READY_TIMEOUT_MS

    def _stay_idle(self, reason):
        """Hold IDLE, say why on the wire, and keep the heartbeat coming."""
        self.state = STATE_IDLE
        self._link.send_error(ERROR_CODE_INTERNAL, reason)
        self._link.pump()
        return self.state

    def resume_lop(self):
        """True once the host declared a run that expects the radar assist."""
        return self._resume_lop

    # --- the main-loop pass ---

    def tick(self):
        """One pass: link, host commands, then the sensor streams."""
        now = self._link.pump()
        self._take_run_start()
        self._run_calibration()
        self._answer_one_shots()
        self._stream_tof(now)
        self._stream_imu(now)
        self._stream_blobs(now)
        return self.state

    def _take_run_start(self):
        """Latch the host's run declaration once; its cache entry is sticky."""
        request = self._link.last_good(MSG.RUN_START)
        if request is not None and not self._run_started:
            self._run_started = True
            self._resume_lop = bool(request["flags"] & RUN_FLAG_RESUME_LOP)

    def _host_running(self):
        """True while the host declared a run and has not left the link IDLE."""
        return self._run_started and self._link.effective_mode() == MODE_RUN

    # --- the calibration the host asks for ---

    def _run_calibration(self):
        """One bounded slice of the host's calibration request, per tick."""
        if not self._cal_request_pending():
            return
        verdict = self._calibrate_slice()
        if verdict is None:
            return
        self._cal_answered = True
        self._link.send_cal_complete(verdict)
        if verdict == config.CAL_OK:
            self._cache_calibration()

    def _cal_request_pending(self):
        """True while the host's sticky CAL_START is unanswered.

        The link holds CAL_START until a CAL_ABORT voids it, so the request is
        spent here once, and the one after an abort is picked up afresh.
        """
        requested = self._link.last_good(MSG.CAL_START) is not None
        if not requested:
            self._cal_started_ms = None
            self._cal_answered = False
        elif self._cal_started_ms is None:
            self._cal_started_ms = self._clock()
        return requested and not self._cal_answered

    def _calibrate_slice(self):
        """One slice of the card: a verdict, or None while time is worth it.

        A slice that spent its whole slice was cut off before the vision module
        could reach a verdict, so it asks for another; one that came back early
        has seen the whole card and refused it.
        """
        started = self._clock()
        try:
            if self._vision.calibrate(TICK_PERIOD_MS):
                return config.CAL_OK
        except Exception:        # a wedged camera cannot learn the arena
            return config.CAL_ERROR
        if self._clock() - started < TICK_PERIOD_MS:
            return config.CAL_REJECTED
        if self._clock() - self._cal_started_ms >= CAL_DEADLINE_MS:
            return config.CAL_ERROR
        return None

    def _cache_calibration(self):
        """Stamp the calibration into flash once, after the full capture.

        A flash that refuses the write is reported, not raised: the card is
        still calibrated for this match, and a full chip must not take the
        main loop down with it.
        """
        try:
            stored = self._vision.save_cache()
        except Exception:        # a wedged or full chip
            stored = False
        if stored is False:
            self._link.send_error(ERROR_CODE_CAL_FLASH, 0)

    # --- the two one-shot host requests ---

    def _answer_one_shots(self):
        """Answer the one-shot range and the one-shot radar sweep."""
        if self._link.take_range_request():
            self._link.send_tof_range(self._tof.distance_mm())
        if self._link.radar_request() is not None and self._radar_wanted():
            self._link.send_radar_result_from_profile(self._tof.sweep())

    def _radar_wanted(self):
        """A sweep is answered with the radar stream on, or after an LoP."""
        return config.ENABLE_RADAR_STREAM or self.resume_lop()

    # --- the periodic sensor streams ---

    def _stream_tof(self, now_ms):
        """Forward one range reading, on the cadence, while the host wants it."""
        if not (config.ENABLE_TOF_STREAM
                and self._link.streams_allowed(GATE_TOF)):
            return
        if now_ms - self._tof_ms < TICK_PERIOD_MS:
            return
        self._tof_ms = now_ms
        self._link.send_tof_range(self._tof.distance_mm())

    def _stream_imu(self, now_ms):
        """Step the IMU and forward the attitude it reached, on the cadence."""
        if not (config.ENABLE_IMU_STREAM
                and self._link.streams_allowed(GATE_YAW)):
            return
        elapsed_ms = now_ms - self._imu_ms
        if elapsed_ms < TICK_PERIOD_MS:
            return
        self._imu_ms = now_ms
        self._imu.step(elapsed_ms / 1000.0)
        self._link.send_imu_frame(self._imu.heading_deg(), self._imu.pitch_deg())

    def _stream_blobs(self, now_ms):
        """One vision pass, forwarded as the frames the host's mask asked for.

        The blob stream is the mission data flow, so it follows the run
        declaration as well as the host's vision gate.
        """
        if not (config.ENABLE_VISION_STREAM and self._host_running()
                and self._link.streams_allowed(GATE_VISION)):
            return
        if now_ms - self._blobs_ms < TICK_PERIOD_MS:
            return
        self._blobs_ms = now_ms
        for emission in self._vision.produce(self._blob_mask()) or ():
            if emission.name == GOAL_BLOB:
                self._link.send_goal_blob(**emission.payload)
            elif emission.name == TRIANGLE_BLOB:
                self._link.send_triangle_blob(**emission.payload)

    def _blob_mask(self):
        """The REQUEST_BLOB mask, 0 while the host has asked for nothing."""
        request = self._link.last_good(MSG.REQUEST_BLOB)
        return 0 if request is None else request["mask"]
