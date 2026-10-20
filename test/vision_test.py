"""Host tests for nicla/vision.py - synthetic frames, no device needed.

    python3 -m unittest test.vision_test
    python3 -m pytest test/vision_test.py
"""

import contextlib
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicla import config, link, vision

FRAME_W = 320
FRAME_H = 240
COORD_MAX = config.COORD_MAX
WHITE_RGB = (235, 235, 235)
FLOOR_RGB = (210, 208, 205)
RED_RGB = (188, 30, 24)
GREEN_RGB = (30, 170, 48)
DARK_RGB = (10, 11, 14)


def raster(fill=FLOOR_RGB, width=FRAME_W, height=FRAME_H):
    """A synthetic RGB frame of one flat colour."""
    return [[fill for _ in range(width)] for _ in range(height)]


def pixels(left, top, width, height):
    """A pixel box expressed in the normalised frame scale."""
    return (left * COORD_MAX // FRAME_W, top * COORD_MAX // FRAME_H,
            (left + width) * COORD_MAX // FRAME_W,
            (top + height) * COORD_MAX // FRAME_H)


def center_box(width, height):
    """A pixel box centred in the frame."""
    return pixels((FRAME_W - width) // 2, (FRAME_H - height) // 2, width, height)


def pixel_rect(frame, rect):
    """A normalised rect as (left, top, right, bottom) pixel indices."""
    height, width = len(frame), len(frame[0])
    return (rect[0] * width // COORD_MAX, rect[1] * height // COORD_MAX,
            rect[2] * width // COORD_MAX, rect[3] * height // COORD_MAX)


def paint(frame, rect, color):
    """Fill a normalised (left, top, right, bottom) rect with ``color``."""
    left, top, right, bottom = pixel_rect(frame, rect)
    for y in range(max(0, top), min(len(frame), bottom)):
        for x in range(max(0, left), min(len(frame[y]), right)):
            frame[y][x] = color


def scatter(frame, rect, color, step=4):
    """A connected comb: one wide region with low fill."""
    left, top, right, bottom = pixel_rect(frame, rect)
    for y in range(top, bottom, step):
        for x in range(left, right):
            frame[y][x] = color
    for y in range(top, bottom):
        frame[y][left] = color


def card_frame(red, green):
    """A held calibration card showing the board's red and green patches."""
    frame = raster(WHITE_RGB)
    for name, color in (("red", red), ("green", green)):
        paint(frame, vision.CARD_PATCHES[name], color)
    return frame


def card_frames(red=RED_RGB, green=GREEN_RGB, count=vision.CAL_SAMPLE_FRAMES_MAX):
    """A repeatable card capture, one frame per calibration sample."""
    return [card_frame(red, green) for _ in range(count)]


def scene(patches=(), fill=FLOOR_RGB):
    """A scene frame with ``patches`` = ((normalised rect, colour), ...)."""
    frame = raster(fill)
    for rect, color in patches:
        paint(frame, rect, color)
    return frame


class StubCamera:
    """Camera seam double: serves synthetic rasters, no device involved."""

    def __init__(self, frames=(), size=(FRAME_W, FRAME_H)):
        self._frames = list(frames)
        self._size = size
        self._frame = None
        self.start_calls = 0
        self.grab_calls = 0

    def start(self):
        self.start_calls += 1

    def size(self):
        return self._size

    def grab(self):
        self.grab_calls += 1
        if len(self._frames) > 1:
            self._frame = self._frames.pop(0)
        elif self._frames:
            self._frame = self._frames[0]
        return self._frame

    def blobs(self, threshold):
        """Connected regions of matching pixels, as frame-pixel detections."""
        seen, found = set(), []
        for y, row in enumerate(self._frame):
            for x in range(len(row)):
                if (x, y) in seen or not self._matches(row[x], threshold):
                    continue
                found.append(self._flood(x, y, threshold, seen))
        return found

    def _flood(self, start_x, start_y, threshold, seen):
        """One connected region's bounding box and its pixel count."""
        stack, xs, ys, count = [(start_x, start_y)], [], [], 0
        while stack:
            x, y = stack.pop()
            if not (0 <= x < self._size[0] and 0 <= y < self._size[1]):
                continue
            if (x, y) in seen or not self._matches(self._frame[y][x], threshold):
                continue
            seen.add((x, y))
            xs.append(x)
            ys.append(y)
            count += 1
            stack.extend(((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)))
        return vision.Detection(sum(xs) // count, sum(ys) // count,
                                max(xs) - min(xs) + 1, max(ys) - min(ys) + 1, count)

    @staticmethod
    def _matches(pixel, threshold):
        return all(abs(pixel[channel] - getattr(threshold, name)) <= threshold.tolerance
                   for channel, name in enumerate(("red", "green", "blue")))

    def patch_mean(self, rect):
        """Mean RGB of a normalised card patch, None when it is off-frame."""
        left, top, right, bottom = rect
        x0, x1 = left * self._size[0] // COORD_MAX, right * self._size[0] // COORD_MAX
        y0, y1 = top * self._size[1] // COORD_MAX, bottom * self._size[1] // COORD_MAX
        sums, count = [0, 0, 0], 0
        for y in range(y0, min(y1, self._size[1])):
            for x in range(x0, min(x1, self._size[0])):
                for channel in range(3):
                    sums[channel] += self._frame[y][x][channel]
                count += 1
        return None if not count else tuple(total // count for total in sums)


class StubStorage:
    """Flash seam double: the whole cache record lives in one buffer."""

    def __init__(self, data=b""):
        self.data = data

    def read(self):
        return self.data

    def write(self, data):
        self.data = data
        return len(data)


class SteppingClock:
    """Millisecond clock that jumps a fixed step on every read."""

    def __init__(self, step_ms):
        self._now = 0
        self._step = step_ms

    def __call__(self):
        self._now += self._step
        return self._now


class VisionTestCase(unittest.TestCase):
    """Shared fixture: a vision primed from a card, then fed scene frames."""

    def setUp(self):
        config.reset()

    def calibrated(self, scene_frames=(), card=None, clock=None, storage=None):
        """A calibrated vision over a stub camera showing card, then scene."""
        camera = StubCamera(list(card or card_frames()) + list(scene_frames))
        core = vision.Vision(camera=camera, storage=storage, clock=clock)
        self.assertTrue(core.calibrate())
        return core, camera


class GoalBlobTest(VisionTestCase):

    def test_sized_centred_red_patch_reports_a_goal_blob(self):
        core, _camera = self.calibrated(
            [scene([(center_box(96, 96), RED_RGB)])])
        emissions = core.produce(vision.SEEK_GOAL)
        self.assertEqual(len(emissions), 1)
        payload = emissions[0].payload
        self.assertAlmostEqual(payload["cx"], 500, delta=5)
        self.assertAlmostEqual(payload["cy"], 500, delta=5)
        self.assertEqual(payload["width"], 300)
        self.assertEqual(payload["height"], 400)
        self.assertEqual(payload["confidence"], 100)

    def test_goal_payload_stays_inside_the_wire_scale(self):
        core, _camera = self.calibrated(
            [scene([(pixels(260, 200, 96, 96), RED_RGB)])])
        payload = core.produce(vision.SEEK_GOAL)[0].payload
        for field in ("cx", "cy", "width", "height"):
            self.assertTrue(0 <= payload[field] <= COORD_MAX, field)
        self.assertTrue(0 <= payload["confidence"] <= config.CONFIDENCE_MAX_PCT)

    def test_payload_fields_round_trip_through_the_link_codec(self):
        core, _camera = self.calibrated([scene([(center_box(96, 96), RED_RGB)]),
                                         scene([(pixels(40, 40, 30, 30), GREEN_RGB)])])
        for mask, msg_id, fields in ((vision.SEEK_GOAL, link.MSG.GOAL_BLOB,
                                      {"cx", "cy", "width", "height", "confidence"}),
                                     (vision.SEEK_TRIANGLE_GREEN,
                                      link.MSG.TRIANGLE_BLOB,
                                      {"color", "cx", "cy", "size"})):
            emission = core.produce(mask)[0]
            self.assertEqual(emission.msg_id, msg_id)
            self.assertEqual(set(emission.payload), fields)
            frame = link.encode_frame(emission.msg_id, emission.payload)
            self.assertIsNotNone(frame)
            self.assertEqual(link.decode_fields(msg_id, frame[4:-1]),
                             emission.payload)

    def test_message_names_and_ids_are_the_protocol_ones(self):
        core, _camera = self.calibrated([scene([(center_box(96, 96), RED_RGB)]),
                                         scene([(pixels(40, 40, 30, 30), RED_RGB)])])
        goal = core.produce(vision.SEEK_GOAL)[0]
        triangle = core.produce(vision.SEEK_TRIANGLE_RED)[0]
        self.assertEqual((goal.name, goal.msg_id), ("GOAL_BLOB", 0x03))
        self.assertEqual((triangle.name, triangle.msg_id),
                         ("TRIANGLE_BLOB", 0x04))


class ConfidenceAndSizeGateTest(VisionTestCase):

    def test_low_confidence_scratter_below_the_floor_is_dropped(self):
        frame = scene()
        scatter(frame, pixels(0, 0, 160, 120), RED_RGB)
        core, _camera = self.calibrated([frame])
        self.assertIsNone(core.produce(vision.SEEK_GOAL))

    def test_speck_below_the_minimum_blob_size_is_dropped(self):
        core, _camera = self.calibrated(
            [scene([(pixels(150, 100, 3, 3), GREEN_RGB)])])
        self.assertIsNone(core.produce(vision.SEEK_TRIANGLE_GREEN))

    def test_low_light_frame_reports_nothing(self):
        core, camera = self.calibrated([scene(fill=DARK_RGB)])
        self.assertIsNone(core.produce())
        self.assertTrue(camera.grab_calls > 3)

    def test_uncalibrated_vision_emits_nothing(self):
        camera = StubCamera([scene([(center_box(96, 96), RED_RGB)])])
        core = vision.Vision(camera=camera)
        self.assertFalse(core.calibrated())
        self.assertIsNone(core.produce())
        self.assertEqual(camera.grab_calls, 0)


class TriangleBlobTest(VisionTestCase):

    def test_red_and_green_markers_flip_the_color_byte(self):
        core, _camera = self.calibrated([
            scene([(pixels(40, 40, 30, 30), RED_RGB)]),
            scene([(pixels(40, 40, 30, 30), GREEN_RGB)]),
        ])
        red = core.produce(vision.SEEK_TRIANGLE_RED)[0].payload
        green = core.produce(vision.SEEK_TRIANGLE_GREEN)[0].payload
        self.assertEqual(red["color"], vision.COLOR_RED)
        self.assertEqual(green["color"], vision.COLOR_GREEN)
        self.assertEqual((red["cx"], red["cy"], red["size"]),
                         (green["cx"], green["cy"], green["size"]))

    def test_sweep_prefers_the_goal_then_falls_back_to_a_red_marker(self):
        core, _camera = self.calibrated([
            scene([(center_box(96, 96), RED_RGB)]),
            scene([(pixels(40, 40, 30, 30), RED_RGB)]),
        ])
        self.assertEqual(core.produce()[0].name, "GOAL_BLOB")
        emission = core.produce()[0]
        self.assertEqual(emission.name, "TRIANGLE_BLOB")
        self.assertEqual(emission.payload["color"], vision.COLOR_RED)

    def test_largest_candidate_of_a_colour_is_reported(self):
        core, _camera = self.calibrated([scene([
            (pixels(20, 20, 20, 20), RED_RGB),
            (pixels(230, 220, 60, 60), RED_RGB),
        ])])
        self.assertGreater(core.produce(vision.SEEK_GOAL)[0].payload["cx"], 500)

    def test_mask_zero_asks_for_nothing(self):
        core, camera = self.calibrated([scene([(center_box(96, 96), RED_RGB)])])
        self.assertIsNone(core.produce(0))
        self.assertEqual(camera.grab_calls, vision.CAL_SAMPLE_FRAMES_MAX)


class CalibrationTest(VisionTestCase):

    def test_card_frames_produce_usable_thresholds(self):
        camera = StubCamera(card_frames(count=3))
        core = vision.Vision(camera=camera)
        self.assertTrue(core.calibrate())
        red = core.thresholds()["red"]
        self.assertEqual((red.red, red.green, red.blue), RED_RGB)
        self.assertTrue(vision.CAL_TOLERANCE_MIN <= red.tolerance
                        <= vision.CAL_TOLERANCE_MAX)

    def test_thresholds_learned_from_a_card_detect_the_same_colours(self):
        camera = StubCamera(card_frames() + [scene([(center_box(96, 96), RED_RGB)])])
        core = vision.Vision(camera=camera)
        self.assertTrue(core.calibrate())
        self.assertEqual(core.produce(vision.SEEK_GOAL)[0].name, "GOAL_BLOB")

    def test_insufficient_contrast_card_is_rejected(self):
        camera = StubCamera(card_frames(red=(120, 60, 50), green=(124, 62, 53)))
        core = vision.Vision(camera=camera)
        self.assertFalse(core.calibrate())
        self.assertFalse(core.calibrated())

    def test_noisy_card_samples_are_rejected(self):
        camera = StubCamera(card_frames()[:1] + card_frames(red=(188, 30, 64)))
        core = vision.Vision(camera=camera)
        self.assertFalse(core.calibrate())

    def test_calibration_stops_at_the_deadline(self):
        camera = StubCamera(card_frames(count=vision.CAL_SAMPLE_FRAMES_MAX))
        core = vision.Vision(camera=camera,
                             clock=SteppingClock(30000))
        self.assertTrue(core.calibrate())
        self.assertLess(camera.grab_calls, vision.CAL_SAMPLE_FRAMES_MAX)

    def test_calibration_without_time_is_rejected(self):
        camera = StubCamera(card_frames(count=vision.CAL_SAMPLE_FRAMES_MAX))
        core = vision.Vision(camera=camera, clock=SteppingClock(100000))
        self.assertFalse(core.calibrate())
        self.assertEqual(camera.grab_calls, 1)


class CalibrationCacheTest(VisionTestCase):

    def _saved_storage(self):
        storage = StubStorage()
        core, _camera = self.calibrated(storage=storage)
        self.assertTrue(core.save_cache())
        return storage

    def test_cache_round_trips_through_the_storage_seam(self):
        storage = self._saved_storage()
        restored = vision.Vision(camera=StubCamera(), storage=storage)
        self.assertTrue(restored.load_cache())
        self.assertTrue(restored.calibrated())
        self.assertEqual(set(restored.thresholds()), {"red", "green"})

    def test_saved_record_carries_the_stamp_and_version(self):
        storage = self._saved_storage()
        self.assertEqual(storage.data[:len(vision.CACHE_MAGIC)], vision.CACHE_MAGIC)
        self.assertEqual(storage.data[len(vision.CACHE_MAGIC)], vision.CACHE_FORMAT)

    def test_torn_write_is_detected_and_ignored(self):
        for damaged in (self._saved_storage().data[:-1],
                        b"",
                        b"XXXX" + self._saved_storage().data[4:]):
            core = vision.Vision(camera=StubCamera(), storage=StubStorage(damaged))
            self.assertFalse(core.load_cache())
            self.assertFalse(core.calibrated())

    def test_save_without_a_store_or_a_calibration_reports_failure(self):
        core, _camera = self.calibrated()
        self.assertFalse(core.save_cache())
        bare = vision.Vision(camera=StubCamera(), storage=StubStorage())
        self.assertFalse(bare.save_cache())


class OpenMvCameraTest(VisionTestCase):
    """The device adapter, driven by stand-in sensor/image modules."""

    def setUp(self):
        super().setUp()
        self.blob = FakeBlob(40, 30, 10, 12, 90)
        self.frame = FakeFrame([self.blob], FakeStats(200, 100, 50))
        self.sensor = FakeSensor(self.frame)
        self.image = FakeImage()
        with self.modules():
            self.camera = vision.OpenMvCamera()
            self.camera.start()

    @contextlib.contextmanager
    def modules(self):
        sys.modules["sensor"] = self.sensor
        sys.modules["image"] = self.image
        try:
            yield
        finally:
            del sys.modules["sensor"]
            del sys.modules["image"]

    def test_start_initialises_the_sensor_and_publishes_the_frame_size(self):
        self.assertEqual(self.sensor.calls,
                         ["reset", ("pixformat", FakeSensor.RGB565),
                          ("framesize", (vision.FRAME_W, vision.FRAME_H))])
        self.assertEqual(self.camera.size(), (vision.FRAME_W, vision.FRAME_H))

    def test_blobs_become_detections_with_a_lab_threshold(self):
        threshold = vision.ColorThreshold("red", 10, 20, 30, 12)
        self.camera.grab()
        detections = self.camera.blobs(threshold)
        self.assertEqual(self.frame.thresholds,
                         [self.image.lab_of((10, 20, 30), vision.LAB_TOLERANCE)])
        self.assertEqual(self.frame.pixels_threshold, vision.BLOB_PIXELS_MIN)
        self.assertEqual([(d.cx, d.cy, d.width, d.height, d.pixels)
                          for d in detections],
                         [(40, 30, 10, 12, 90)])

    def test_card_patch_means_come_from_the_pixel_roi(self):
        self.camera.grab()
        patch = vision.CARD_PATCHES["red"]
        self.assertEqual(self.camera.patch_mean(patch), (200, 100, 50))
        left = patch[0] * vision.FRAME_W // COORD_MAX
        top = patch[1] * vision.FRAME_H // COORD_MAX
        self.assertEqual(self.frame.roi, (left, top,
                                          patch[2] * vision.FRAME_W // COORD_MAX - left,
                                          patch[3] * vision.FRAME_H // COORD_MAX - top))

    def test_patch_off_the_frame_reports_no_mean(self):
        self.camera.grab()
        self.assertIsNone(self.camera.patch_mean((0, 0, 0, 0)))
        self.assertIsNone(self.camera.patch_mean((COORD_MAX, 0, COORD_MAX, 100)))


class CameraSeamTest(VisionTestCase):

    def test_start_forwards_to_the_camera(self):
        camera = StubCamera([scene()])
        vision.Vision(camera=camera).start()
        self.assertEqual(camera.start_calls, 1)

    def test_produce_grabs_exactly_one_frame_per_call(self):
        core, camera = self.calibrated([scene([(center_box(96, 96), RED_RGB)])])
        core.produce()
        self.assertEqual(camera.grab_calls, vision.CAL_SAMPLE_FRAMES_MAX + 1)
        core.produce()
        self.assertEqual(camera.grab_calls, vision.CAL_SAMPLE_FRAMES_MAX + 2)

    def test_a_camera_without_a_frame_size_reports_nothing(self):
        core, camera = self.calibrated([scene([(center_box(96, 96), RED_RGB)])])
        camera._size = (0, 0)
        self.assertIsNone(core.produce())


class _MeanReader:
    """Answers a named getter with a callable over one tuple of values."""

    def __init__(self, names, values):
        self._names = names
        self._values = values

    def __getattr__(self, name):
        value = self._values[self._names.index(name)]
        return lambda: value


class FakeBlob(_MeanReader):
    """An OpenMV blob: the pixel measurements the adapter reads."""

    def __init__(self, *values):
        _MeanReader.__init__(self, ("cx", "cy", "w", "h", "pixels"), values)


class FakeStats(_MeanReader):
    """An OpenMV statistics block: the per-channel means."""

    def __init__(self, red, green, blue):
        _MeanReader.__init__(self, ("r_mean", "g_mean", "b_mean"),
                             (red, green, blue))


class FakeFrame:

    def __init__(self, blobs=(), stats=None):
        self._blobs = blobs
        self._stats = stats
        self.thresholds = None
        self.pixels_threshold = None
        self.roi = None

    def find_blobs(self, thresholds, pixels_threshold=0, merge=False):
        self.thresholds = list(thresholds)
        self.pixels_threshold = pixels_threshold
        return self._blobs

    def get_statistics(self, roi=None):
        self.roi = roi
        return self._stats


class FakeSensor:
    RGB565 = 2

    def __init__(self, frame):
        self._frame = frame
        self.calls = []

    def reset(self):
        self.calls.append("reset")

    def set_pixformat(self, pixformat):
        self.calls.append(("pixformat", pixformat))

    def set_framesize(self, framesize):
        self.calls.append(("framesize", framesize))

    def snapshot(self):
        return self._frame


class FakeImage:

    @staticmethod
    def rgb_to_lab(rgb):
        return tuple(component * 2 for component in rgb)

    @staticmethod
    def lab_of(rgb, tolerance):
        return tuple(component * 2 for component in rgb) + (tolerance,)


if __name__ == "__main__":
    unittest.main(verbosity=2)
