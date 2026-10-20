"""Pure-Python unit tests for nicla/tof.py.

Runs on macOS (no MicroPython runtime needed):
    python3 -m unittest test.tof_test
    python3 -m pytest test/tof_test.py   (if pytest is installed)
"""

import unittest

from nicla.tof import (
    CLASS_ENCLOSED,
    CLASS_OPEN,
    EFFECTIVE_MAX_MM,
    ENCLOSED_MIN_RUN,
    SWEEP_RETRY_MAX,
    SWEEP_SAMPLE_COUNT,
    SWEEP_STEP_DEG,
    ToFSensor,
    classify,
)


class StubDriver:
    """Injectable seam double: serves canned raw readings, then out-of-range."""

    def __init__(self, readings=()):
        self._readings = list(readings)
        self.init_calls = 0
        self.read_calls = 0

    def init(self):
        self.init_calls += 1

    def read_range(self):
        raw = self._readings[self.read_calls] if self.read_calls < len(self._readings) else None
        self.read_calls += 1
        return raw


class FakeRotator:
    """Records the rotation steps requested between range samples."""

    def __init__(self):
        self.steps = []

    def step(self, deg):
        self.steps.append(deg)


class ToFSensorApiTest(unittest.TestCase):

    def test_distance_mm_converts_and_rounds_ranges(self):
        sensor = ToFSensor(driver=StubDriver([423.4, 423.6, 0]))
        sensor.init()
        self.assertEqual(sensor.distance_mm(), 423)
        self.assertEqual(sensor.distance_mm(), 424)
        self.assertEqual(sensor.distance_mm(), 0)

    def test_out_of_range_and_error_readings_bin_to_effective_max(self):
        sensor = ToFSensor(driver=StubDriver([None, -1, 4155]))
        sensor.init()
        self.assertEqual(sensor.distance_mm(), EFFECTIVE_MAX_MM)
        self.assertEqual(sensor.distance_mm(), EFFECTIVE_MAX_MM)
        self.assertEqual(sensor.distance_mm(), EFFECTIVE_MAX_MM)

    def test_init_forwards_to_driver_seam(self):
        driver = StubDriver([])
        ToFSensor(driver=driver).init()
        self.assertEqual(driver.init_calls, 1)

    def test_sweep_samples_18_bins_at_20_degree_steps(self):
        driver = StubDriver([EFFECTIVE_MAX_MM] * SWEEP_SAMPLE_COUNT)
        rotator = FakeRotator()
        sensor = ToFSensor(driver=driver)
        sensor.init()
        profile = sensor.sweep(rotator=rotator)
        self.assertEqual(len(profile["bins_mm"]), SWEEP_SAMPLE_COUNT)
        self.assertEqual(profile["bins_mm"], [EFFECTIVE_MAX_MM] * SWEEP_SAMPLE_COUNT)
        self.assertEqual(rotator.steps, [SWEEP_STEP_DEG] * SWEEP_SAMPLE_COUNT)

    def test_sweep_works_without_rotator(self):
        driver = StubDriver([EFFECTIVE_MAX_MM] * SWEEP_SAMPLE_COUNT)
        sensor = ToFSensor(driver=driver)
        sensor.init()
        profile = sensor.sweep()
        self.assertEqual(profile["outcome"], CLASS_OPEN)


class ClassifyTest(unittest.TestCase):

    def test_enclosed_sustained_close_run_across_contiguous_arc(self):
        bins = [500] * 8 + [EFFECTIVE_MAX_MM] * (SWEEP_SAMPLE_COUNT - 8)
        self.assertEqual(classify(bins), (CLASS_ENCLOSED, False))

    def test_enclosed_all_around_walls(self):
        bins = [800] * SWEEP_SAMPLE_COUNT
        self.assertEqual(classify(bins), (CLASS_ENCLOSED, False))

    def test_open_sparse_or_far_profile(self):
        bins = [EFFECTIVE_MAX_MM] * SWEEP_SAMPLE_COUNT
        self.assertEqual(classify(bins), (CLASS_OPEN, False))

    def test_open_tolerates_few_isolated_close_bins(self):
        bins = [EFFECTIVE_MAX_MM] * 16 + [700, EFFECTIVE_MAX_MM]
        self.assertEqual(classify(bins), (CLASS_OPEN, False))

    def test_ambiguous_mid_run_marks_retry_with_conservative_open(self):
        bins = [500] * 5 + [EFFECTIVE_MAX_MM] * (SWEEP_SAMPLE_COUNT - 5)
        self.assertEqual(classify(bins), (CLASS_OPEN, True))


class SweepRetryTest(unittest.TestCase):

    def _ambiguous_bins(self):
        return [500] * 5 + [EFFECTIVE_MAX_MM] * (SWEEP_SAMPLE_COUNT - 5)

    def _enclosed_bins(self):
        return [500] * 8 + [EFFECTIVE_MAX_MM] * (SWEEP_SAMPLE_COUNT - 8)

    def test_ambiguous_first_sweep_retries_then_finds_enclosed(self):
        driver = StubDriver(self._ambiguous_bins() + self._enclosed_bins())
        rotator = FakeRotator()
        sensor = ToFSensor(driver=driver)
        sensor.init()
        profile = sensor.sweep(rotator=rotator)
        self.assertEqual(profile["outcome"], CLASS_ENCLOSED)
        self.assertFalse(profile["retry"])
        self.assertEqual(rotator.steps, [SWEEP_STEP_DEG] * SWEEP_SAMPLE_COUNT * 2)

    def test_retry_cap_exhausted_leaves_retry_flag_set(self):
        driver = StubDriver(self._ambiguous_bins() * (SWEEP_RETRY_MAX + 1))
        sensor = ToFSensor(driver=driver)
        sensor.init()
        profile = sensor.sweep()
        self.assertEqual(profile["outcome"], CLASS_OPEN)
        self.assertTrue(profile["retry"])


if __name__ == "__main__":
    unittest.main(verbosity=2)