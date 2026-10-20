import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nicla import imu


class SequenceSource:
    """Synthetic gyro/accel stream; each gyro read pops the next sample."""

    def __init__(self, samples):
        self._samples = iter(samples)
        self._current = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def gyro_dps(self):
        self._current = next(self._samples)
        return self._current[0:3]

    def accel_g(self):
        return self._current[3:6]


class FakeRawDevice:
    """Raw LSM6DSOX-style counts on demand."""

    def __init__(self, gyro_raw_seq, accel_raw_seq):
        self._gyro = iter(gyro_raw_seq)
        self._accel = iter(accel_raw_seq)

    def gyro_raw(self):
        return next(self._gyro)

    def accel_raw(self):
        return next(self._accel)


class YawIntegrationTest(unittest.TestCase):
    def test_integrates_known_rate_sequence(self):
        heading = imu.Imu(SequenceSource([]))
        heading.feed_gyro_z(10.0, 1.0)
        heading.feed_gyro_z(20.0, 0.5)
        self.assertAlmostEqual(heading.heading_deg(), 20.0)

    def test_wraps_across_360(self):
        heading = imu.Imu(SequenceSource([]))
        heading.feed_gyro_z(350.0, 1.0)
        self.assertAlmostEqual(heading.heading_deg(), 350.0)
        heading.feed_gyro_z(20.0, 1.0)
        self.assertAlmostEqual(heading.heading_deg(), 10.0)

    def test_wraps_below_zero_to_360(self):
        heading = imu.Imu(SequenceSource([]))
        heading.feed_gyro_z(-10.0, 1.0)
        self.assertAlmostEqual(heading.heading_deg(), 350.0)

    def test_yaw_gate_disabled_holds_heading(self):
        heading = imu.Imu(SequenceSource([]), yaw_enabled=False)
        heading.feed_gyro_z(90.0, 2.0)
        self.assertAlmostEqual(heading.heading_deg(), 0.0)

    def test_step_consumes_injected_source(self):
        sample = (0.0, 0.0, 90.0, -math.sin(math.radians(22.5)), 0.0,
                  math.cos(math.radians(22.5)))
        bot = imu.Imu(SequenceSource([sample, sample]))
        bot.step(1.0)
        self.assertAlmostEqual(bot.heading_deg(), 90.0)
        self.assertAlmostEqual(bot.pitch_deg(), 22.5)
        self.assertEqual(bot.terrain_class(), imu.TERRAIN_RAMP)
        bot.step(1.0)
        self.assertAlmostEqual(bot.heading_deg(), 180.0)


class PitchFromAccelTest(unittest.TestCase):
    def test_flat_board_reads_zero_pitch(self):
        self.assertAlmostEqual(imu.pitch_from_accel(0.0, 0.0, 1.0), 0.0)

    def test_tilted_board_reads_angle(self):
        pitch = imu.pitch_from_accel(-math.sin(math.radians(30)), 0.0,
                                     math.cos(math.radians(30)))
        self.assertAlmostEqual(pitch, 30.0, places=5)


class TerrainBandTest(unittest.TestCase):
    def test_ramp_band_in(self):
        for angle in (imu.RAMP_PITCH_MIN_DEG, 22.5, imu.RAMP_PITCH_MAX_DEG):
            self.assertEqual(imu.classify_terrain(angle), imu.TERRAIN_RAMP)
            self.assertEqual(imu.classify_terrain(-angle), imu.TERRAIN_RAMP)

    def test_seesaw_bin_in(self):
        for angle in (0.0, 10.0, 19.9):
            self.assertEqual(imu.classify_terrain(angle), imu.TERRAIN_SEESAW)
            self.assertEqual(imu.classify_terrain(-angle), imu.TERRAIN_SEESAW)

    def test_out_of_band(self):
        self.assertEqual(imu.classify_terrain(25.1), imu.TERRAIN_NONE)
        self.assertEqual(imu.classify_terrain(45.0), imu.TERRAIN_NONE)


class SourceScalingTest(unittest.TestCase):
    def test_scales_raw_gyro_and_accel_counts(self):
        device = FakeRawDevice(gyro_raw_seq=[(1000, 0, 0)],
                               accel_raw_seq=[(0, 0, 4096)])
        source = imu.Lsm6dsoxSource(device)
        gx, gy, gz = source.gyro_dps()
        self.assertAlmostEqual(gx, 1000 * imu.GYRO_DPS_PER_LSB_FS1000)
        self.assertAlmostEqual(gy, 0.0)
        self.assertAlmostEqual(gz, 0.0)
        ax, ay, az = source.accel_g()
        self.assertAlmostEqual(az, 4096 * imu.ACCEL_G_PER_LSB_FS8G)


if __name__ == "__main__":
    unittest.main()
