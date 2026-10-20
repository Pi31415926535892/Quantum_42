"""Nicla Vision IMU: config-gated yaw heading frames + terrain pitch bands.

Gyro/accel samples arrive through an injectable source so the integrate
handler is pure Python — real LSM6DSOX hardware or a synthetic test feed.
Units: heading/pitch in degrees, angular rate in deg/s, dt in seconds.
"""

import math

YAW_WRAP_DEG = 360.0
SEESAW_PITCH_LIMIT_DEG = 20.0
RAMP_PITCH_MIN_DEG = 20.0
RAMP_PITCH_MAX_DEG = 25.0
TERRAIN_RAMP = "ramp"
TERRAIN_SEESAW = "seesaw"
TERRAIN_NONE = "none"
GYRO_DPS_PER_LSB_FS1000 = 0.035
ACCEL_G_PER_LSB_FS8G = 0.000244


def _wrap_heading(deg):
    return deg % YAW_WRAP_DEG


def pitch_from_accel(ax, ay, az):
    """Pitch angle (deg) from one accelerometer sample."""
    return math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))


def classify_terrain(pitch_deg):
    """Band discriminator: ramp 20-25 deg, seesaw below 20 deg, else none."""
    tilt = abs(pitch_deg)
    if RAMP_PITCH_MIN_DEG <= tilt <= RAMP_PITCH_MAX_DEG:
        return TERRAIN_RAMP
    if tilt < SEESAW_PITCH_LIMIT_DEG:
        return TERRAIN_SEESAW
    return TERRAIN_NONE


class Lsm6dsoxSource:
    """Adapter isolating the real IMU driver behind an injectable device.

    The device object exposes raw counts via gyro_raw()/accel_raw().
    """

    def __init__(self, device):
        self._device = device

    def gyro_dps(self):
        rx, ry, rz = self._device.gyro_raw()
        return (rx * GYRO_DPS_PER_LSB_FS1000,
                ry * GYRO_DPS_PER_LSB_FS1000,
                rz * GYRO_DPS_PER_LSB_FS1000)

    def accel_g(self):
        rx, ry, rz = self._device.accel_raw()
        return (rx * ACCEL_G_PER_LSB_FS8G,
                ry * ACCEL_G_PER_LSB_FS8G,
                rz * ACCEL_G_PER_LSB_FS8G)


class Imu:
    """Yaw integrator + pitch tracker fed from an injectable source.

    The source must provide gyro_dps() -> (gx, gy, gz) and
    accel_g() -> (ax, ay, az). yaw_enabled gates heading advance.
    """

    def __init__(self, source, yaw_enabled=True):
        self._source = source
        self._yaw_enabled = yaw_enabled
        self._heading_deg = 0.0
        self._pitch_deg = 0.0

    def feed_gyro_z(self, gyro_z_dps, dt_s):
        """Pure integrate handler: advance heading by one gyro sample."""
        if not self._yaw_enabled:
            return
        self._heading_deg = _wrap_heading(self._heading_deg + gyro_z_dps * dt_s)

    def step(self, dt_s):
        """Pull one gyro/accel sample from the source and integrate."""
        _, _, gz = self._source.gyro_dps()
        self.feed_gyro_z(gz, dt_s)
        ax, ay, az = self._source.accel_g()
        self._pitch_deg = pitch_from_accel(ax, ay, az)

    def heading_deg(self):
        """Current yaw heading, wrapped to [0, 360)."""
        return self._heading_deg

    def pitch_deg(self):
        """Current pitch from the latest accelerometer sample."""
        return self._pitch_deg

    def terrain_class(self):
        """Classify the latest pitch into ramp / seesaw / none."""
        return classify_terrain(self._pitch_deg)
