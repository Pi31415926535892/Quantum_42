"""Nicla VL53L1CB time-of-flight ranging plus coarse radar sweep.

Two working modes:

1. Continuous ranging - ``ToFSensor.distance_mm()`` returns a normalised
   millimetre distance for obstacle/goal cross-validation.
2. Radar sweep - ``ToFSensor.sweep()`` samples a 360-degree rotation at
   ``SWEEP_STEP_DEG`` steps and returns a range profile whose outcome is one
   of exactly two contract classes: ``CLASS_ENCLOSED`` or ``CLASS_OPEN``.
   When the profile is ambiguous a bounded number of re-sweeps is attempted
   and the final profile carries ``retry=True`` if the cap is exhausted.

Out-of-range (no-return) bins are calibrated to the sensor's effective max
range (``OUT_OF_RANGE_MM``), a tuning constant, not a magic number.

Radar profile wire shape (PROVISIONAL - the documented frame layout does not
exist yet; this dict is the Nicla-side contract until the protocol doc lands):

    {
        "bins_mm": [int] * SWEEP_SAMPLE_COUNT,   # out-of-range bins == OUT_OF_RANGE_MM
        "outcome": "enclosed" | "open",          # exactly the two contract classes
        "retry": bool,                           # re-sweep needed after ambiguity
    }

Hardware: the VL53L1 driver calls (init/read) live behind an injectable
driver seam (anything with ``init()`` and ``read_range()``). Tests feed
synthetic range profiles through a stub driver; the on-device driver is only
imported when ``Vl53L1xDriver`` is actually used.
"""

# --- Tuning knobs (calibrate on-device; live here until the config layer lands) ---

EFFECTIVE_MAX_MM = 4000          # VL53L1CB guaranteed range (~4 m).
OUT_OF_RANGE_MM = EFFECTIVE_MAX_MM  # Bin value for no-return / error readings.
SWEEP_STEP_DEG = 20              # Angular step per radar sample.
SWEEP_SAMPLE_COUNT = 360 // SWEEP_STEP_DEG  # 18 bins per rotation.
ENCLOSED_CLOSE_MM = 1200         # A bin at/below this is a "close echo".
ENCLOSED_MIN_RUN = 6             # Contiguous close bins (120 deg) => enclosed.
OPEN_MAX_RUN = 3                 # Longest contiguous close run tolerated as open.
OPEN_MAX_CLOSE_BINS = 4          # Total close bins tolerated as open.
SWEEP_RETRY_MAX = 2              # Bounded re-sweep cap on ambiguity.

CLASS_ENCLOSED = "enclosed"
CLASS_OPEN = "open"


def classify(bins_mm):
    """Reduce a range profile to one of the two contract classes.

    Returns ``(outcome, retry)``. ``retry`` is True only when the profile is
    ambiguous (not clearly enclosed, not clearly open); the label then stays
    conservative. Exactly two labels are ever produced.
    """
    close = [1 if bin_mm <= ENCLOSED_CLOSE_MM else 0 for bin_mm in bins_mm]
    longest_run = _longest_close_run(close)
    close_total = sum(close)
    if longest_run >= ENCLOSED_MIN_RUN:
        return CLASS_ENCLOSED, False
    if longest_run <= OPEN_MAX_RUN and close_total <= OPEN_MAX_CLOSE_BINS:
        return CLASS_OPEN, False
    return CLASS_OPEN, True


def _longest_close_run(close_bits):
    best = 0
    current = 0
    for bit in close_bits:
        current = current + 1 if bit else 0
        if current > best:
            best = current
    return best


class Vl53L1xDriver:
    """On-device VL53L1 seam implementation (OpenMV/MicroPython only).

    Only ever used on the Nicla itself; the ST driver module is imported
    lazily so this file stays importable on macOS for the test suite.
    Inject any object exposing ``init()`` and ``read_range()`` instead.
    """

    def __init__(self, i2c_bus=2, address=0x29):
        self._i2c_bus = i2c_bus
        self._address = address
        self._sensor = None

    def init(self):
        import machine
        import vl53l1x  # ST VL53L1X MicroPython driver (device-only)
        self._sensor = vl53l1x.VL53L1X(machine.I2C(self._i2c_bus), self._address)
        self._sensor.init()

    def read_range(self):
        return self._sensor.read()


class _NullRotator:
    """Samples a sweep without commanding any physical rotation."""

    def step(self, deg):
        pass


class ToFSensor:
    """ToF ranging and radar sweep over an injected driver seam."""

    def __init__(self, driver=None):
        self._driver = driver if driver is not None else Vl53L1xDriver()

    def init(self):
        self._driver.init()

    def distance_mm(self):
        """Raw driver reading normalised to a mm int (out-of-range binned)."""
        return self._normalise(self._driver.read_range())

    def sweep(self, rotator=None, max_retries=SWEEP_RETRY_MAX):
        """360-degree radar sweep: ``SWEEP_SAMPLE_COUNT`` samples at 20 deg.

        ``rotator`` integrates the sweep with robot rotation: its ``step(deg)``
        is called once per sample (single-read seam; on-device wiring signs a
        turn frame over the host link). Ambiguous profiles are re-swept up to
        ``max_retries`` times.
        """
        if rotator is None:
            rotator = _NullRotator()
        attempts = 0
        while True:
            bins_mm = self._sample_profile(rotator)
            outcome, retry = classify(bins_mm)
            if not retry or attempts >= max_retries:
                return {
                    "bins_mm": bins_mm,
                    "outcome": outcome,
                    "retry": retry,
                }
            attempts += 1

    def _sample_profile(self, rotator):
        bins_mm = []
        for _ in range(SWEEP_SAMPLE_COUNT):
            bins_mm.append(self.distance_mm())
            rotator.step(SWEEP_STEP_DEG)
        return bins_mm

    def _normalise(self, raw):
        if raw is None or raw < 0:
            return OUT_OF_RANGE_MM
        return min(round(raw), OUT_OF_RANGE_MM)