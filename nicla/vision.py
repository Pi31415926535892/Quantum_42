"""Nicla Vision colour perception: arena calibration, goal and triangle blobs.

The camera is reached only through an injected seam, so the detection core is
pure Python and host tests feed it synthetic frames. ``produce()`` returns the
wire-shaped payload dicts of the goal and triangle messages in
``docs/mega-nicla-protocol.md``, and the link codec turns them into frames.
Colour knowledge is learned from a held calibration card under a hard ceiling,
then cached to flash behind a stamp that catches a torn write; nothing is
emitted before a calibration is held, from the card or from the cache.
"""

import binascii
import math

from collections import namedtuple

try:                                  # MicroPython ships ujson, not json.
    import ujson as json
except ImportError:                   # pragma: no cover - the host branch.
    import json

from . import config, link

# What the host asks vision for: the protocol's REQUEST_BLOB mask bits.
SEEK_GOAL = 0x0001
SEEK_TRIANGLE_RED = 0x0002
SEEK_TRIANGLE_GREEN = 0x0004
SEEK_ALL = SEEK_GOAL | SEEK_TRIANGLE_RED | SEEK_TRIANGLE_GREEN
COLOR_RED = 0
COLOR_GREEN = 1
GOAL_BLOB = "GOAL_BLOB"
TRIANGLE_BLOB = "TRIANGLE_BLOB"
_SEEK_FOR_COLOR = {COLOR_RED: SEEK_TRIANGLE_RED,
                   COLOR_GREEN: SEEK_TRIANGLE_GREEN}

# The held calibration card: the red and green squares of the bench board, in
# the normalised frame scale, inside the frame and clear of each other.
CARD_PATCHES = {"red": (150, 300, 400, 550), "green": (600, 300, 850, 550)}

# Calibration bounds: the pre-run ceiling, the sample budget and the health
# gates a card has to pass before its thresholds are believed.
CAL_DEADLINE_MS = 90000
CAL_SAMPLE_FRAMES_MAX = 6
CAL_SAMPLE_FRAMES_MIN = 2
CAL_TOLERANCE_MIN = 12
CAL_TOLERANCE_MAX = 60
CAL_TOLERANCE_PER_SPREAD = 2
CAL_SPREAD_MAX = 30
CAL_SEPARATION_MIN = 60

# A goal patch reads large at approach distance, an evacuation marker does not.
# The host still names the class it wants; this only stops a swept frame from
# reporting a red marker as the goal.
GOAL_MIN_SIZE = 150

# Flash record: magic, format version, CRC-32 over the body, then the JSON.
CACHE_MAGIC = b"NVC1"
CACHE_FORMAT = 1
_CACHE_HEADER = len(CACHE_MAGIC) + 1 + 4

# On-device frame and LAB matching, owned by the camera adapter.
FRAME_W = 320
FRAME_H = 240
LAB_TOLERANCE = 18
BLOB_PIXELS_MIN = 10

# A blob the camera found, still measured in frame pixels.
Detection = namedtuple("Detection", "cx cy width height pixels")
# A calibrated colour: a mean RGB triple and its per-channel match window.
ColorThreshold = namedtuple("ColorThreshold", "name red green blue tolerance")


class Emission:
    """One wire message: a message-table name plus its field dict."""

    __slots__ = ("name", "payload")

    def __init__(self, name, payload):
        self.name = name
        self.payload = payload

    @property
    def msg_id(self):
        """Message id read from the link's table, the one source of truth."""
        return getattr(link.MSG, self.name)


def _be32(value):
    """Big-endian unsigned 32-bit word."""
    return bytes((value >> shift) & 0xFF for shift in (24, 16, 8, 0))


def _word32(record, offset):
    """Big-endian unsigned 32-bit word at ``offset``."""
    return sum(byte << (8 * (3 - index))
               for index, byte in enumerate(record[offset:offset + 4]))


def encode_cache(thresholds):
    """``NVC1 | format | CRC-32 | JSON``; the stamp catches a torn write."""
    body = json.dumps([list(threshold) for threshold in thresholds]).encode()
    stamp = binascii.crc32(body) & 0xFFFFFFFF
    return bytes(CACHE_MAGIC) + bytes((CACHE_FORMAT,)) + _be32(stamp) + body


def decode_cache(record):
    """The colours a stamped record holds, or None when it cannot be read."""
    if record is None or len(record) < _CACHE_HEADER:
        return None
    if not record.startswith(CACHE_MAGIC) or record[4] != CACHE_FORMAT:
        return None
    body = record[_CACHE_HEADER:]
    if _word32(record, len(CACHE_MAGIC) + 1) != binascii.crc32(body):
        return None
    try:
        rows = json.loads(body.decode())
        if sorted(row[0] for row in rows) != sorted(CARD_PATCHES):
            return None
        return dict((row[0], ColorThreshold(*row)) for row in rows)
    except (ValueError, TypeError, KeyError, IndexError):
        return None


def _patch_summary(means):
    """Channel means of the samples plus the widest spread across them."""
    count = len(means)
    average = [sum(mean[channel] for mean in means) // count
               for channel in range(3)]
    spread = max(max(mean[channel] for mean in means)
                 - min(mean[channel] for mean in means) for channel in range(3))
    return tuple(average), spread


def _thresholds_from_summary(summary):
    """Calibrated colours from the card summary, or None when it is unhealthy."""
    red, green = summary["red"][0], summary["green"][0]
    if max(abs(one - other) for one, other in zip(red, green)) < CAL_SEPARATION_MIN:
        return None
    thresholds = {}
    for name, (mean, spread) in summary.items():
        if spread > CAL_SPREAD_MAX:
            return None
        window = max(CAL_TOLERANCE_MIN,
                     min(CAL_TOLERANCE_MAX, spread * CAL_TOLERANCE_PER_SPREAD))
        thresholds[name] = ColorThreshold(name, mean[0], mean[1], mean[2], window)
    return thresholds


def _scale(value, extent):
    """A frame measurement onto the wire's 0..1000 scale, clamped."""
    return max(0, min(config.COORD_MAX, value * config.COORD_MAX // extent))


def _diagonal(box):
    """Normalised blob diagonal: the wire's ``size`` field."""
    return max(0, min(config.COORD_MAX,
                      int(math.sqrt(box["width"] ** 2 + box["height"] ** 2))))


def _largest(blobs):
    """The biggest detection of one colour, or None when there is none."""
    best = None
    for blob in blobs:
        if best is None or blob.width * blob.height > best.width * best.height:
            best = blob
    return best


def _host_millis():
    """Millisecond clock: the microsecond timer on device, monotonic on host."""
    import time
    ticks = getattr(time, "ticks_ms", None)
    return ticks() if ticks else int(time.monotonic() * 1000)


class Vision:
    """Calibrated colour perception over an injected camera and flash seam.

    ``camera`` offers ``start()``, ``size()``, ``grab()``, ``blobs(colour)``
    and ``patch_mean(rect)``; ``storage`` offers ``read()`` and
    ``write(data)``; ``clock`` returns milliseconds. ``mask`` names the class
    the host is asking about, so an approach never has to guess whether a red
    patch is the goal or an evacuation marker.
    """

    def __init__(self, camera=None, storage=None, clock=None):
        self._camera = OpenMvCamera() if camera is None else camera
        self._storage = storage
        self._clock = _host_millis if clock is None else clock
        self._thresholds = {}

    def start(self):
        """Bring the camera seam up."""
        self._camera.start()

    def calibrated(self):
        """True once a calibration is held, learned or loaded from cache."""
        return bool(self._thresholds)

    def thresholds(self):
        """The calibrated colours, keyed by name."""
        return dict(self._thresholds)

    def calibrate(self, deadline_ms=CAL_DEADLINE_MS):
        """Learn the arena colours from the held card, inside the deadline."""
        samples = dict((name, []) for name in CARD_PATCHES)
        started, frames = self._clock(), 0
        while frames < CAL_SAMPLE_FRAMES_MAX:
            self._camera.grab()
            frames += 1
            for name, rect in CARD_PATCHES.items():
                mean = self._camera.patch_mean(rect)
                if mean is not None:
                    samples[name].append(mean)
            if self._clock() - started >= deadline_ms:
                break
        if frames < CAL_SAMPLE_FRAMES_MIN:
            return False
        summary = dict((name, _patch_summary(means) if means else None)
                       for name, means in samples.items())
        thresholds = (None if None in summary.values()
                      else _thresholds_from_summary(summary))
        if thresholds is None:
            return False
        self._thresholds = thresholds
        return True

    def save_cache(self):
        """Stamp the calibration into flash; False when none is held."""
        if not self.calibrated() or self._storage is None:
            return False
        return bool(self._storage.write(
            encode_cache(list(self._thresholds.values()))))

    def load_cache(self):
        """Adopt a stamped calibration from flash; False when unusable."""
        if self._storage is None:
            return False
        thresholds = decode_cache(self._storage.read())
        if thresholds is None:
            return False
        self._thresholds = thresholds
        return True

    def produce(self, mask=SEEK_ALL):
        """Detections from one frame, or None when nothing may be emitted."""
        if not self.calibrated() or not mask:
            return None
        self._camera.grab()
        frame_w, frame_h = self._camera.size()
        if frame_w <= 0 or frame_h <= 0:
            return None
        red = _largest(self._camera.blobs(self._thresholds["red"]))
        green = _largest(self._camera.blobs(self._thresholds["green"]))
        goal = self._goal_blob(red, mask, frame_w, frame_h)
        if goal is not None:
            return [goal]
        emissions = [emission for emission
                     in (self._triangle(COLOR_RED, red, mask, frame_w, frame_h),
                         self._triangle(COLOR_GREEN, green, mask, frame_w, frame_h))
                     if emission is not None]
        return emissions or None

    def _goal_blob(self, blob, mask, frame_w, frame_h):
        """The red patch as a goal message, when it reads big and solid."""
        if blob is None or not mask & SEEK_GOAL:
            return None
        box = self._normalise(blob, frame_w, frame_h)
        if (max(box["width"], box["height"]) < GOAL_MIN_SIZE
                or box["confidence"] < config.GOAL_CONF_MIN):
            return None
        return Emission(GOAL_BLOB, {"cx": box["cx"], "cy": box["cy"],
                                    "width": box["width"],
                                    "height": box["height"],
                                    "confidence": box["confidence"]})

    def _triangle(self, color, blob, mask, frame_w, frame_h):
        """A red or green evacuation marker as a triangle message."""
        if blob is None or not mask & _SEEK_FOR_COLOR[color]:
            return None
        box = self._normalise(blob, frame_w, frame_h)
        size = _diagonal(box)
        if size < config.BLOB_MIN_SIZE:
            return None
        return Emission(TRIANGLE_BLOB, {"color": color, "cx": box["cx"],
                                       "cy": box["cy"], "size": size})

    @staticmethod
    def _normalise(blob, frame_w, frame_h):
        """Frame pixels onto the wire: coordinates 0..1000, fill as percent."""
        area = max(1, blob.width) * max(1, blob.height)
        return {"cx": _scale(blob.cx, frame_w), "cy": _scale(blob.cy, frame_h),
                "width": _scale(blob.width, frame_w),
                "height": _scale(blob.height, frame_h),
                "confidence": min(config.CONFIDENCE_MAX_PCT,
                                  blob.pixels * config.CONFIDENCE_MAX_PCT // area)}


class OpenMvCamera:
    """Camera seam for the Nicla's GC2145 through the OpenMV image API.

    ``start()`` configures the sensor and ``grab()`` takes the frame that
    ``blobs()`` and ``patch_mean()`` then read. The device modules are
    imported inside ``start()``, so this file stays importable on a host
    where tests inject stand-ins for them.
    """

    def __init__(self, framesize=None):
        self._framesize = (FRAME_W, FRAME_H) if framesize is None else framesize
        self._sensor = None
        self._image = None
        self._frame = None

    def start(self):
        """Reset the sensor and fix the pixel format and the frame size."""
        import image
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(self._framesize)
        self._sensor = sensor
        self._image = image

    def size(self):
        """Frame width and height in pixels."""
        return self._framesize

    def grab(self):
        """Take the next frame; the blobs and patch means read this one."""
        self._frame = self._sensor.snapshot()

    def blobs(self, threshold):
        """Every blob of a calibrated colour, as frame-pixel detections."""
        lab = tuple(self._image.rgb_to_lab((threshold.red, threshold.green,
                                            threshold.blue)))
        found = self._frame.find_blobs([lab + (LAB_TOLERANCE,)],
                                       pixels_threshold=BLOB_PIXELS_MIN,
                                       merge=True)
        return [Detection(blob.cx(), blob.cy(), blob.w(), blob.h(),
                          blob.pixels()) for blob in found]

    def patch_mean(self, rect):
        """Mean RGB of a normalised card patch, None when it is off-frame."""
        roi = self._pixel_roi(rect)
        if roi is None:
            return None
        stats = self._frame.get_statistics(roi=roi)
        return (int(stats.r_mean()), int(stats.g_mean()), int(stats.b_mean()))

    def _pixel_roi(self, rect):
        """A normalised card rect as an OpenMV (x, y, w, h) pixel window."""
        frame_w, frame_h = self._framesize
        left = rect[0] * frame_w // config.COORD_MAX
        top = rect[1] * frame_h // config.COORD_MAX
        width = rect[2] * frame_w // config.COORD_MAX - left
        height = rect[3] * frame_h // config.COORD_MAX - top
        return None if width <= 0 or height <= 0 else (left, top, width, height)
