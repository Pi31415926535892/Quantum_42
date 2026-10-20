// line.h — pure host-compilable line sensing logic (C++14, no AVR I/O).
//
// Turns eight raw analog readings from the QTR-8A bar into a weighted centroid
// across the 0..7000 position range, names the situation that centroid implies,
// and maps the offset from centre onto a steering correction.
//
// Two rules govern this file: the centroid is the control signal and the
// coarse class is a qualifier that never steers; and nothing here knows sensor
// polarity or gain, both of which come from the per-channel black/white
// readings the calibration sweep installs.
//
// All arithmetic is 32-bit: the AVR's native int is 16 bits and overflows here.

#ifndef RESCUE_ROBOT_LINE_H
#define RESCUE_ROBOT_LINE_H

#include <cstdint>

namespace rescue {

// --- Sensor bar geometry --------------------------------------------------
// Channel 0 is the leftmost sensor and weighs 0; channel 7 is the rightmost
// and weighs 7000, so the range advances in even steps across the bar.

constexpr uint8_t LINE_SENSOR_COUNT = 8;
constexpr int32_t LINE_POSITION_MIN = 0;
constexpr int32_t LINE_POSITION_MAX = 7000;
constexpr int32_t LINE_POSITION_CENTER = 3500;
constexpr int32_t LINE_POSITION_PITCH =
    (LINE_POSITION_MAX - LINE_POSITION_MIN) / (LINE_SENSOR_COUNT - 1);

// --- Normalised line signal -----------------------------------------------
// Every channel is reduced to one number: LINE_WEIGHT_MAX means the sensor sits
// squarely over the black line, LINE_WEIGHT_MIN means it sees only white floor.
// Calibration reclaims the dynamic range the 11.2 mm ride height squeezes out
// of the raw signal and discounts channels the sweep never saw at full contrast.

constexpr int32_t LINE_WEIGHT_MIN = 0;
constexpr int32_t LINE_WEIGHT_MAX = 1000;

// Signal level at which a channel counts as touching the line.
constexpr int32_t LINE_PRESENT_THRESHOLD = 150;

// Contrast at which a channel is trusted completely, and the floor below which
// it is treated as blind. Between the two the weight ramps linearly, so a
// channel that reached only partial contrast is trusted in proportion.
constexpr int32_t LINE_FULL_CONTRAST = 150;
constexpr int32_t LINE_MIN_CONTRAST = 30;

// Raw analog span of the AVR's 10-bit converter, and the placeholder
// calibration installed until the sweep records venue readings.
constexpr int32_t LINE_ANALOG_MIN = 0;
constexpr int32_t LINE_ANALOG_MAX = 1023;

// How close to either end of the bar still counts as ON_LINE rather than an
// edge. One sensor pitch separates neighbouring sensors, so a line under the
// outermost channel already reads as one.
constexpr int32_t LINE_EDGE_MARGIN = LINE_POSITION_PITCH;

// --- Steering -------------------------------------------------------------
// The centroid offset is the control signal, so the gain is positive and
// small: with the corrected motor wiring a line to the RIGHT (positive error)
// must speed the LEFT wheel up to chase it. The derivative gain stays at zero
// until the proportional term tracks; it is wired in so that enabling it later
// is a one-constant change.

constexpr double LINE_KP = 0.10;
constexpr double LINE_KD = 0.0;

// Steering cap: past this the robot turns at full lock however far off
// centre the line is. LINE_CORRECTION_ROUNDING is half a correction unit,
// used to round a demand to the nearest whole unit.
constexpr int32_t LINE_MAX_CORRECTION = 65;
constexpr double LINE_CORRECTION_ROUNDING = 0.5;

// --- Samples --------------------------------------------------------------

// One raw analog reading per sensor, leftmost first.
struct LineVector {
  int32_t reading[LINE_SENSOR_COUNT];
};

// Per-channel calibration: the readings the sweep recorded for this sensor on
// black and on white. Their distance is the channel's contrast and the
// direction of that distance is its polarity, so an inverted sensor bar
// calibrates itself without any change here.
struct ChannelCalibration {
  int32_t black;
  int32_t white;
};

struct LineCalibration {
  ChannelCalibration channel[LINE_SENSOR_COUNT];
};

// Per-sensor line signal after calibration, LINE_WEIGHT_MIN..LINE_WEIGHT_MAX.
struct LineWeights {
  int32_t weight[LINE_SENSOR_COUNT];
};

// Coarse description of what the bar is sitting on. Qualifiers ONLY — the
// steering signal is always the centroid.
enum class LineClass {
  ON_LINE,    // the line is on the bar and clear of both ends
  EDGE,       // the line is on the bar but running off one end of it
  ALL_WHITE,  // no channel is on the line at all
  ALL_DARK    // every channel is on the line at once
};

// Everything one look at the bar yields.
struct LineReading {
  int32_t position;    // weighted centroid, LINE_POSITION_MIN..LINE_POSITION_MAX
  int32_t error;       // position - LINE_POSITION_CENTER; positive is line right
  bool lineDetected;   // false when no channel is touching the line
  LineClass qualifier; // coarse class for the state machine; never steering
};

// Per-wheel trim to add to the base speed, in the units the drive layer
// expects. Positive on the left with the line to the right turns back onto it.
struct WheelBias {
  int32_t left;
  int32_t right;
};

// --- Calibration ----------------------------------------------------------

// Calibration applying the same two readings to every sensor.
inline LineCalibration defaultCalibration(int32_t black, int32_t white) {
  LineCalibration calibration{};
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    calibration.channel[channel].black = black;
    calibration.channel[channel].white = white;
  }
  return calibration;
}

namespace detail {

// The calibration currently in force. Until the sweep runs it spans the full
// analog range: a placeholder, not a venue-recorded basis.
inline LineCalibration& installedCalibration() {
  static LineCalibration store = defaultCalibration(LINE_ANALOG_MIN, LINE_ANALOG_MAX);
  return store;
}

// Distance between a channel's black and white readings.
inline int32_t channelContrast(const ChannelCalibration& calibration) {
  const int32_t span = calibration.white - calibration.black;
  return span < 0 ? -span : span;
}

// How much of the full line signal a channel may deliver, scaled by the
// contrast the sweep found on it. A blind channel delivers none.
inline int32_t channelTrust(const ChannelCalibration& calibration) {
  const int32_t contrast = channelContrast(calibration);
  if (contrast <= LINE_MIN_CONTRAST) return LINE_WEIGHT_MIN;
  if (contrast >= LINE_FULL_CONTRAST) return LINE_WEIGHT_MAX;
  const int32_t ramp = LINE_FULL_CONTRAST - LINE_MIN_CONTRAST;
  return LINE_WEIGHT_MAX * (contrast - LINE_MIN_CONTRAST) / ramp;
}

// How far along the black-to-white span a reading sits, scaled to
// LINE_WEIGHT_MAX: 0 on the black line, LINE_WEIGHT_MAX on the white floor. A
// reading outside the calibrated span falls outside that range and the caller
// clamps it, so a saturated reading saturates rather than wrapping.
inline int32_t whiteness(const ChannelCalibration& calibration, int32_t rawReading) {
  const int32_t span = calibration.white - calibration.black;
  const int32_t spanMagnitude = span < 0 ? -span : span;
  const int32_t offset = rawReading - calibration.black;
  const int32_t travel = span < 0 ? -offset : offset;
  return travel * LINE_WEIGHT_MAX / spanMagnitude;
}

// How many channels are touching the line.
inline uint8_t contactCount(const LineWeights& weights) {
  uint8_t contacts = 0;
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    if (weights.weight[channel] >= LINE_PRESENT_THRESHOLD) ++contacts;
  }
  return contacts;
}

// Sum of the calibrated line signal across the bar.
inline int32_t totalWeight(const LineWeights& weights) {
  int32_t total = LINE_WEIGHT_MIN;
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    total += weights.weight[channel];
  }
  return total;
}

// Round a steering demand to the nearest whole unit and cap its authority.
inline int32_t clampCorrection(double demand) {
  const double rounded = demand >= 0.0 ? demand + LINE_CORRECTION_ROUNDING
                                       : demand - LINE_CORRECTION_ROUNDING;
  int32_t correction = static_cast<int32_t>(rounded);
  if (correction > LINE_MAX_CORRECTION) correction = LINE_MAX_CORRECTION;
  if (correction < -LINE_MAX_CORRECTION) correction = -LINE_MAX_CORRECTION;
  return correction;
}

}  // namespace detail

// Install the per-channel readings produced by the calibration sweep.
inline void setCalibration(const LineCalibration& calibration) {
  detail::installedCalibration() = calibration;
}

// --- Normalisation --------------------------------------------------------

// Weight the sensor at this index carries across the 0..7000 range.
inline int32_t sensorPosition(uint8_t index) {
  return LINE_POSITION_MIN + static_cast<int32_t>(index) * LINE_POSITION_PITCH;
}

// Reduce one raw reading to a calibrated line signal in
// LINE_WEIGHT_MIN..LINE_WEIGHT_MAX, scaled by how far that channel is trusted.
inline int32_t channelWeight(uint8_t index, int32_t rawReading) {
  const ChannelCalibration& calibration = detail::installedCalibration().channel[index];
  const int32_t trust = detail::channelTrust(calibration);
  if (trust == LINE_WEIGHT_MIN) return LINE_WEIGHT_MIN;

  int32_t signal = LINE_WEIGHT_MAX - detail::whiteness(calibration, rawReading);
  if (signal > LINE_WEIGHT_MAX) signal = LINE_WEIGHT_MAX;
  if (signal < LINE_WEIGHT_MIN) signal = LINE_WEIGHT_MIN;
  return signal * trust / LINE_WEIGHT_MAX;
}

// Calibrate the whole bar in one pass.
inline LineWeights normalize(const LineVector& raw) {
  LineWeights weights{};
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    weights.weight[channel] = channelWeight(channel, raw.reading[channel]);
  }
  return weights;
}

// --- Centroid -------------------------------------------------------------

// Weighted centroid of the line signal. With no signal there is no centroid to
// report, so the range centre stands in and the caller is told no channel is
// touching the line.
inline int32_t centroid(const LineWeights& weights) {
  const int32_t total = detail::totalWeight(weights);
  if (total <= LINE_WEIGHT_MIN) return LINE_POSITION_CENTER;

  int32_t moment = 0;
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    moment += weights.weight[channel] * sensorPosition(channel);
  }
  return (moment + total / 2) / total;
}

// --- Read the bar ---------------------------------------------------------

// Name the situation the bar is in. Only the mission state machine may branch
// on this; the drive always follows the centroid.
inline LineClass classify(const LineWeights& weights, int32_t position) {
  const uint8_t contacts = detail::contactCount(weights);
  if (contacts == 0) return LineClass::ALL_WHITE;
  if (contacts == LINE_SENSOR_COUNT) return LineClass::ALL_DARK;
  if (position <= LINE_EDGE_MARGIN) return LineClass::EDGE;
  if (position >= LINE_POSITION_MAX - LINE_EDGE_MARGIN) return LineClass::EDGE;
  return LineClass::ON_LINE;
}

// Normalise, locate the line and report where it sits relative to the centre.
inline LineReading analyze(const LineVector& raw) {
  const LineWeights weights = normalize(raw);
  const int32_t position = centroid(weights);
  return LineReading{position, position - LINE_POSITION_CENTER,
                     detail::contactCount(weights) > 0,
                     classify(weights, position)};
}

// --- Steering -------------------------------------------------------------

// Proportional-plus-derivative steering demand from the centroid error. The
// derivative term is present but switched off by LINE_KD.
inline int32_t pdCorrection(int32_t error, int32_t previousError) {
  const double proportional = static_cast<double>(error) * LINE_KP;
  const double derivative = static_cast<double>(error - previousError) * LINE_KD;
  return detail::clampCorrection(proportional + derivative);
}

// Steering correction for a fresh reading, carrying nothing over from the last.
inline int32_t correctionFor(int32_t error) {
  return pdCorrection(error, error);
}

// Split the correction across the two wheels. Positive error means the line
// sits right, so the left wheel is driven faster and the right slower.
inline WheelBias steerFor(int32_t error) {
  const int32_t correction = correctionFor(error);
  return WheelBias{correction, -correction};
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_LINE_H
