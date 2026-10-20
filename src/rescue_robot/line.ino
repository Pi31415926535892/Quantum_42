// line.ino — AVR delivery layer for the QTR-8A line bar.
//
// Owns every Arduino-side touch on behalf of line.h: the per-loop read, and the
// updateLine() entry point. It contains no logic — calibration, centroid,
// classification and steering all live in line.h. The bar's pin list lives in
// config.h, next to every other pin, so the calibration sweep and this module
// cannot disagree about which sensor is which.

#include <Arduino.h>

#include "config.h"
#include "line.h"

using namespace rescue;

namespace {

// config.h writes the Mega's analog numbers so it stays host-compilable; here
// they are checked against the names the wiring document uses, so a drift
// between the table and the A14/A12/... it claims to be is a build error on the
// board rather than a sensor read from the wrong pin at the venue. The bar's
// connector was flipped on 2026-10-20, so logical index 0 (leftmost) is A14.
static_assert(sizeof(LINE_SENSOR_PINS) / sizeof(LINE_SENSOR_PINS[0]) ==
                  LINE_SENSOR_COUNT,
              "config.h's line sensor table and line.h's channel count disagree");
static_assert(LINE_SENSOR_PINS[0] == A14 && LINE_SENSOR_PINS[1] == A12 &&
                  LINE_SENSOR_PINS[2] == A10 && LINE_SENSOR_PINS[3] == A8 &&
                  LINE_SENSOR_PINS[4] == A6 && LINE_SENSOR_PINS[5] == A4 &&
                  LINE_SENSOR_PINS[6] == A2 && LINE_SENSOR_PINS[7] == A0,
              "line sensor pins are not the A14,A12,...,A0 the wiring document "
              "gives for the flipped QTR-8A connector");

// One pass over the bar, in channel order.
LineVector readSensors() {
  LineVector samples{};
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    samples.reading[channel] = analogRead(LINE_SENSOR_PINS[channel]);
  }
  return samples;
}

}  // namespace

// Declare the bar pins as plain inputs; the QTR drives them itself.
void lineBegin() {
  for (uint8_t channel = 0; channel < LINE_SENSOR_COUNT; ++channel) {
    pinMode(LINE_SENSOR_PINS[channel], INPUT);
  }
}

// Read the bar and report the line: where it sits, how far off centre, and
// which coarse class qualifies that reading. Call once per main loop.
LineReading updateLine() {
  return analyze(readSensors());
}
