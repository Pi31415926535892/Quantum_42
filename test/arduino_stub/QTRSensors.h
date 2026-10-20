// QTRSensors.h — host-side stand-in for the Pololu QTR-8A Arduino library.
//
// Enough of the API for the legacy Robocup.ino archive to be type-checked
// without the vendor library installed. Inert: calibrate() does nothing and
// readLineBlack() returns a scripted centroid. Readings can be seeded with
// qtrSeedDarkChannels() by a host test if the follow loop is ever driven.
//
// This stub does NOT verify that the real library behaves the same way; the
// bench (HARDWARE.md section 7) remains the authority on QTR-8A polarity.

#ifndef ARDUINO_STUB_QTRSENSORS_H
#define ARDUINO_STUB_QTRSENSORS_H

#include <cstdint>

class QTRSensors {
 public:
  void setTypeAnalog() { analog_ = true; }
  void setTypeDigital() { analog_ = false; }

  void setSensorPins(const uint8_t* pins, uint8_t count) {
    (void)pins;
    (void)count;
  }

  void calibrate() {}

  uint16_t readLineBlack(uint16_t* values) {
    for (uint8_t channel = 0; channel < 8; ++channel) {
      values[channel] = darkChannels_ > channel ? 0 : 900;
    }
    return centroid_;
  }

  // --- test seam -----------------------------------------------------------
  void qtrSeedDarkChannels(uint8_t count) { darkChannels_ = count; }
  void qtrSeedCentroid(uint16_t centroid) { centroid_ = centroid; }

 private:
  bool analog_ = true;
  uint8_t darkChannels_ = 0;
  uint16_t centroid_ = 3500;
};

#endif  // ARDUINO_STUB_QTRSENSORS_H
