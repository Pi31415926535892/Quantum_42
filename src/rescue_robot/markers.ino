// markers.ino — AVR delivery layer for the green-marker pair.
//
// Owns every Arduino-side touch point on behalf of markers.h: the Wire bus at
// 400 kHz, TCA9548A channel select/deselect, the APDS-9960 green-channel read,
// and the millis clock handed to the decision logic. Contains zero business
// logic — the green test, the two-sample dead-end confirm, the blind window and
// the per-slice budget model all live in markers.h.
//
// Wiring (HARDWARE.md §9): I2C on D20 (SDA) / D21 (SCL), mux CH0 = LEFT
// sensor, CH1 = RIGHT sensor, both APDS-9960 answering at 0x39 on 3.3 V logic.

#include <Arduino.h>
#include <Wire.h>
#include "markers.h"

using namespace rescue;

namespace {

// ---------------------------------------------------------------------------
// Bus addressing
// ---------------------------------------------------------------------------

constexpr uint8_t TCA_ADDRESS = 0x70;   // TCA9548A control port
constexpr uint8_t APDS_ADDRESS = 0x39;  // APDS-9960 (both units answer here)
constexpr uint8_t TCA_ALL_OFF = 0x00;   // deselect every mux channel

// ---------------------------------------------------------------------------
// APDS-9960 registers used by the passive RGB green poll
// ---------------------------------------------------------------------------

constexpr uint8_t APDS_REG_COMMAND = 0x00;
constexpr uint8_t APDS_REG_ENABLE = 0x01;
constexpr uint8_t APDS_REG_CONFIG = 0x04;
constexpr uint8_t APDS_REG_GREEN_LOW = 0x0A;  // green low then green high
constexpr uint8_t APDS_GREEN_DATA_BYTES = 2;

// ENABLE register: colour engine on, proximity engine deliberately off so the
// proximity IR LED cannot see the QTR-8A's own emitters.
constexpr uint8_t APDS_ENABLE_OFF = 0x00;
constexpr uint8_t APDS_ENABLE_COLOR = 0x10;

// COMMAND register mode field (bits 6:4) plus power-on.
constexpr uint8_t APDS_COMMAND_POWER_ON = 0x01;
constexpr uint8_t APDS_COMMAND_RESET = 0x02;
constexpr uint8_t APDS_MODE_RESET = 0x40;
constexpr uint8_t APDS_MODE_COLOR = 0x60;

// CONFIG register integration-time field (bits 2:0). 31 ms suits the 1-2 cm
// passive RGB mount height and keeps a conversion inside the poll cadence.
constexpr uint8_t APDS_CONFIG_INTEGRATION_31MS = 0x01;

// ---------------------------------------------------------------------------
// Mux control
// ---------------------------------------------------------------------------

void tcaWriteMask(uint8_t channelMask) {
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(channelMask);
  Wire.endTransmission();
}

void tcaDeselect() { tcaWriteMask(TCA_ALL_OFF); }

// ---------------------------------------------------------------------------
// APDS-9960 access on the currently selected sensor
// ---------------------------------------------------------------------------

bool apdsWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Bring the selected sensor into passive RGB mode with proximity IR off.
bool apdsBeginColorMode() {
  if (!apdsWrite(APDS_REG_ENABLE, APDS_ENABLE_OFF)) return false;
  if (!apdsWrite(APDS_REG_COMMAND, APDS_COMMAND_POWER_ON)) return false;
  if (!apdsWrite(APDS_REG_COMMAND, APDS_COMMAND_RESET)) return false;
  if (!apdsWrite(APDS_REG_COMMAND, APDS_MODE_RESET)) return false;
  if (!apdsWrite(APDS_REG_COMMAND, APDS_MODE_COLOR)) return false;
  if (!apdsWrite(APDS_REG_CONFIG, APDS_CONFIG_INTEGRATION_31MS)) return false;
  return apdsWrite(APDS_REG_ENABLE, APDS_ENABLE_COLOR);
}

// Read the 16-bit green-channel word of the selected sensor.
bool apdsReadGreen(uint16_t* green) {
  if (green == nullptr) return false;
  Wire.beginTransmission(APDS_ADDRESS);
  Wire.write(APDS_REG_GREEN_LOW);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(APDS_ADDRESS),
                       static_cast<int>(APDS_GREEN_DATA_BYTES)) !=
      APDS_GREEN_DATA_BYTES) {
    return false;
  }
  const uint8_t low = static_cast<uint8_t>(Wire.read());
  const uint8_t high = static_cast<uint8_t>(Wire.read());
  *green = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
  return true;
}

// ---------------------------------------------------------------------------
// Bus seam handed to markers.h
// ---------------------------------------------------------------------------

void busSelect(uint8_t channelMask, void* ctx) {
  (void)ctx;
  tcaWriteMask(channelMask);
}

bool busReadGreen(uint16_t* green, void* ctx) {
  (void)ctx;
  return apdsReadGreen(green);
}

void busDeselect(void* ctx) {
  (void)ctx;
  tcaDeselect();
}

// Arduino's clock accessors return unsigned long; the logic seam wants a
// fixed-width return type.
uint32_t clockMicros() { return static_cast<uint32_t>(micros()); }
uint32_t clockMillis() { return static_cast<uint32_t>(millis()); }

MarkerBus makeBus() {
  return MarkerBus{busSelect, busReadGreen, busDeselect, clockMicros, nullptr};
}

// The millis clock is live from static-init, so the dead-end blind window runs
// from the first poll without an explicit arming call.
MarkerLogic gMarkerLogic(clockMillis);

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Bring up the marker bus: 400 kHz Wire, mux idle, both sensors in RGB mode.
void markersBegin() {
  Wire.begin();
  Wire.setClock(MARKER_BUS_HZ);
  tcaDeselect();
  tcaWriteMask(LEFT_MUX_CHANNEL);
  apdsBeginColorMode();
  tcaDeselect();
  tcaWriteMask(RIGHT_MUX_CHANNEL);
  apdsBeginColorMode();
  tcaDeselect();
}

// Feed the calibrated white baseline from the calibration sweep.
void setMarkerWhiteBaseline(uint16_t whiteBaseline) {
  gMarkerLogic.setWhiteBaseline(whiteBaseline);
}

// Poll both sides across separate mux reads; yields the published marker state.
MarkerPoll pollMarkerSensors() {
  return pollMarkers(makeBus(), gMarkerLogic, LEFT_MUX_CHANNEL,
                     RIGHT_MUX_CHANNEL);
}
