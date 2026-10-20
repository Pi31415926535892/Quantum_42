// Wire.h — host stub for the Arduino Wire (I2C) library.
//
// Exists so the assembled sketch (test/sketch_assembly_test.cpp) can compile
// markers.ino without an AVR toolchain or an I2C bus.
//
// markers.ino's own tests do NOT use this. They inject a MarkerBus, which is
// the seam the module was built around, so the colour read is exercised without
// a bus at all. This stub is here for the delivery layer, not for the logic.
//
// The device model is a tiny register file per address, so a write and the read
// that follows it can agree — enough for a host test to drive markersBegin() and
// see the code path run. Nothing is acknowledged by a real peripheral: an
// address with nothing behind it simply never ACKs.

#ifndef ARDUINO_STUB_WIRE_H
#define ARDUINO_STUB_WIRE_H

#include <cstddef>
#include <cstdint>

// Enough register space per address for the APDS-9960's colour and proximity
// blocks, which is all markers.ino reads.
constexpr uint8_t WIRE_STUB_REGISTER_COUNT = 64;
constexpr uint8_t WIRE_STUB_NO_ACK = 0;

class TwoWireClass {
 public:
  // Per-address register file, filled in by a test that wants a device present.
  uint8_t registers[256][WIRE_STUB_REGISTER_COUNT] = {};

  void begin() {}
  void begin(uint8_t address) { (void)address; }
  void setClock(uint32_t hz) { (void)hz; }
  void end() {}

  void beginTransmission(uint8_t address) {
    mAddress = address;
    mRegister = 0;
    mHasRegister = false;
    mLength = 0;
    mRepeatedStart = false;
  }

  size_t write(uint8_t value) {
    if (mLength >= WIRE_STUB_REGISTER_COUNT) return 0;
    // The first byte after an address is a register pointer; the rest is a
    // payload, which is what the APDS write path in markers.ino does.
    if (!mHasRegister) {
      mRegister = value;
      mHasRegister = true;
    } else {
      registers[mAddress][(mRegister + mPayloadIndex) % WIRE_STUB_REGISTER_COUNT] =
          value;
      ++mPayloadIndex;
    }
    return 1;
  }

  uint8_t endTransmission() {
    if (mRepeatedStart) mRepeatedStart = false;
    return endTransmissionInternal(true);
  }

  // The false overload keeps the bus held for a repeated start, which is how
  // the APDS read path addresses a register and reads it without releasing.
  uint8_t endTransmission(bool holdBus) {
    mRepeatedStart = holdBus;
    return endTransmissionInternal(holdBus);
  }

  uint8_t requestFrom(int address, int count) {
    mAddress = static_cast<uint8_t>(address);
    mRequested = count > 0 ? count : 0;
    mReadIndex = 0;
    return static_cast<uint8_t>(mRequested);
  }

  int available() { return mRequested - mReadIndex; }

  int read() {
    if (mReadIndex >= mRequested) return -1;
    const uint8_t value =
        registers[mAddress][(mRegister + mReadIndex) % WIRE_STUB_REGISTER_COUNT];
    ++mReadIndex;
    return static_cast<int>(value);
  }

 private:
  uint8_t endTransmissionInternal(bool) { return WIRE_STUB_NO_ACK; }

  uint8_t mAddress = 0;
  uint8_t mRegister = 0;
  uint8_t mPayloadIndex = 0;
  uint8_t mLength = 0;
  uint8_t mRequested = 0;
  uint8_t mReadIndex = 0;
  bool mHasRegister = false;
  bool mRepeatedStart = false;
};

extern TwoWireClass Wire;

#endif  // ARDUINO_STUB_WIRE_H