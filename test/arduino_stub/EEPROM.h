// EEPROM.h — host stub for the Arduino EEPROM library.
//
// Exists so the assembled sketch (test/sketch_assembly_test.cpp) can compile
// eeprom_store.ino without an AVR toolchain. The store's own tests do NOT use
// this: they drive the record codec in eeprom_store.h over plain byte buffers,
// because the layout is what is under test, not the library.
//
// The device here is a byte array, so a store read and a store write both work
// and a test can look at what was committed. Nothing is powered, nothing
// wears out, and no write is slow — none of which is true of the real part.

#ifndef ARDUINO_STUB_EEPROM_H
#define ARDUINO_STUB_EEPROM_H

#include <cstddef>
#include <cstdint>

// The Mega 2560's real ceiling. eeprom_store.ino static_asserts its map against
// this, so the stub has to answer with the true device size or that check would
// pass here and fail on the board.
constexpr int E2END = 4095;

class EEPROMClass {
 public:
  // The whole device, as the core exposes it.
  uint8_t data[E2END + 1] = {};

  uint8_t read(int address) const {
    if (address < 0 || address > E2END) return 0;
    return data[address];
  }

  void write(int address, uint8_t value) {
    if (address < 0 || address > E2END) return;
    data[address] = value;
  }

  // update() is the "skip bytes that already match" entry point on real silicon,
  // which is what the store's write budget counts. Over an array there is no
  // wear to save, so it is a plain write here.
  void update(int address, uint8_t value) { write(address, value); }

  // put() is the block entry point on real silicon, and it is the one the store
  // uses: a slot is written as a buffer, never as a typed field.
  void put(int address, const uint8_t* value, size_t length) {
    updateBlock(address, value, length);
  }

  template <typename T>
  void put(int address, T& value) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    updateBlock(address, bytes, sizeof(T));
  }

  void get(int address, uint8_t* out, size_t length) const {
    if (out == nullptr) return;
    for (size_t offset = 0; offset < length; ++offset) {
      const int at = address + static_cast<int>(offset);
      out[offset] = (at < 0 || at > E2END) ? 0 : data[at];
    }
  }

  template <typename T>
  void get(int address, T& out) const {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&out);
    get(address, bytes, sizeof(T));
  }

  void update(int address, const uint8_t* value, size_t length) {
    updateBlock(address, value, length);
  }

  int length() const { return E2END + 1; }

  // A host test that wants a blank device, or a corrupt one, clears it directly
  // through this; the store never does.
  void erase() {
    for (int address = 0; address <= E2END; ++address) data[address] = 0;
  }

 private:
  void updateBlock(int address, const uint8_t* value, size_t length) {
    if (value == nullptr) return;
    for (size_t offset = 0; offset < length; ++offset) {
      write(address + static_cast<int>(offset), value[offset]);
    }
  }
};

extern EEPROMClass EEPROM;

#endif  // ARDUINO_STUB_EEPROM_H