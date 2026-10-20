// Arduino.h — host-side stand-in for the AVR Arduino core, enough to
// -fsyntax-check the .ino delivery files without an AVR toolchain and to drive
// src/rescue_robot/diag.ino from a host test (test/diag_test.cpp).
//
// This is NOT an emulator: pins, timers and millis() are inert. It exists so
// the delivery layer can be type-checked and the diag menu's command routing
// can be asserted. AVR behaviour and motor polarity remain bench questions
// (HARDWARE.md section 7).
//
// Serial I/O is redirected through two globals: gSerialIn holds bytes the
// "terminal" sends, gSerialOut accumulates everything printed. Tests clear
// gSerialOut, feed gSerialIn, call the sketch entry point, then assert.

#ifndef ARDUINO_STUB_ARDUINO_H
#define ARDUINO_STUB_ARDUINO_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// --- Pin modes and levels --------------------------------------------------
constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t INPUT = 0;
constexpr uint8_t OUTPUT = 1;
constexpr uint8_t INPUT_PULLUP = 2;
constexpr uint8_t INPUT_PULLDOWN = 3;

constexpr uint8_t A0 = 54;
constexpr uint8_t A1 = 55;
constexpr uint8_t A2 = 56;
constexpr uint8_t A3 = 57;
constexpr uint8_t A4 = 58;
constexpr uint8_t A5 = 59;
constexpr uint8_t A6 = 60;
constexpr uint8_t A7 = 61;
constexpr uint8_t A8 = 62;
constexpr uint8_t A9 = 63;
constexpr uint8_t A10 = 64;
constexpr uint8_t A11 = 65;
constexpr uint8_t A12 = 66;
constexpr uint8_t A13 = 67;
constexpr uint8_t A14 = 68;
constexpr uint8_t A15 = 69;

#define _BV(bit) (1 << (bit))

// --- Inert Timer4 registers (motors.ino pwmBegin) ---------------------------
// The registers are plain host variables: a write lands and stays there, and a
// test can read back what the PWM setup asked for. The bit names are the real
// avr/io.h numbers, so _BV(WGM40) here selects the same bit it selects on the
// Mega — a wrong allocation in config.h shows up as a wrong register value
// rather than as a silent no-op.
extern uint8_t TCCR4A;
extern uint8_t TCCR4B;
extern uint16_t OCR4A;
extern uint16_t OCR4B;

constexpr uint8_t WGM40 = 0;  // TCCR4A: waveform generation mode
constexpr uint8_t WGM41 = 1;
constexpr uint8_t CS40 = 0;   // TCCR4B: clock select
constexpr uint8_t CS41 = 1;
constexpr uint8_t CS42 = 2;

// --- Redirected serial -----------------------------------------------------
// Defined in stub.cpp. Every print appends here as well as to stdout.
extern std::string gSerialOut;
extern std::string gSerialIn;
void serialClearOut();
void serialFeed(const char* line);

// Minimal stand-in for the Arduino String the legacy sketches use.
class String {
 public:
  String() = default;
  String(const char* s) : buf_(s ? s : "") {}
  String(int v) : buf_(std::to_string(v)) {}
  int indexOf(char c) const {
    const size_t at = buf_.find(c);
    return at == std::string::npos ? -1 : static_cast<int>(at);
  }
  String substring(int from) const {
    return String(buf_.substr(static_cast<size_t>(from)).c_str());
  }
  String substring(int from, int to) const {
    return String(buf_.substr(static_cast<size_t>(from),
                              static_cast<size_t>(to - from)).c_str());
  }
  void trim() {
    const size_t first = buf_.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      buf_.clear();
      return;
    }
    buf_.erase(0, first);
    const size_t last = buf_.find_last_not_of(" \t\r\n");
    buf_.erase(last + 1);
  }
  int toInt() const { return std::atoi(buf_.c_str()); }
  const std::string& c_str() const { return buf_; }

 private:
  std::string buf_;
};

class Print {
 public:
  virtual ~Print() = default;
  void print(const char* s) { emit(s); }
  void print(char c) { emit(std::string(1, c)); }
  void print(int v) { emit(std::to_string(v)); }
  void print(unsigned int v) { emit(std::to_string(v)); }
  void print(long v) { emit(std::to_string(v)); }
  void print(unsigned long v) { emit(std::to_string(v)); }
  void print(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    emit(buf);
  }
  void println() { emit("\n"); }
  void println(char c) { emit(std::string(1, c) + "\n"); }
  void println(const char* s) { emit(std::string(s) + "\n"); }
  void println(int v) { emit(std::to_string(v) + "\n"); }
  void println(long v) { emit(std::to_string(v) + "\n"); }
  void println(unsigned long v) { emit(std::to_string(v) + "\n"); }

 protected:
  void emit(const std::string& s) {
    gSerialOut += s;
    std::fputs(s.c_str(), stdout);
  }
};

class Stream : public Print {
 public:
  int available() { return static_cast<int>(gSerialIn.size()); }
  int peek() { return gSerialIn.empty() ? -1 : static_cast<int>(gSerialIn[0]); }
  int read() {
    if (gSerialIn.empty()) return -1;
    const int byte = static_cast<int>(gSerialIn[0]);
    gSerialIn.erase(0, 1);
    return byte;
  }
  void flush() {}
  // Consume up to and including `terminator`; consume everything when absent.
  String readStringUntil(char terminator) {
    const size_t at = gSerialIn.find(terminator);
    if (at == std::string::npos) {
      String line(gSerialIn.c_str());
      gSerialIn.clear();
      return line;
    }
    String line(gSerialIn.substr(0, at).c_str());
    gSerialIn.erase(0, at + 1);
    return line;
  }
};

class HardwareSerial : public Stream {
 public:
  void begin(unsigned long) {}
  void end() {}
  int availableForWrite() { return 0; }
  size_t write(uint8_t) { return 1; }
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return LOW; }
inline int analogRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
// The real core defines constrain as a macro; a single template is the
// faithful stand-in. Overloads would make calls like constrain(float, int, int)
// ambiguous, which is a stub artifact, not a sketch defect.
template <typename T>
inline T constrain(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}

#endif  // ARDUINO_STUB_ARDUINO_H
