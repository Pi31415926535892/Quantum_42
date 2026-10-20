// stub.cpp — definitions for the host-side Arduino core stand-in.
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

uint8_t TCCR4A = 0;
uint8_t TCCR4B = 0;
uint16_t OCR4A = 0;
uint16_t OCR4B = 0;

std::string gSerialOut;
std::string gSerialIn;

HardwareSerial Serial;
HardwareSerial Serial1;
EEPROMClass EEPROM;
TwoWireClass Wire;

void serialClearOut() { gSerialOut.clear(); }

void serialFeed(const char* line) {
  gSerialIn += line;
  gSerialIn += '\n';
}
