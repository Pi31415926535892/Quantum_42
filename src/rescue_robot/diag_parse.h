// diag_parse.h — pure host-compilable command-line parsing for the Serial
// diagnostic menu (C++14, no AVR I/O).
//
// The menu reads one command per line from a fixed-size buffer, so the parser
// takes a (pointer, length) pair rather than a String: no heap, no
// termination assumptions, and no way to overrun the caller's buffer. It lives
// here rather than in diag.ino because it is pure logic with no Serial in it,
// which makes it directly testable on the host.
//
// The grammar is deliberately tiny — a verb plus up to two signed integers —
// because the menu is a bench tool, and a bench tool that misreads a command is
// worse than one that rejects it.

#ifndef RESCUE_ROBOT_DIAG_PARSE_H
#define RESCUE_ROBOT_DIAG_PARSE_H

#include <cstdint>

namespace rescue {

// One parsed command line. A zero verb means the line held no command.
struct ParsedCommand {
  char verb;         // first non-blank character, lowercased
  int first;         // first numeric argument, 0 when absent
  int second;        // second numeric argument, 0 when absent
  uint8_t argCount;  // how many arguments were present, 0..2
};

// A cursor over the argument text, holding the number currently being read.
struct ArgumentScanner {
  int first = 0;
  int second = 0;
  uint8_t count = 0;

  bool inNumber = false;
  bool negative = false;
  int value = 0;

  // Close the number being read and roll over to the next argument. A rollover
  // past the second argument is dropped rather than overwriting it, so extra
  // numbers on a line are ignored instead of corrupting an otherwise valid
  // pair.
  void endNumber() {
    if (!inNumber) return;
    if (count < 2) {
      const int finished = negative ? -value : value;
      if (count == 0) {
        first = finished;
      } else {
        second = finished;
      }
      ++count;
    }
    value = 0;
    negative = false;
  }

  void beginNumber(bool isNegative) {
    inNumber = true;
    negative = isNegative;
    value = 0;
  }
};

// Read the numeric arguments that follow the verb. A sign always starts a new
// number, and space, tab, comma and CR all end one, so "150,150", "150 150"
// and "150 -150" are the same command.
inline void scanArguments(const char* line, uint8_t length, uint8_t from,
                          ArgumentScanner& scan) {
  for (uint8_t index = from; index < length; ++index) {
    const char c = line[index];
    if (c >= '0' && c <= '9') {
      scan.inNumber = true;
      scan.value = scan.value * 10 + (c - '0');
      continue;
    }
    if (c == '-' || c == '+') {
      scan.endNumber();
      scan.beginNumber(c == '-');
      continue;
    }
    scan.endNumber();
    scan.inNumber = false;
  }
  scan.endNumber();
}

// Parse one line. Leading blanks are skipped; the first remaining character is
// the verb. A blank line yields a zero verb and no arguments, which the caller
// treats as "nothing to do".
inline ParsedCommand parseLine(const char* line, uint8_t length) {
  ParsedCommand parsed = {'\0', 0, 0, 0};
  uint8_t index = 0;
  while (index < length && (line[index] == ' ' || line[index] == '\t')) ++index;
  if (index >= length) return parsed;

  const char verb = line[index];
  parsed.verb = (verb >= 'A' && verb <= 'Z') ? verb - 'A' + 'a' : verb;
  ++index;

  ArgumentScanner scan;
  scanArguments(line, length, index, scan);
  parsed.first = scan.first;
  parsed.second = scan.second;
  parsed.argCount = scan.count;
  return parsed;
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_DIAG_PARSE_H
