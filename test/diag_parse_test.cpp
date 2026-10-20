// diag_parse_test.cpp — host-side tests for src/rescue_robot/diag_parse.h.
//
// The menu's parser is the one place a mistyped bench command turns into a
// surprise at the motor pins, so its edge cases are pinned here: the separators
// people actually type, a sign that could be read as mid-number, an over-long
// argument list, and a blank line.
//
// Pure C++14, no Arduino. Build/run: ./test/run_tests.sh.

#include <cstdio>
#include <cstring>
#include <string>

#include "diag_parse.h"

namespace {

int gFailures = 0;
int gChecks = 0;

void report(bool ok, const std::string& what) {
  ++gChecks;
  if (!ok) {
    ++gFailures;
    std::printf("FAIL diag_parse: %s\n", what.c_str());
  }
}

rescue::ParsedCommand parse(const char* text) {
  return rescue::parseLine(text, static_cast<uint8_t>(std::strlen(text)));
}

}  // namespace

int main() {
  using rescue::ParsedCommand;

  // A bare verb: no arguments at all.
  {
    const ParsedCommand p = parse("h");
    report(p.verb == 'h', "'h' did not parse as the verb h");
    report(p.argCount == 0, "'h' reported arguments");
  }

  // Uppercase is folded, so HELP works without a second case arm.
  {
    const ParsedCommand p = parse("H");
    report(p.verb == 'h', "an uppercase verb was not folded to lowercase");
  }

  // Leading blanks are skipped.
  {
    const ParsedCommand p = parse("   \t m 150");
    report(p.verb == 'm', "leading blanks were not skipped");
    report(p.first == 150, "leading blanks swallowed the first argument");
  }

  // A blank line is "no command", not "verb zero with arguments".
  {
    const ParsedCommand p = parse("");
    report(p.verb == '\0', "an empty line did not yield a zero verb");
    report(p.argCount == 0, "an empty line reported arguments");
  }
  {
    const ParsedCommand p = parse("   ");
    report(p.verb == '\0', "a whitespace-only line did not yield a zero verb");
  }

  // The section 7 checklist commands, in the separator forms an operator
  // actually types.
  {
    const ParsedCommand p = parse("m 150,150");
    report(p.verb == 'm' && p.argCount == 2, "'m 150,150' did not parse");
    report(p.first == 150 && p.second == 150,
           "'m 150,150' produced the wrong speeds");
  }
  {
    const ParsedCommand p = parse("m 150 -150");
    report(p.argCount == 2, "'m 150 -150' did not see two arguments");
    report(p.first == 150 && p.second == -150,
           "'m 150 -150' produced the wrong speeds");
  }
  {
    const ParsedCommand p = parse("m-150,150");
    report(p.argCount == 2, "'m-150,150' did not see two arguments");
    report(p.first == -150 && p.second == 150,
           "'m-150,150' produced the wrong speeds");
  }
  {
    const ParsedCommand p = parse("m 150 50");
    report(p.first == 150 && p.second == 50,
           "'m 150 50' produced the wrong speeds");
  }

  // A sign must start a NEW number, never be applied mid-number: "150-150" is
  // two arguments, not one malformed 150.
  {
    const ParsedCommand p = parse("m 150-150");
    report(p.argCount == 2, "'m 150-150' was read as a single argument");
    report(p.first == 150 && p.second == -150,
           "'m 150-150' produced the wrong speeds");
  }

  // An explicit plus sign is accepted.
  {
    const ParsedCommand p = parse("m +150,-150");
    report(p.first == 150 && p.second == -150,
           "'m +150,-150' mishandled the explicit plus sign");
  }

  // A single-argument motor command.
  {
    const ParsedCommand p = parse("f -150");
    report(p.verb == 'f' && p.argCount == 1, "'f -150' did not parse");
    report(p.first == -150, "'f -150' produced the wrong speed");
  }

  // A zero argument is a real argument, not a missing one.
  {
    const ParsedCommand p = parse("m 0,0");
    report(p.argCount == 2, "'m 0,0' did not see two arguments");
    report(p.first == 0 && p.second == 0, "'m 0,0' lost the zeros");
  }

  // A missing argument must be visible as a low count, so a command needing
  // two values refuses to act rather than silently driving with a zero.
  {
    const ParsedCommand p = parse("m 150");
    report(p.argCount == 1, "'m 150' did not report exactly one argument");
  }

  // A third number must not overwrite the pair.
  {
    const ParsedCommand p = parse("m 150,-150,999");
    report(p.argCount == 2, "a third number was counted as an argument");
    report(p.first == 150 && p.second == -150,
           "a third number corrupted the pair");
  }

  // Trailing junk is ignored, and CR is treated as a separator so a line
  // arriving with CRLF endings parses the same.
  {
    const ParsedCommand p = parse("m 150,150\r");
    report(p.first == 150 && p.second == 150, "a CR ending broke the parse");
  }
  {
    const ParsedCommand p = parse("m 150,150extra");
    report(p.first == 150 && p.second == 150, "trailing junk broke the parse");
  }

  // A non-numeric argument position yields no argument rather than garbage.
  {
    const ParsedCommand p = parse("m abc");
    report(p.argCount == 0, "a non-numeric argument was counted");
  }

  std::printf("[diag_parse] checks: %d  failures: %d\n", gChecks, gFailures);
  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  return 1;
}
