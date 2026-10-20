// diag.ino — Serial diagnostic menu for the bench and pit.
//
// DELIVERY ONLY: it parses one command line, calls the module it names, and
// prints what that module reports. Every decision lives elsewhere — the pin
// table in config.h, direction/polarity/shaping in motors.h, the centroid,
// class and steering in line.h, the link framing and liveness in link.h, the
// victim truth table in sensors.h.
//
// Four command groups, the four that can be checked on a bench with no Nicla
// attached: motor (drive either channel or both, stop), line (one look at the
// QTR-8A bar), link (liveness, policy, tx queue) and probe (the victim verdict).
// Plus 'c' for the compiled-in constants and 'h' for help. Calibration and
// mission behaviour are deliberately absent: they are stateful and belong to
// the sketch that owns the mission, not to a menu poked at mid-run.
//
// Call diagTick() once per main loop. It consumes at most one line per call and
// never blocks, so it cannot delay the caller's control loop.

#include <Arduino.h>

#include "config.h"
#include "diag_parse.h"
#include "line.h"
#include "link.h"
#include "motors.h"
#include "sensors.h"

// Delivery entry points this menu drives, declared so the file is valid C++
// without the IDE's generated prototypes. Definitions live in motors.ino,
// line.ino, link.ino and sensors.ino.
void setMotor(rescue::MotorSide side, int speed);
void setMotor(int leftSpeed, int rightSpeed);
void motorsStop();
rescue::LineReading updateLine();
rescue::VictimClass classifyVictim();
int readReflectivity();
bool readConductivity();
rescue::SurfaceKind probeSurface();
rescue::Liveness linkLiveness();
rescue::LinkPolicy linkPolicy();
uint16_t linkTxOverflows();
uint8_t linkTxQueued();

namespace {

using namespace rescue;  // NOLINT(module types are used unqualified below)

// A command is at most a verb plus two signed speeds, so a short buffer is
// enough; anything longer is rejected rather than allowed to overrun.
constexpr uint8_t DIAG_LINE_MAX = 32;

// --- Formatting helpers ----------------------------------------------------
// The AVR has no std::string and String is heap-backed, so the menu prints
// straight to Serial from fixed values.

void printLabeled(const char* label, long value) {
  Serial.print(label);
  Serial.print(value);
}

// Gains are fractional; printing them through the integer path would report 0
// and hide a mis-tuned Kp.
void printGain(const char* label, double value) {
  Serial.print(label);
  Serial.print(value);
}

void printLabeled(const char* label, const char* value) {
  Serial.print(label);
  Serial.println(value);
}

// The module enums name a state, and a diagnostic menu that prints a raw
// integer is useless. Each enum is mapped through a name table indexed by its
// underlying value, which is also what a name table exists for: adding a state
// to a module enum without a name here degrades to UNKNOWN instead of printing
// a number nobody can interpret.
template <typename Enum, size_t Count>
struct EnumNames {
  const char* names[Count];
  const char* name(Enum value) const {
    const size_t index = static_cast<size_t>(value);
    return index < Count ? names[index] : "UNKNOWN";
  }
};

// The name tables below are indexed by the enum's underlying value, so each
// entry must sit in the SAME order as the enumerators in the module header:
// LineClass is ON_LINE, EDGE, ALL_WHITE, ALL_DARK and SurfaceKind is Black,
// Silver. test/diag_test.cpp asserts the mapping so a reordering upstream is
// caught here rather than showing up as a mislabelled reading on the bench.
const EnumNames<LineClass, 4> kLineClassNames = {
    {"ON_LINE", "EDGE", "ALL_WHITE", "ALL_DARK"}};
const EnumNames<Liveness, 3> kLivenessNames = {
    {"Fresh", "Stale", "Lost"}};
const EnumNames<LinkPolicy, 3> kPolicyNames = {
    {"LINK_FRESH", "HOLD_LAST_GOOD", "DEGRADE_LINE_ONLY"}};
const EnumNames<VictimClass, 3> kVictimNames = {
    {"Live", "Dead", "Fake"}};
const EnumNames<SurfaceKind, 2> kSurfaceNames = {{"BLACK", "SILVER"}};

const char* directionName(MotorDirection direction) {
  return direction == MotorDirection::Forward ? "FWD" : "REV";
}

const char* sideName(MotorSide side) {
  return side == MotorSide::Left ? "LEFT" : "RIGHT";
}

// "cmd 150 FWD" — the command the drive layer will actually execute.
void printCommand(const MotorCommand& command) {
  printLabeled("cmd ", command.pwm);
  printLabeled(" ", directionName(command.direction));
}

// --- Command groups --------------------------------------------------------

void printHelp() {
  Serial.println("diag menu — one command per line, ? for this help");
  Serial.println("  c            print compiled-in constants");
  Serial.println("  motor group:");
  Serial.println("    m L,R      drive both wheels   (m 150,150  m 150,-150)");
  Serial.println("    f S        drive LEFT only");
  Serial.println("    r S        drive RIGHT only");
  Serial.println("    x          stop both");
  Serial.println("  line  group: l          centroid, error, class");
  Serial.println("  link  group: k          liveness, policy, tx queue");
  Serial.println("  probe group: p          reflectivity, conductivity, verdict");
}

void printConstants() {
  // Printed from config.h, so a bench engineer can diff this against the
  // HARDWARE.md section 6 table and catch a wrong flash at the terminal
  // rather than by watching the robot spin.
  printLabeled("LEFT  in1=", MOTOR_LEFT_IN1);
  printLabeled(" in2=", MOTOR_LEFT_IN2);
  printLabeled(" pwm=", MOTOR_LEFT_PWM);
  Serial.println("  (L298N channel B)");
  printLabeled("RIGHT in1=", MOTOR_RIGHT_IN1);
  printLabeled(" in2=", MOTOR_RIGHT_IN2);
  printLabeled(" pwm=", MOTOR_RIGHT_PWM);
  Serial.println("  (L298N channel A)");
  printLabeled("speed min=", MOTOR_MIN_SPEED);
  printLabeled(" max=", MOTOR_MAX_SPEED);
  printLabeled(" trim_left=", MOTOR_TRIM_LEFT);
  Serial.println();
  printLabeled("line center=", LINE_POSITION_CENTER);
  printLabeled(" present=", LINE_PRESENT_THRESHOLD);
  Serial.println();
  printGain("steer kp=", LINE_KP);
  printGain(" kd=", LINE_KD);
  printLabeled(" max_corr=", LINE_MAX_CORRECTION);
  Serial.println();
  printLabeled("reflect silver>=", REFLECTIVITY_SILVER_THRESHOLD);
  printLabeled(" hyst=", REFLECTIVITY_HYSTERESIS);
  Serial.println();
}

// Report the command the drive layer will actually execute, so a bench
// operator sees the shaped value rather than the request.
void printDriveResult(int leftSpeed, int rightSpeed) {
  const DriveCommand drive = driveFor(leftSpeed, rightSpeed);
  printLabeled("cmd LEFT  ", directionName(drive.left.direction));
  printLabeled("  pwm ", drive.left.pwm);
  Serial.println();
  printLabeled("cmd RIGHT ", directionName(drive.right.direction));
  printLabeled("  pwm ", drive.right.pwm);
  Serial.println();
}

void runMotorBoth(int leftSpeed, int rightSpeed) {
  setMotor(leftSpeed, rightSpeed);
  printLabeled("LEFT  = ", leftSpeed);
  printLabeled("  RIGHT = ", rightSpeed);
  Serial.println();
  printDriveResult(leftSpeed, rightSpeed);
}

void runMotorSide(MotorSide side, int speed) {
  setMotor(side, speed);
  printLabeled(sideName(side), speed);
  Serial.println();
  printCommand(sideCommand(side, speed));
}

void runLine() {
  const LineReading reading = updateLine();
  printLabeled("pos=", reading.position);
  printLabeled(" err=", reading.error);
  Serial.println();
  Serial.print("class=");
  Serial.println(kLineClassNames.name(reading.qualifier));
  Serial.print("line=");
  Serial.println(reading.lineDetected ? "SEEN" : "LOST");
}

void runLink() {
  Serial.print("liveness=");
  Serial.println(kLivenessNames.name(linkLiveness()));
  Serial.print("policy=");
  Serial.println(kPolicyNames.name(linkPolicy()));
  printLabeled("tx_queued=", linkTxQueued());
  printLabeled(" tx_overflows=", linkTxOverflows());
  Serial.println();
}

void runProbe() {
  // classifyVictim() advances the hysteresis state, so it is the reading of
  // record; the raw probe values are printed alongside for tuning.
  const VictimClass victim = classifyVictim();
  printLabeled("reflectivity=", readReflectivity());
  printLabeled(" conductive=", readConductivity() ? 1 : 0);
  Serial.println();
  Serial.print("surface=");
  Serial.println(kSurfaceNames.name(probeSurface()));
  Serial.print("victim=");
  Serial.println(kVictimNames.name(victim));
}

void dispatch(const ParsedCommand& command) {
  switch (command.verb) {
    case '\0': return;  // blank line: nothing to do
    case 'h': printHelp(); return;
    case '?': printHelp(); return;
    case 'c': printConstants(); return;
    case 'm':
      if (command.argCount < 2) {
        Serial.println("ERR: m <left>,<right>");
        return;
      }
      runMotorBoth(command.first, command.second);
      return;
    case 'f':
      if (command.argCount < 1) {
        Serial.println("ERR: f <speed>");
        return;
      }
      runMotorSide(MotorSide::Left, command.first);
      return;
    case 'r':
      if (command.argCount < 1) {
        Serial.println("ERR: r <speed>");
        return;
      }
      runMotorSide(MotorSide::Right, command.first);
      return;
    case 'x':
      motorsStop();
      Serial.println("stopped");
      return;
    case 'l': runLine(); return;
    case 'k': runLink(); return;
    case 'p': runProbe(); return;
    default:
      Serial.print("ERR: unknown '");
      Serial.println(command.verb);
      printHelp();
      return;
  }
}

}  // namespace

// Consume at most one command line from Serial. Call once per main loop.
void diagTick() {
  if (Serial.available() == 0) return;

  char line[DIAG_LINE_MAX];
  uint8_t length = 0;
  bool overflow = false;

  // Read up to the terminator, or until the buffer is full. Bytes beyond the
  // buffer are still consumed so the next call starts on a command boundary.
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    if (incoming == '\n' || incoming == '\r') break;
    if (length + 1 < DIAG_LINE_MAX) {
      line[length++] = static_cast<char>(incoming);
    } else {
      overflow = true;
    }
  }
  line[length] = '\0';

  if (overflow) {
    Serial.println("ERR: command too long");
    printHelp();
    return;
  }
  dispatch(parseLine(line, length));
}
