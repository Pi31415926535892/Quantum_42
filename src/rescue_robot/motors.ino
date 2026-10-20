// motors.ino — AVR delivery layer for the drivetrain.
//
// Owns all Arduino-side I/O on behalf of motors.h: pin modes, Timer4 PWM
// setup on D6/D7, and the setMotor() entry points. Contains zero business
// logic — every decision (direction, polarity, clamping, trim) lives in
// config.h/motors.h.

#include <Arduino.h>
#include "config.h"
#include "motors.h"

using namespace rescue;

namespace {

// Push one channel's drive command out to its L298N pins.
void writeChannel(const MotorChannelPins& pins, const MotorCommand& cmd) {
  const ChannelPolarity pol = polarityFor(cmd.direction);
  digitalWrite(pins.in1, pol.in1 == PinState::High ? HIGH : LOW);
  digitalWrite(pins.in2, pol.in2 == PinState::High ? HIGH : LOW);
  analogWrite(pins.pwm, cmd.pwm);
}

}  // namespace

// Timer4 phase-correct 8-bit PWM on D6 (OC4A) / D7 (OC4B), prescaler /64.
// The registers below ARE Timer4, so the config allocation must agree.
static_assert(MOTOR_PWM_TIMER == 4, "motor PWM timer allocation changed");
void pwmBegin() {
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS40) | _BV(CS41);
  OCR4A = 0;
  OCR4B = 0;
}

// Set pin modes, arm Timer4 PWM, and park every motor to a stopped state.
void motorsBegin() {
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  pwmBegin();

  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// Drive a single side with a signed speed.
void setMotor(MotorSide side, int speed) {
  writeChannel(pinsFor(side), sideCommand(side, speed));
}

// Drive both sides with signed speeds.
void setMotor(int leftSpeed, int rightSpeed) {
  const DriveCommand cmd = driveFor(leftSpeed, rightSpeed);
  writeChannel(pinsFor(MotorSide::Left), cmd.left);
  writeChannel(pinsFor(MotorSide::Right), cmd.right);
}

// Command a full stop (both channels, zero duty, forward polarity).
void motorsStop() {
  setMotor(0, 0);
}