// motors.h — pure host-compilable drivetrain logic (C++14, no AVR I/O).
//
// Maps signed drive speeds to per-channel motor commands using the physical
// channel/polarity ground truth from HARDWARE.md, which is injected through
// config.h. The AVR delivery layer (motors.ino) is the only file that may
// touch pins or timers.

#ifndef RESCUE_ROBOT_MOTORS_H
#define RESCUE_ROBOT_MOTORS_H

#include <cstdint>
#include "config.h"

namespace rescue {

// ---------------------------------------------------------------------------
// Physical channel layout
// ---------------------------------------------------------------------------
// LEFT  = L298N channel B = {pwm: ENB/D6, in1: IN3/D4, in2: IN4/D2}
// RIGHT = L298N channel A = {pwm: ENA/D7, in1: IN1/D3, in2: IN2/D5}

enum class MotorSide { Left, Right };

struct MotorChannelPins {
  uint8_t pwm;  // channel enable pin (duty via analog PWM)
  uint8_t in1;  // first direction input of the L298N channel
  uint8_t in2;  // second direction input of the L298N channel
};

constexpr MotorChannelPins LEFT_CHANNEL_PINS = {
    MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2};
constexpr MotorChannelPins RIGHT_CHANNEL_PINS = {
    MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2};

inline const MotorChannelPins& pinsFor(MotorSide side) {
  return side == MotorSide::Left ? LEFT_CHANNEL_PINS : RIGHT_CHANNEL_PINS;
}

// ---------------------------------------------------------------------------
// Direction and polarity
// ---------------------------------------------------------------------------

enum class MotorDirection { Forward, Reverse };

// Driver pin states that realise a direction on an L298N channel.
enum class PinState { Low, High };

struct ChannelPolarity {
  PinState in1;
  PinState in2;
};

// L298N forward convention: channel input-1 HIGH, input-2 LOW. On the fixed
// wiring this means Ch B (left) runs IN3=1/IN4=0 and Ch A (right) runs
// IN1=1/IN2=0 — both channels drive physically forward.
inline ChannelPolarity polarityFor(MotorDirection direction) {
  if (direction == MotorDirection::Reverse) {
    return ChannelPolarity{PinState::Low, PinState::High};
  }
  return ChannelPolarity{PinState::High, PinState::Low};
}

inline MotorDirection directionOf(int speed) {
  return speed < 0 ? MotorDirection::Reverse : MotorDirection::Forward;
}

// ---------------------------------------------------------------------------
// Speed shaping
// ---------------------------------------------------------------------------

// Symmetric clamp into [-MOTOR_MAX_SPEED, +MOTOR_MAX_SPEED].
inline int clampSpeed(int speed) {
  if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
  if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
  return speed;
}

// Raise a non-zero magnitude sitting below the minimum duty up to it. Zero
// stays zero so a commanded stop is always a hard stop.
inline int liftToMinimum(int speed) {
  const int magnitude = speed < 0 ? -speed : speed;
  if (magnitude == 0 || magnitude >= MOTOR_MIN_SPEED) return speed;
  return speed < 0 ? -MOTOR_MIN_SPEED : MOTOR_MIN_SPEED;
}

// ---------------------------------------------------------------------------
// Command mapping
// ---------------------------------------------------------------------------

struct MotorCommand {
  MotorDirection direction;
  int pwm;  // duty magnitude in 0..MOTOR_MAX_SPEED
};

struct DriveCommand {
  MotorCommand left;
  MotorCommand right;
};

// Signed speed -> per-channel drive command (direction + duty magnitude).
inline MotorCommand commandFor(int speed) {
  const int shaped = liftToMinimum(clampSpeed(speed));
  return MotorCommand{directionOf(shaped), shaped < 0 ? -shaped : shaped};
}

// Per-side command; the straight-trim slot applies to the LEFT side only.
inline MotorCommand sideCommand(MotorSide side, int speed) {
  return commandFor(side == MotorSide::Left ? speed + MOTOR_TRIM_LEFT : speed);
}

// Both-channel drive command from signed left/right speeds.
inline DriveCommand driveFor(int leftSpeed, int rightSpeed) {
  return DriveCommand{
      sideCommand(MotorSide::Left, leftSpeed),
      sideCommand(MotorSide::Right, rightSpeed)};
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_MOTORS_H