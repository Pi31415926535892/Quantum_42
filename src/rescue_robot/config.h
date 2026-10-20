// config.h — single source of truth for drivetrain constants.
//
// Physical ground truth lives in HARDWARE.md (sections 2 and 6): the LEFT
// motor pair is wired to L298N channel B, the RIGHT pair to channel A. No
// AVR I/O happens in this file; it is host-compilable C++14.

#ifndef RESCUE_ROBOT_CONFIG_H
#define RESCUE_ROBOT_CONFIG_H

#include <cstdint>

namespace rescue {

// ---------------------------------------------------------------------------
// Motor pins
// ---------------------------------------------------------------------------

// L298N enable pins (PWM-capable). The enable drives the duty cycle.
constexpr uint8_t MOTOR_LEFT_PWM = 6;   // D6 -> ENB (channel B, left pair)
constexpr uint8_t MOTOR_RIGHT_PWM = 7;  // D7 -> ENA (channel A, right pair)

// L298N direction inputs. in1/in2 are the channel's two direction lines.
constexpr uint8_t MOTOR_LEFT_IN1 = 4;   // D4 -> IN3 (channel B)
constexpr uint8_t MOTOR_LEFT_IN2 = 2;   // D2 -> IN4 (channel B)
constexpr uint8_t MOTOR_RIGHT_IN1 = 3;  // D3 -> IN1 (channel A)
constexpr uint8_t MOTOR_RIGHT_IN2 = 5;  // D5 -> IN2 (channel A)

// ---------------------------------------------------------------------------
// Speed bounds (PWM duty, 0..255)
// ---------------------------------------------------------------------------

constexpr int MOTOR_MAX_SPEED = 200;  // hard ceiling for any commanded speed
constexpr int MOTOR_MIN_SPEED = 45;   // minimum useful duty while moving

// Straight-trim slot: drift correction added to the LEFT command. Positive
// trims the left side up (robot yaws right), negative trims it down (yaws
// left). 0 = no trim. A right-side trim is the negated value.
constexpr int MOTOR_TRIM_LEFT = 0;

// ---------------------------------------------------------------------------
// Line sensor pins
// ---------------------------------------------------------------------------
//
// QTR-8A OUT1..OUT8, leftmost sensor first (HARDWARE.md §4): the bar's connector
// was flipped on 2026-10-20, so OUT1 (leftmost) now lands on A14 and OUT8
// (rightmost) on A0, at the locked 11.2 mm ride height. The table stays in
// logical order (index 0 = leftmost), which is why the analog numbers descend.
// The even-numbered analog inputs are the even ones because the odd neighbours
// are the victim's reflectivity (A1) and the conductancy pair.
//
// The Mega's analog inputs are numbered 54..69, so A0 is 54 and A14 is 68.
// These are written as numbers rather than as A0/A2/... on purpose: this file is
// host-compilable C++14 and must not need <Arduino.h> to say which pin a sensor
// is on. line.ino static_asserts every entry against the real A-names, so the
// board build fails loudly if these ever drift from the wiring above — the host
// build and the wiring document agree by construction, not by trust.
//
// One table, two readers: the line module follows the bar every tick, and the
// calibration sweep sweeps the same bar over the same eight pins. Left unsized
// so this file needs no channel count of its own — line.h owns that one, and
// line.ino static_asserts the two agree.
constexpr uint8_t LINE_SENSOR_PINS[] = {
    68,  // A14 — OUT1 (leftmost)
    66,  // A12 — OUT2
    64,  // A10 — OUT3
    62,  // A8  — OUT4
    60,  // A6  — OUT5
    58,  // A4  — OUT6
    56,  // A2  — OUT7
    54,  // A0  — OUT8 (rightmost)
};

// ---------------------------------------------------------------------------
// Status lamp
// ---------------------------------------------------------------------------

// The on-board LED on D13: the one diode that needs no wiring to check. It
// carries the status signals in status.h — steady while healthy, a blink shape
// for each latched reason the robot stopped trusting the field.
constexpr uint8_t STATUS_LED_PIN = 13;

// ---------------------------------------------------------------------------
// Timers
// ---------------------------------------------------------------------------

constexpr uint8_t MOTOR_PWM_TIMER = 4;  // motor PWM timer driving D6/D7

}  // namespace rescue

#endif  // RESCUE_ROBOT_CONFIG_H