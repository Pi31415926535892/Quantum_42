// motors_test.cpp — host-side tests for src/rescue_robot/motors.h (pure C++14).
// Build/run: g++ -std=c++14 test/motors_test.cpp -o /tmp/mtest && /tmp/mtest

#include "../src/rescue_robot/motors.h"

#include <cstdio>

namespace {

int gFailures = 0;

#define EXPECT_TRUE(cond)                                                 \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
      ++gFailures;                                                        \
    }                                                                     \
  } while (0)

#define EXPECT_EQ(a, b)                                                   \
  do {                                                                    \
    const auto va = (a);                                                  \
    const auto vb = (b);                                                  \
    if (!(va == vb)) {                                                    \
      std::printf("FAIL %s:%d  %s == %s  (got %d, want %d)\n",            \
                  __FILE__, __LINE__, #a, #b, (int)va, (int)vb);          \
      ++gFailures;                                                        \
    }                                                                     \
  } while (0)

}  // namespace

int main() {
  const rescue::MotorDirection FORWARD = rescue::MotorDirection::Forward;
  const rescue::MotorDirection REVERSE = rescue::MotorDirection::Reverse;

  // Physical channel tables (ground truth: LEFT = L298N Ch B, RIGHT = Ch A).
  EXPECT_EQ(rescue::LEFT_CHANNEL_PINS.pwm, 6);
  EXPECT_EQ(rescue::LEFT_CHANNEL_PINS.in1, 4);
  EXPECT_EQ(rescue::LEFT_CHANNEL_PINS.in2, 2);
  EXPECT_EQ(rescue::RIGHT_CHANNEL_PINS.pwm, 7);
  EXPECT_EQ(rescue::RIGHT_CHANNEL_PINS.in1, 3);
  EXPECT_EQ(rescue::RIGHT_CHANNEL_PINS.in2, 5);

  // pinsFor resolves the physical tables.
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Left).pwm, 6);
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Right).pwm, 7);

  // L298N polarity: channel input-1 HIGH, input-2 LOW = forward.
  {
    const rescue::ChannelPolarity fwd = rescue::polarityFor(FORWARD);
    EXPECT_TRUE(fwd.in1 == rescue::PinState::High && fwd.in2 == rescue::PinState::Low);
    const rescue::ChannelPolarity rev = rescue::polarityFor(REVERSE);
    EXPECT_TRUE(rev.in1 == rescue::PinState::Low && rev.in2 == rescue::PinState::High);
  }

  // MotorSide tables cover both channels with every field.
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Right).in1, 3);
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Right).in2, 5);
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Left).in1, 4);
  EXPECT_EQ(rescue::pinsFor(rescue::MotorSide::Left).in2, 2);

  // Command clamping behaviour (symmetric, respects the configured ceiling).
  EXPECT_EQ(rescue::clampSpeed(250), rescue::MOTOR_MAX_SPEED);
  EXPECT_EQ(rescue::clampSpeed(-250), -rescue::MOTOR_MAX_SPEED);
  EXPECT_EQ(rescue::clampSpeed(150), 150);
  EXPECT_EQ(rescue::clampSpeed(-150), -150);
  EXPECT_EQ(rescue::clampSpeed(0), 0);

  // Minimum-duty lift: stop stays stop, small drives raise to the floor.
  EXPECT_EQ(rescue::liftToMinimum(0), 0);
  EXPECT_EQ(rescue::liftToMinimum(30), rescue::MOTOR_MIN_SPEED);
  EXPECT_EQ(rescue::liftToMinimum(-30), -rescue::MOTOR_MIN_SPEED);
  EXPECT_EQ(rescue::liftToMinimum(150), 150);

  // Direction sign rule.
  EXPECT_TRUE(rescue::directionOf(150) == FORWARD);
  EXPECT_TRUE(rescue::directionOf(-150) == REVERSE);
  EXPECT_TRUE(rescue::directionOf(0) == FORWARD);
  EXPECT_TRUE(rescue::directionOf(-1) == REVERSE);

  // commandFor: signed speed -> {direction, duty magnitude}.
  {
    const rescue::MotorCommand cmd = rescue::commandFor(150);
    EXPECT_TRUE(cmd.direction == FORWARD);
    EXPECT_EQ(cmd.pwm, 150);
  }
  {
    const rescue::MotorCommand cmd = rescue::commandFor(-150);
    EXPECT_TRUE(cmd.direction == REVERSE);
    EXPECT_EQ(cmd.pwm, 150);
  }
  {
    const rescue::MotorCommand cmd = rescue::commandFor(0);
    EXPECT_TRUE(cmd.direction == FORWARD);
    EXPECT_EQ(cmd.pwm, 0);
  }
  {
    const rescue::MotorCommand cmd = rescue::commandFor(30);
    EXPECT_EQ(cmd.pwm, rescue::MOTOR_MIN_SPEED);
  }
  {
    const rescue::MotorCommand cmd = rescue::commandFor(250);
    EXPECT_EQ(cmd.pwm, rescue::MOTOR_MAX_SPEED);
  }

  // sideCommand: per-side path (left side carries the straight-trim slot).
  {
    const rescue::MotorCommand cmd = rescue::sideCommand(rescue::MotorSide::Left, 150);
    EXPECT_TRUE(cmd.direction == FORWARD);
    EXPECT_EQ(cmd.pwm, 150);
  }
  {
    const rescue::MotorCommand cmd = rescue::sideCommand(rescue::MotorSide::Right, -150);
    EXPECT_TRUE(cmd.direction == REVERSE);
    EXPECT_EQ(cmd.pwm, 150);
  }

  // 150,150 -> both channels forward (drives straight).
  {
    const rescue::DriveCommand cmd = rescue::driveFor(150, 150);
    EXPECT_TRUE(cmd.left.direction == FORWARD);
    EXPECT_TRUE(cmd.right.direction == FORWARD);
    EXPECT_EQ(cmd.left.pwm, 150);
    EXPECT_EQ(cmd.right.pwm, 150);
  }

  // 150,-150 -> left forward / right reverse (in-place pivot).
  {
    const rescue::DriveCommand cmd = rescue::driveFor(150, -150);
    EXPECT_TRUE(cmd.left.direction == FORWARD);
    EXPECT_TRUE(cmd.right.direction == REVERSE);
    EXPECT_EQ(cmd.left.pwm, 150);
    EXPECT_EQ(cmd.right.pwm, 150);
  }

  // -150,150 -> mirror pivot.
  {
    const rescue::DriveCommand cmd = rescue::driveFor(-150, 150);
    EXPECT_TRUE(cmd.left.direction == REVERSE);
    EXPECT_TRUE(cmd.right.direction == FORWARD);
    EXPECT_EQ(cmd.left.pwm, 150);
    EXPECT_EQ(cmd.right.pwm, 150);
  }

  if (gFailures == 0) {
    std::printf("ALL PASS\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", gFailures);
  return 1;
}