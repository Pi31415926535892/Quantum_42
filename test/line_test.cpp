// line_test.cpp — host-side tests for src/rescue_robot/line.h (pure C++14).
//
// The sensor bar reads the black line as a HIGH analog value and the white
// floor as a LOW one, so these tests install that calibration and drive the
// classifier with synthetic vectors. Building this file on the host is also
// the proof that line.h carries no AVR-only identifiers.

#include "line.h"
#include "dryframe.h"

#include <cstdint>

namespace {

// Raw analog values recorded on the two extremes of a black line on a white
// floor, per the QTR-8A analog output.
constexpr int32_t BLACK_READING = 1023;
constexpr int32_t WHITE_READING = 0;

// Bit masks naming which sensors of the bar sit over the black line.
constexpr uint8_t LEFT_EDGE_CHANNELS = 0x01;   // channel 0 only
constexpr uint8_t RIGHT_EDGE_CHANNELS = 0x80;  // channel 7 only
constexpr uint8_t CENTRED_PAIR = 0x18;         // channels 3 and 4
constexpr uint8_t CENTRED_QUAD = 0x66;         // channels 1, 2, 5 and 6
constexpr uint8_t EVERY_CHANNEL = 0xFF;        // the whole bar
constexpr uint8_t OFF_CENTRE_PAIR = 0x28;      // channels 3 and 5

// Centroid errors that drive the correction past its cap, and one that does
// not. The gain is 0.10 and the cap is 65, so the boundary sits near 650.
constexpr int32_t ERROR_BEYOND_CAP = 2000;
constexpr int32_t ERROR_WITH_HEADROOM = 400;

// Calibration covering the full analog range on every channel.
rescue::LineCalibration fullContrastCalibration() {
  return rescue::defaultCalibration(BLACK_READING, WHITE_READING);
}

// Raw vector with the sensor bar over the black line wherever the bit is set.
rescue::LineVector patternOn(uint8_t mask, int32_t blackRaw, int32_t whiteRaw) {
  rescue::LineVector samples{};
  for (uint8_t channel = 0; channel < rescue::LINE_SENSOR_COUNT; ++channel) {
    const uint8_t bit = static_cast<uint8_t>(1u << channel);
    samples.reading[channel] = (mask & bit) ? blackRaw : whiteRaw;
  }
  return samples;
}

rescue::LineVector blackPattern(uint8_t mask) {
  return patternOn(mask, BLACK_READING, WHITE_READING);
}

}  // namespace

int main() {
  // ---- sensor geometry --------------------------------------------------
  // Channel 0 is the leftmost sensor (weight 0), channel 7 the rightmost
  // (weight 7000); the centre of the range sits at 3500.
  CHECK_EQ(rescue::sensorPosition(0), 0);
  CHECK_EQ(rescue::sensorPosition(7), 7000);
  CHECK_EQ(rescue::sensorPosition(3), 3000);
  CHECK_EQ(rescue::LINE_POSITION_CENTER, 3500);

  // ---- centroid of symmetric vectors ------------------------------------
  rescue::setCalibration(fullContrastCalibration());

  // A line centred between channels 3 and 4 has no left/right bias.
  {
    const rescue::LineReading reading = rescue::analyze(blackPattern(CENTRED_PAIR));
    CHECK_EQ(reading.position, 3500);
    CHECK_EQ(reading.error, 0);
  }

  // Four channels lit symmetrically about the centre.
  {
    const rescue::LineReading reading = rescue::analyze(blackPattern(CENTRED_QUAD));
    CHECK_EQ(reading.position, 3500);
  }

  // A uniformly lit bar has no bias either.
  {
    const rescue::LineReading reading = rescue::analyze(blackPattern(EVERY_CHANNEL));
    CHECK_EQ(reading.position, 3500);
  }

  // ---- the line sitting to the right must read as positive error ---------
  {
    const rescue::LineReading rightEdge = rescue::analyze(blackPattern(RIGHT_EDGE_CHANNELS));
    CHECK(rightEdge.error > 0, "line under the rightmost sensor reads as positive error");
    CHECK_EQ(rightEdge.position, 7000);

    const rescue::LineReading leftEdge = rescue::analyze(blackPattern(LEFT_EDGE_CHANNELS));
    CHECK(leftEdge.error < 0, "line under the leftmost sensor reads as negative error");
    CHECK_EQ(leftEdge.position, 0);
  }

  // ---- calibration weighting -------------------------------------------
  // Channels 3 and 5 are lit, sitting at positions 3000 and 5000, so a
  // healthy bar balances at 4000. Bleeding the contrast out of channel 5 must
  // lower its weight and drag the centroid back toward the healthy channel.
  {
    const int32_t balanced = rescue::analyze(blackPattern(OFF_CENTRE_PAIR)).position;
    CHECK_EQ(balanced, 4000);

    rescue::LineCalibration weakChannel = fullContrastCalibration();
    weakChannel.channel[5].black = 100;  // only 100 counts of contrast left
    weakChannel.channel[5].white = WHITE_READING;
    rescue::setCalibration(weakChannel);

    CHECK(rescue::channelWeight(5, BLACK_READING) < rescue::LINE_WEIGHT_MAX,
          "a weak-contrast channel carries less than full weight");
    CHECK_EQ(rescue::channelWeight(3, BLACK_READING), rescue::LINE_WEIGHT_MAX);

    const int32_t weakened = rescue::analyze(blackPattern(OFF_CENTRE_PAIR)).position;
    CHECK(weakened < balanced, "weakening a channel moves the centroid off its balance point");
    CHECK(weakened < 4000, "the weakened bar leans toward the healthy channel");
    CHECK(weakened > 3000, "the weakened bar still leans right of the healthy channel");
  }

  // A channel with no contrast at all is blind: zero weight, never counted.
  {
    rescue::LineCalibration deadChannel = fullContrastCalibration();
    deadChannel.channel[5].black = BLACK_READING;
    deadChannel.channel[5].white = BLACK_READING;
    rescue::setCalibration(deadChannel);

    CHECK_EQ(rescue::channelWeight(5, BLACK_READING), 0);
    CHECK_EQ(rescue::analyze(blackPattern(OFF_CENTRE_PAIR)).position, 3000);
  }

  // ---- polarity comes from the calibration, not an assumption -----------
  // The same physical bar described with its black/white readings swapped and
  // calibrated the swapped way round must normalise identically: nothing in
  // the logic knows which way the sensor reads.
  {
    const rescue::LineVector sameBarInverted =
        patternOn(CENTRED_PAIR, WHITE_READING, BLACK_READING);
    rescue::setCalibration(rescue::defaultCalibration(WHITE_READING, BLACK_READING));
    CHECK_EQ(rescue::analyze(sameBarInverted).position, 3500);
    CHECK_EQ(rescue::channelWeight(3, WHITE_READING), rescue::LINE_WEIGHT_MAX);
  }

  // A reading past the calibrated black end saturates rather than wrapping
  // back around to no line signal.
  {
    rescue::setCalibration(fullContrastCalibration());
    CHECK_EQ(rescue::channelWeight(0, 2046), rescue::LINE_WEIGHT_MAX);
    CHECK_EQ(rescue::channelWeight(0, WHITE_READING), 0);
  }

  // ---- detection -------------------------------------------------------
  {
    rescue::setCalibration(fullContrastCalibration());
    CHECK(rescue::analyze(blackPattern(0)).lineDetected == false,
          "an all-white bar reports no line");
    CHECK(rescue::analyze(blackPattern(EVERY_CHANNEL)).lineDetected == true,
          "a fully black bar reports the line");
    CHECK(rescue::analyze(blackPattern(CENTRED_PAIR)).lineDetected == true,
          "a centred line reports the line");
  }

  // ---- the four coarse classes ------------------------------------------
  // These name the situation for the mission state machine. They never steer.
  rescue::setCalibration(fullContrastCalibration());
  CHECK(rescue::analyze(blackPattern(0)).qualifier == rescue::LineClass::ALL_WHITE,
        "no channel on the line is all-white");
  CHECK(rescue::analyze(blackPattern(EVERY_CHANNEL)).qualifier == rescue::LineClass::ALL_DARK,
        "every channel on the line is all-dark");
  CHECK(rescue::analyze(blackPattern(CENTRED_PAIR)).qualifier == rescue::LineClass::ON_LINE,
        "a centred line is on-line");
  CHECK(rescue::analyze(blackPattern(RIGHT_EDGE_CHANNELS)).qualifier == rescue::LineClass::EDGE,
        "a line under the rightmost sensor is an edge");
  CHECK(rescue::analyze(blackPattern(LEFT_EDGE_CHANNELS)).qualifier == rescue::LineClass::EDGE,
        "a line under the leftmost sensor is an edge");

  // A line running off under the two rightmost channels is still an edge.
  {
    constexpr uint8_t RIGHT_QUAD = 0xC0;  // channels 6 and 7
    CHECK(rescue::analyze(blackPattern(RIGHT_QUAD)).qualifier == rescue::LineClass::EDGE,
          "a line running off under the two rightmost channels is an edge");
  }

  // ---- the class is a qualifier, never the control signal ---------------
  // Two bars in the same class still produce different errors, so the
  // steering signal rides the centroid and not the class.
  {
    constexpr uint8_t LEFT_OF_CENTRE = 0x08;  // channel 3 alone
    constexpr uint8_t RIGHT_OF_CENTRE = 0x10;  // channel 4 alone
    const rescue::LineReading left = rescue::analyze(blackPattern(LEFT_OF_CENTRE));
    const rescue::LineReading right = rescue::analyze(blackPattern(RIGHT_OF_CENTRE));
    CHECK(left.qualifier == right.qualifier,
          "a line either side of centre shares one class");
    CHECK(left.error < 0, "a line left of centre is negative error");
    CHECK(right.error > 0, "a line right of centre is positive error");
    CHECK(left.error != right.error,
          "one class spans a continuum of errors, so the class cannot be the signal");
  }

  // A fully dark bar still reports a real centroid rather than a canned value.
  {
    const rescue::LineReading dark = rescue::analyze(blackPattern(EVERY_CHANNEL));
    CHECK_EQ(dark.position, 3500);
    CHECK_EQ(dark.error, 0);
  }

  // ---- steering: positive error must demand a faster left wheel ---------
  CHECK(rescue::LINE_KP > 0.0, "the proportional gain is positive");
  CHECK(rescue::correctionFor(3500) > 0, "a line to the right demands positive correction");
  CHECK(rescue::correctionFor(-3500) < 0, "a line to the left demands negative correction");
  CHECK_EQ(rescue::correctionFor(0), 0);

  {
    const rescue::WheelBias lineOnRight = rescue::steerFor(3500);
    CHECK(lineOnRight.left > 0, "line on the right speeds the left wheel up");
    CHECK(lineOnRight.right < 0, "line on the right slows the right wheel down");

    const rescue::WheelBias lineOnLeft = rescue::steerFor(-3500);
    CHECK(lineOnLeft.left < 0, "line on the left slows the left wheel down");
    CHECK(lineOnLeft.right > 0, "line on the left speeds the right wheel up");

    CHECK_EQ(lineOnRight.left, -lineOnRight.right);
  }

  // End to end: a reading taken off the right edge steers left.
  {
    const rescue::LineReading offRight =
        rescue::analyze(blackPattern(RIGHT_EDGE_CHANNELS));
    CHECK(rescue::steerFor(offRight.error).left > 0,
          "an off-right reading turns the robot left, back onto the line");
  }

  // ---- the derivative term stays switched off ---------------------------
  CHECK(rescue::LINE_KD == 0.0, "the derivative gain is disabled");
  CHECK_EQ(rescue::pdCorrection(1000, 0), rescue::pdCorrection(1000, 1000));
  CHECK_EQ(rescue::pdCorrection(1000, -1000), rescue::correctionFor(1000));

  // ---- correction is capped ---------------------------------------------
  CHECK_EQ(rescue::correctionFor(ERROR_BEYOND_CAP), rescue::LINE_MAX_CORRECTION);
  CHECK_EQ(rescue::correctionFor(-ERROR_BEYOND_CAP), -rescue::LINE_MAX_CORRECTION);
  CHECK(rescue::correctionFor(ERROR_WITH_HEADROOM) < rescue::LINE_MAX_CORRECTION,
        "a moderate error leaves headroom under the cap");

  // ---- the intermediate seams the diagnostic dump reads ---------------
  rescue::setCalibration(fullContrastCalibration());
  {
    const rescue::LineWeights weights = rescue::normalize(blackPattern(CENTRED_PAIR));
    CHECK_EQ(weights.weight[3], rescue::LINE_WEIGHT_MAX);
    CHECK_EQ(weights.weight[4], rescue::LINE_WEIGHT_MAX);
    CHECK_EQ(weights.weight[2], rescue::LINE_WEIGHT_MIN);
    CHECK_EQ(weights.weight[7], rescue::LINE_WEIGHT_MIN);
  }
  {
    const rescue::LineWeights weights = rescue::normalize(blackPattern(OFF_CENTRE_PAIR));
    CHECK_EQ(rescue::centroid(weights), 4000);
    CHECK(rescue::classify(weights, 4000) == rescue::LineClass::ON_LINE,
          "a centred pair of channels is on-line");

    // All-white weights carry no line at all, and no centroid either.
    const rescue::LineWeights nothing = rescue::normalize(blackPattern(0));
    CHECK_EQ(rescue::centroid(nothing), rescue::LINE_POSITION_CENTER);
    CHECK(rescue::classify(nothing, rescue::LINE_POSITION_CENTER) ==
              rescue::LineClass::ALL_WHITE,
          "no signal at all is all-white");

    const rescue::LineWeights allDark = rescue::normalize(blackPattern(EVERY_CHANNEL));
    CHECK(rescue::classify(allDark, 3500) == rescue::LineClass::ALL_DARK,
          "every channel lit is all-dark");

    const rescue::LineWeights offRight = rescue::normalize(blackPattern(RIGHT_EDGE_CHANNELS));
    CHECK(rescue::classify(offRight, 7000) == rescue::LineClass::EDGE,
          "a line at the far end is an edge");
  }

  return DF_SUMMARIZE("line_test");
}
