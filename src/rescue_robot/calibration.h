// calibration.h — pure host-compilable venue sweep logic (C++14).
//
// The 10 s sweep is the only source of sensory truth for the sensor bar: held
// with the bar over the black line for the first half of the window and over the
// white floor for the second, each channel's mean reading in each half becomes
// its black and white levels. Their midpoint is that channel's threshold and
// their distance is its contrast, so an inverted bar calibrates itself and no
// polarity is assumed anywhere.
//
// The contrast-health gate is what carries the locked 11.2 mm ride height, where
// the black-to-white spread collapses: a channel that saw too little spread is
// rejected outright, its bit recorded, and the healthy channels' weights
// redistributed so the bar keeps a full share. Too few healthy channels and the
// sweep is refused — the run then retries twice and ends in one of three named
// decisions: run on the last-good basis from the store (the declared
// degradation carve-out), run degraded on this sweep, or refuse to start.
//
// No AVR identifiers live here. calibration.ino is the only delivery layer and
// owns the pins, the clock, the console dump and the store commit.

#ifndef RESCUE_ROBOT_CALIBRATION_H
#define RESCUE_ROBOT_CALIBRATION_H

#include <cstdint>

// Two of the modules this one feeds declare a per-channel struct of the same
// name: the line layer's {black, white} and the store's {black_level,
// white_level, weight}. Both shapes are needed here, and both now carry names of
// their own, so these two headers are included plainly and in any order — the
// line module's is ChannelCalibration and the store's is
// StoredChannelCalibration.
#include "eeprom_store.h"
#include "line.h"

namespace rescue {

// The window is timed against the game clock, so its shape is fixed. Samples
// land on a fixed cadence to keep the totals' arithmetic bounded however fast
// the loop runs.
constexpr uint32_t CAL_SWEEP_WINDOW_MS = 10000;
constexpr uint32_t CAL_SWEEP_HALF_MS = CAL_SWEEP_WINDOW_MS / 2;
constexpr uint32_t CAL_SAMPLE_INTERVAL_MS = 2;
constexpr uint8_t CAL_CHANNEL_COUNT = QTR_CHANNEL_COUNT;
constexpr int32_t CAL_ANALOG_MIN = LINE_ANALOG_MIN;
constexpr int32_t CAL_ANALOG_MAX = LINE_ANALOG_MAX;

// The gate borrows the line layer's trust ramp: at or above the floor a channel
// is healthy, below it the channel is blind and delivers nothing.
constexpr int32_t CAL_MIN_CHANNEL_CONTRAST = LINE_MIN_CONTRAST;
constexpr uint8_t CAL_MIN_HEALTHY_CHANNELS = 5;

// The degraded branch needs a sweep that still saw something worth reading,
// though nowhere near a venue-grade basis.
constexpr uint8_t CAL_MIN_DEGRADED_CHANNELS = 2;

// --- Normalised weights ----------------------------------------------------
// The healthy channels share CAL_WEIGHT_TOTAL in proportion to their contrast,
// so a rejected channel's share is redistributed rather than lost and a channel
// that saw more spread is trusted more. Rejected channels carry no weight.
constexpr uint16_t CAL_WEIGHT_TOTAL = 1000;
constexpr uint16_t CAL_REJECTED_CHANNEL_WEIGHT = 0;

// The venue sweep plus two retries.
constexpr uint8_t CAL_MAX_ATTEMPTS = 3;
constexpr uint8_t CAL_RETRY_COUNT = CAL_MAX_ATTEMPTS - 1;

static_assert(CAL_WEIGHT_TOTAL == LINE_WEIGHT_MAX,
              "the stored share and the line scale must be the same number");
static_assert(CAL_MIN_HEALTHY_CHANNELS <= CAL_CHANNEL_COUNT,
              "the healthy floor cannot exceed the bar");
static_assert(CAL_RETRY_COUNT == 2, "the venue allows two retries");

// One raw reading per channel, leftmost first, for the phase being swept.
struct CalRawVector {
  int32_t reading[CAL_CHANNEL_COUNT];
};

// Running totals for one attempt; nothing is derived until the window closes.
struct SweepAccumulator {
  uint32_t black_sum[CAL_CHANNEL_COUNT];
  uint32_t white_sum[CAL_CHANNEL_COUNT];
  uint16_t black_count;
  uint16_t white_count;

  bool complete() const { return black_count > 0 && white_count > 0; }
};

// The half of the window the bar is being held over at `elapsedMs`.
enum class SweepPhase : uint8_t { Black = 0, White = 1 };

inline SweepPhase sweepPhaseAt(uint32_t elapsedMs) {
  return elapsedMs < CAL_SWEEP_HALF_MS ? SweepPhase::Black : SweepPhase::White;
}

inline bool sweepWindowElapsed(uint32_t elapsedMs) {
  return elapsedMs >= CAL_SWEEP_WINDOW_MS;
}

struct SweepChannel {
  uint16_t black_level = 0;
  uint16_t white_level = 0;
  uint16_t weight = CAL_REJECTED_CHANNEL_WEIGHT;
  bool healthy = false;

  // The black-to-white distance, direction ignored.
  int32_t contrast() const {
    const int32_t span =
        static_cast<int32_t>(white_level) - static_cast<int32_t>(black_level);
    return span < 0 ? -span : span;
  }

  // The channel's own threshold, halfway between the two levels.
  int32_t midpoint() const {
    return (static_cast<int32_t>(black_level) +
            static_cast<int32_t>(white_level)) /
           2;
  }
};

// What one attempt produced. The mask and the verdict are the basis a later run
// reasons about, which is why they travel with the readings.
struct SweepResult {
  SweepChannel channel[CAL_CHANNEL_COUNT];
  uint8_t healthy_count = 0;
  uint16_t weak_channel_mask = 0;
  ContrastVerdict contrast = ContrastVerdict::Unknown;
  bool accepted = false;
};

// Keep a reading inside the converter's span, so a value from outside it cannot
// corrupt the totals that value is added to.
inline int32_t clampReading(int32_t reading) {
  if (reading < CAL_ANALOG_MIN) return CAL_ANALOG_MIN;
  if (reading > CAL_ANALOG_MAX) return CAL_ANALOG_MAX;
  return reading;
}

inline void sweepReset(SweepAccumulator& accumulator) {
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    accumulator.black_sum[channel] = 0;
    accumulator.white_sum[channel] = 0;
  }
  accumulator.black_count = 0;
  accumulator.white_count = 0;
}

// Fold one sample of the phase currently being swept into the totals.
inline void sweepAccumulate(SweepAccumulator& accumulator, SweepPhase phase,
                            const CalRawVector& raw) {
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    const uint32_t reading =
        static_cast<uint32_t>(clampReading(raw.reading[channel]));
    if (phase == SweepPhase::Black) {
      accumulator.black_sum[channel] += reading;
    } else {
      accumulator.white_sum[channel] += reading;
    }
  }
  if (phase == SweepPhase::Black) {
    ++accumulator.black_count;
  } else {
    ++accumulator.white_count;
  }
}

// The mean level a channel saw across one phase; no samples means no level.
inline uint16_t sweepMeanLevel(uint32_t sum, uint16_t count) {
  if (count == 0) return 0;
  return static_cast<uint16_t>(sum / count);
}

inline bool sweepChannelHealthy(int32_t contrast) {
  return contrast >= CAL_MIN_CHANNEL_CONTRAST;
}

inline uint16_t channelBit(uint8_t channel) {
  return static_cast<uint16_t>(1u << channel);
}

namespace detail {

// The healthy channels' contrast total, and with it the channel that saw the
// most — the one that absorbs the division remainder.
struct WeightBasis {
  uint32_t contrast_total = 0;
  uint8_t strongest = 0;
};

inline WeightBasis weightBasis(const SweepResult& result) {
  WeightBasis basis;
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    if (!result.channel[channel].healthy) continue;
    const int32_t contrast = result.channel[channel].contrast();
    if (contrast > result.channel[basis.strongest].contrast()) {
      basis.strongest = channel;
    }
    basis.contrast_total += static_cast<uint32_t>(contrast);
  }
  return basis;
}

}  // namespace detail

// Hand CAL_WEIGHT_TOTAL to the healthy channels in proportion to their
// contrast, rejected channels taking none. Integer division leaves a remainder
// and the strongest channel absorbs it, so the weights sum to the full share.
inline void normaliseWeights(SweepResult& result) {
  const detail::WeightBasis basis = detail::weightBasis(result);
  if (basis.contrast_total == 0) return;
  uint32_t assigned = 0;
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    if (!result.channel[channel].healthy) {
      result.channel[channel].weight = CAL_REJECTED_CHANNEL_WEIGHT;
      continue;
    }
    const uint32_t share = static_cast<uint32_t>(CAL_WEIGHT_TOTAL) *
                           static_cast<uint32_t>(result.channel[channel].contrast()) /
                           basis.contrast_total;
    result.channel[channel].weight = static_cast<uint16_t>(share);
    assigned += share;
  }
  result.channel[basis.strongest].weight = static_cast<uint16_t>(
      result.channel[basis.strongest].weight + (CAL_WEIGHT_TOTAL - assigned));
}

// Turn a closed window into levels, midpoints, weights and the health verdict.
// All arithmetic is 32-bit: the AVR's native int is 16 bits and the sample
// totals overflow it.
inline SweepResult sweepEvaluate(const SweepAccumulator& accumulator) {
  SweepResult result;
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    SweepChannel& entry = result.channel[channel];
    entry.black_level =
        sweepMeanLevel(accumulator.black_sum[channel], accumulator.black_count);
    entry.white_level =
        sweepMeanLevel(accumulator.white_sum[channel], accumulator.white_count);
    entry.healthy = sweepChannelHealthy(entry.contrast());
    if (entry.healthy) {
      ++result.healthy_count;
    } else {
      result.weak_channel_mask |= channelBit(channel);
    }
  }
  normaliseWeights(result);
  if (result.healthy_count == CAL_CHANNEL_COUNT) {
    result.contrast = ContrastVerdict::Healthy;
  } else if (result.healthy_count > 0) {
    result.contrast = ContrastVerdict::Weak;
  }
  result.accepted = result.healthy_count >= CAL_MIN_HEALTHY_CHANNELS;
  return result;
}

// The sweep produces exactly the two shapes its consumers take.

inline LineCalibration lineCalibrationFrom(const SweepResult& result) {
  LineCalibration calibration{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    calibration.channel[channel].black = result.channel[channel].black_level;
    calibration.channel[channel].white = result.channel[channel].white_level;
  }
  return calibration;
}

inline LineCalibration lineCalibrationFrom(const CalibrationRecord& record) {
  LineCalibration calibration{};
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    calibration.channel[channel].black = record.channels[channel].black_level;
    calibration.channel[channel].white = record.channels[channel].white_level;
  }
  return calibration;
}

// The green baseline belongs to the marker layer, so the store's defaults stay
// in place rather than being stamped with zeros; a later marker save fills it in
// and re-commits the record.
inline CalibrationRecord storedRecordFrom(const SweepResult& result) {
  CalibrationRecord record = freshCalibration();
  for (uint8_t channel = 0; channel < CAL_CHANNEL_COUNT; ++channel) {
    record.channels[channel].black_level = result.channel[channel].black_level;
    record.channels[channel].white_level = result.channel[channel].white_level;
    record.channels[channel].weight = result.channel[channel].weight;
  }
  record.contrast = result.contrast;
  record.weak_channel_mask = result.weak_channel_mask;
  return record;
}

// What the venue has decided to do when the gate keeps refusing.
enum class CalFallback : uint8_t {
  LastGoodThenDegrade = 0,
  AbortRun = 1,
};

enum class CalDecision : uint8_t {
  Pending = 0,            // sweeping, or a retry is owed
  Persisted = 1,          // accepted sweep, installed and written as last-good
  LastGoodPersisted = 2,  // retries spent: the stored venue basis is in force
  Degraded = 3,           // retries spent, nothing stored: this sweep, logged
  Abort = 4,              // retries spent with no basis at all: do not start
};

struct CalVenuePolicy {
  CalFallback fallback = CalFallback::LastGoodThenDegrade;
  bool last_good_stored = false;
};

// One run's standing: attempts spent, what the last sweep found, and the
// decision that leaves open.
struct CalAttempt {
  uint8_t attempt = 1;  // 1-based; the first sweep is attempt one
  uint8_t healthy_count = 0;
  uint16_t weak_channel_mask = 0;
  CalDecision decision = CalDecision::Pending;
};

namespace detail {

// The terminal branch, reached once the retry allowance is spent.
inline CalDecision exhaustedDecision(const SweepResult& result,
                                     const CalVenuePolicy& policy) {
  if (policy.fallback == CalFallback::AbortRun) return CalDecision::Abort;
  if (policy.last_good_stored) return CalDecision::LastGoodPersisted;
  if (result.healthy_count >= CAL_MIN_DEGRADED_CHANNELS) {
    return CalDecision::Degraded;
  }
  return CalDecision::Abort;
}

}  // namespace detail

inline void calRunBegin(CalAttempt& run) {
  run.attempt = 1;
  run.healthy_count = 0;
  run.weak_channel_mask = 0;
  run.decision = CalDecision::Pending;
}

// Record what a closed sweep found and settle the run's decision: accept it,
// owe a retry, or with the allowance spent fall through to venue policy.
inline void calConcludeAttempt(CalAttempt& run, const SweepResult& result,
                               const CalVenuePolicy& policy) {
  run.healthy_count = result.healthy_count;
  run.weak_channel_mask = result.weak_channel_mask;
  if (result.accepted) {
    run.decision = CalDecision::Persisted;
  } else if (run.attempt < CAL_MAX_ATTEMPTS) {
    ++run.attempt;
    run.decision = CalDecision::Pending;
  } else {
    run.decision = detail::exhaustedDecision(result, policy);
  }
}

// True only when the run owes another sweep.
inline bool calRetryOwed(const CalAttempt& run) {
  return run.decision == CalDecision::Pending && run.attempt > 1;
}

inline bool calDecisionAcceptsRun(CalDecision decision) {
  return decision == CalDecision::Persisted ||
         decision == CalDecision::LastGoodPersisted ||
         decision == CalDecision::Degraded;
}

inline const char* calDecisionName(CalDecision decision) {
  switch (decision) {
    case CalDecision::Pending: return "pending";
    case CalDecision::Persisted: return "persisted";
    case CalDecision::LastGoodPersisted: return "last-good-persisted";
    case CalDecision::Degraded: return "degraded";
    case CalDecision::Abort: return "abort";
  }
  return "unknown";
}

}  // namespace rescue

#endif  // RESCUE_ROBOT_CALIBRATION_H
