// markers_test.cpp — host-side tests for src/rescue_robot/markers.h (pure C++14).
// Run: ./test/run_tests.sh
//   or: g++ -std=c++14 -Itest -Isrc/rescue_robot test/markers_test.cpp -o /tmp/mtest && /tmp/mtest

#include "dryframe.h"
#include "markers.h"

#include <string>
#include <vector>

using namespace rescue;

namespace {

// ---------------------------------------------------------------------------
// Mux spy: records every select / read / deselect the logic drives.
// ---------------------------------------------------------------------------

enum class BusOp { Select, Read, Deselect };

const char* opName(BusOp op) {
  switch (op) {
    case BusOp::Select: return "select";
    case BusOp::Read: return "read";
    case BusOp::Deselect: return "deselect";
  }
  return "?";
}

struct SpyEvent {
  BusOp op;
  uint8_t channel;
};

struct SpyBus {
  std::vector<SpyEvent> events;
  uint16_t leftGreen = 0;
  uint16_t rightGreen = 0;
  uint8_t selectedChannel = 0;
  bool readsFail = false;
  uint32_t sliceCostUs = 100;  // charged once per micros() call
  uint32_t clockUs = 0;
  uint32_t millisValue = 0;
  uint32_t readCount = 0;
};

SpyBus gSpy;

void spySelect(uint8_t channel, void* ctx) {
  SpyBus& spy = *static_cast<SpyBus*>(ctx);
  spy.selectedChannel = channel;
  spy.events.push_back(SpyEvent{BusOp::Select, channel});
}

bool spyReadGreen(uint16_t* green, void* ctx) {
  SpyBus& spy = *static_cast<SpyBus*>(ctx);
  spy.events.push_back(SpyEvent{BusOp::Read, spy.selectedChannel});
  ++spy.readCount;
  if (spy.readsFail) return false;
  *green = (spy.selectedChannel == LEFT_MUX_CHANNEL) ? spy.leftGreen
                                                     : spy.rightGreen;
  return true;
}

void spyDeselect(void* ctx) {
  SpyBus& spy = *static_cast<SpyBus*>(ctx);
  spy.events.push_back(SpyEvent{BusOp::Deselect, spy.selectedChannel});
}

uint32_t spyMicros() {
  gSpy.clockUs += gSpy.sliceCostUs;
  return gSpy.clockUs;
}

uint32_t spyMillis() { return gSpy.millisValue; }

MarkerBus makeBus() {
  return MarkerBus{spySelect, spyReadGreen, spyDeselect, spyMicros, &gSpy};
}

std::string eventTrace() {
  std::string trace;
  for (const SpyEvent& ev : gSpy.events) {
    if (!trace.empty()) trace += ",";
    trace += opName(ev.op);
  }
  return trace;
}

}  // namespace

int main() {
  // -------------------------------------------------------------------------
  // Green test: active-low against the calibrated white baseline.
  // -------------------------------------------------------------------------
  CHECK(!isGreen(0, 0), "uncalibrated baseline never reports green");
  const uint16_t white = 1000;
  const uint16_t boundary =
      static_cast<uint16_t>(white * (100u - MARKER_GREEN_DROP_PERCENT) / 100u);
  CHECK(isGreen(boundary, white), "reading exactly at the drop boundary is green");
  CHECK(!isGreen(static_cast<uint16_t>(boundary + 1), white),
        "one count above the drop boundary is not green");
  CHECK(isGreen(10, white), "strongly attenuated channel is green");
  CHECK(!isGreen(990, white), "near-white reading is not green");

  // -------------------------------------------------------------------------
  // NONE: nothing green on either side.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 59500;
    gSpy.rightGreen = 60200;
    for (int i = 0; i < 3; ++i) {
      const MarkerPoll poll = pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL,
                                          RIGHT_MUX_CHANNEL);
      CHECK(poll.state == MarkerState::None, "near-white both sides reads NONE");
      CHECK(poll.leftOk && poll.rightOk, "both side reads succeeded");
    }
  }

  // -------------------------------------------------------------------------
  // LEFT / RIGHT: green on one side only.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 6000;
    gSpy.rightGreen = 59000;
    MarkerPoll poll = pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL,
                                  RIGHT_MUX_CHANNEL);
    CHECK(poll.state == MarkerState::Left, "left green, right white -> LEFT");
    CHECK(!logic.deadEndConfirmed(), "single-side green is not a dead end");
    CHECK(logic.pendingBothSamples() == 0, "single-side green arms no confirm");

    gSpy.leftGreen = 59000;
    gSpy.rightGreen = 6500;
    poll = pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(poll.state == MarkerState::Right, "right green, left white -> RIGHT");
  }

  // -------------------------------------------------------------------------
  // BOTH: dead end only after two agreeing sample pairs.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 5000;
    gSpy.rightGreen = 5500;

    const MarkerPoll first =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(first.state == MarkerState::None, "first BOTH sample is not published");
    CHECK(!logic.deadEndConfirmed(), "first BOTH sample is not a confirmed dead end");
    CHECK(logic.pendingBothSamples() == 1, "first BOTH sample is held pending");

    const MarkerPoll second =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(second.state == MarkerState::Both, "second agreeing BOTH sample confirms");
    CHECK(logic.deadEndConfirmed(), "confirmed dead end flag raised");
    CHECK(logic.pendingBothSamples() == 0, "confirm counter cleared after publish");
    CHECK(std::string(markerStateName(MarkerState::Both)) == "both",
          "state name renders");
  }

  // -------------------------------------------------------------------------
  // One BOTH sample then disagreement -> never confirmed.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 5000;
    gSpy.rightGreen = 5500;
    const MarkerPoll first =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(first.state == MarkerState::None, "transient BOTH stays unpublished");

    gSpy.rightGreen = 59500;  // right side went white
    const MarkerPoll second =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(second.state == MarkerState::Left, "disagreement degrades to LEFT");
    CHECK(!logic.deadEndConfirmed(), "disagreement is never a confirmed dead end");
    CHECK(logic.pendingBothSamples() == 0, "disagreement clears the pending confirm");

    gSpy.leftGreen = 5000;    // both green again -> needs a fresh pair
    gSpy.rightGreen = 5500;
    const MarkerPoll third =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(third.state == MarkerState::None, "restarted confirm needs two samples");
    const MarkerPoll fourth =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(fourth.state == MarkerState::Both, "second fresh pair confirms again");
  }

  // -------------------------------------------------------------------------
  // Blind time: a confirmed dead end is held, then re-armed.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 5000;
    gSpy.rightGreen = 5500;
    pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(logic.deadEndConfirmed(), "baseline confirm before the blind-time check");

    gSpy.millisValue = MARKER_DEADEND_BLIND_MS / 2;
    const MarkerPoll insideBlind =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(insideBlind.state == MarkerState::None, "inside blind time the marker is held");
    CHECK(logic.pendingBothSamples() == 0, "blind hold clears pending confirms");

    gSpy.millisValue = MARKER_DEADEND_BLIND_MS + 1;
    const MarkerPoll afterBlind =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(afterBlind.state == MarkerState::None, "re-armed first sample is unpublished");
    const MarkerPoll afterBlindSecond =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(afterBlindSecond.state == MarkerState::Both,
          "re-armed marker confirms again after the blind window");

    logic.reset();
    CHECK(logic.pendingBothSamples() == 0 && !logic.deadEndConfirmed(),
          "reset clears confirm state");
  }

  // -------------------------------------------------------------------------
  // Mux discipline: select -> read -> deselect per side, deselect always last.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    gSpy.sliceCostUs = 200;
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    gSpy.leftGreen = 30000;
    gSpy.rightGreen = 30000;

    const MarkerPoll poll =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(poll.state == MarkerState::None,
          "30000-of-60000 on both sides is green but still unconfirmed");
    CHECK(eventTrace() == "select,read,deselect,select,read,deselect",
          "one poll drives select/read/deselect per side, in channel order");

    bool deselectAlwaysFollowsRead = true;
    for (size_t i = 0; i + 1 < gSpy.events.size(); ++i) {
      if (gSpy.events[i].op == BusOp::Read) {
        deselectAlwaysFollowsRead =
            deselectAlwaysFollowsRead && gSpy.events[i + 1].op == BusOp::Deselect;
      }
    }
    CHECK(deselectAlwaysFollowsRead, "every read is followed by a deselect");
    CHECK(gSpy.events.back().op == BusOp::Deselect, "poll ends on a deselect");
    CHECK(gSpy.readCount == 2, "one read per side per poll");

    // CH0 carries the left sensor, CH1 the right sensor.
    CHECK(gSpy.events[0].channel == LEFT_MUX_CHANNEL, "left side uses mux CH0");
    CHECK(gSpy.events[3].channel == RIGHT_MUX_CHANNEL, "right side uses mux CH1");
  }

  // -------------------------------------------------------------------------
  // Failed read is treated as "no green" on that side.
  // -------------------------------------------------------------------------
  {
    gSpy = SpyBus();
    gSpy.readsFail = true;
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    const MarkerPoll poll =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(!poll.leftOk && !poll.rightOk, "failed reads are reported");
    CHECK(poll.state == MarkerState::None, "failed reads never read as green");
  }

  // -------------------------------------------------------------------------
  // Per-sample budget: each select/read/deselect slice stays inside 1 ms.
  // -------------------------------------------------------------------------
  {
    MarkerSampleTiming slice;
    slice.selectUs = 200;
    slice.readUs = 500;
    slice.deselectUs = 250;
    CHECK(slice.totalUs() == 950, "slice total sums its three phases");
    CHECK(slice.withinBudget(), "950 us slice is inside the 1 ms budget");

    MarkerSampleTiming overrun;
    overrun.selectUs = 400;
    overrun.readUs = 400;
    overrun.deselectUs = 400;
    CHECK(!overrun.withinBudget(), "1200 us slice breaks the 1 ms budget");
    CHECK(MARKER_SAMPLE_BUDGET_US == 1000, "budget constant is 1 ms");

    gSpy = SpyBus();
    gSpy.sliceCostUs = 100;  // 3 micros phases * 100 us = 300 us per side
    MarkerLogic logic(spyMillis);
    logic.setWhiteBaseline(60000);
    const MarkerPoll poll =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(poll.leftTiming.totalUs() == 300, "left slice timing measured from the bus");
    CHECK(poll.rightTiming.totalUs() == 300, "right slice timing measured from the bus");
    CHECK(poll.withinBudget(), "both measured slices are inside the budget");

    gSpy.sliceCostUs = 400;  // 3 * 400 = 1200 us per side -> over budget
    const MarkerPoll slow =
        pollMarkers(makeBus(), logic, LEFT_MUX_CHANNEL, RIGHT_MUX_CHANNEL);
    CHECK(!slow.withinBudget(), "over-budget slice is flagged by the model");
  }

  // -------------------------------------------------------------------------
  // Named constants the delivery layer relies on.
  // -------------------------------------------------------------------------
  CHECK(MARKER_CONFIRM_SAMPLES == 2, "dead end needs two agreeing samples");
  CHECK(MARKER_BUS_HZ == 400000, "I2C bus runs at 400 kHz");
  CHECK(LEFT_MUX_CHANNEL == 0x01 && RIGHT_MUX_CHANNEL == 0x02,
        "left is mux CH0, right is mux CH1");

  return DF_SUMMARIZE("markers");
}
