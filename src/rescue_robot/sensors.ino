// sensors.ino — AVR delivery layer for the front bumper plate and the victim
// probe pair.
//
// Owns every Arduino-side touch on behalf of sensors.h: pin modes and the
// millis()-driven sample calls. Contains zero decision logic — debouncing,
// latching, and the live/dead/fake truth table all live in sensors.h.
//
// Bumper wiring: the two soft push buttons on the front plate are the
// never-fail mechanical backstop. Each button is one leg tied to GND, the
// other to its GPIO, which is held HIGH by the internal pull-up. A press
// therefore reads LOW. Nothing else drives the plate, so contact chatter is
// the only noise the logic has to reject.
//
// Probe wiring: the reflectivity probe is an analog output (higher = more
// reflective); the conductivity probe is a digital output driven through a
// pull-down, so conduction reads HIGH. Both probes sit at the front of the
// victim cup and are read in the same call.

#include <Arduino.h>

#include "sensors.h"

using namespace rescue;

namespace {

// ---------------------------------------------------------------------------
// Bumper plate pins (front plate, LEFT and RIGHT zones)
// ---------------------------------------------------------------------------
constexpr uint8_t BUMPER_LEFT_BUTTON_PIN = 8;   // D8  <- LEFT soft button
constexpr uint8_t BUMPER_RIGHT_BUTTON_PIN = 9;  // D9  <- RIGHT soft button

// ---------------------------------------------------------------------------
// Victim probe pins (front of the cup)
// ---------------------------------------------------------------------------
// A1 is analog input 1; the odd analog channels are free (QTR-8A takes the
// even ones) and the Mega maps A1 to the A-pin's first comparator channel.
constexpr uint8_t REFLECTIVITY_PROBE_PIN = A1;   // analog: brighter = more reflective
constexpr uint8_t CONDUCTIVITY_PROBE_PIN = 22;  // D22 <- conductive HIGH

// The live plate and probe verdicts, owned by this delivery layer.
BumperPlate bumperPlate;
ProbeClassifier probeClassifier;

}  // namespace

// Configure both bumper buttons as pull-ups and the probe inputs. Call once
// from setup() before the mission loop starts.
void sensorsBegin() {
  pinMode(BUMPER_LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUMPER_RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(REFLECTIVITY_PROBE_PIN, INPUT);
  pinMode(CONDUCTIVITY_PROBE_PIN, INPUT_PULLDOWN);
}

// Sample the plate. Call once per loop() iteration; it debounces on millis()
// and returns immediately, never waiting.
void updateBumper() {
  const bool leftPressed = digitalRead(BUMPER_LEFT_BUTTON_PIN) == LOW;
  const bool rightPressed = digitalRead(BUMPER_RIGHT_BUTTON_PIN) == LOW;
  bumperPlate.update(leftPressed, rightPressed, millis());
}

// Read a latched strike on one zone, clearing it. Returns false if that side
// has not been struck since the last read.
bool consumeBumperEvent(BumperZone zone) { return bumperPlate.consumeEvent(zone); }

// Read whichever zone was struck first, clearing that one. False if neither
// side is latched.
bool consumeAnyBumperEvent(BumperZone& zone) {
  return bumperPlate.consumeAnyZone(zone);
}

// Debounced level of one zone right now (true = held down).
bool bumperPressed(BumperZone zone) { return bumperPlate.isPressed(zone); }

// Forget plate history and latched strikes.
void resetBumper() { bumperPlate.reset(); }

// Read both probes once and classify the surface under the cup. The verdict
// is Live, Dead, or Fake — the caller ignores Fake and never deposits it.
VictimClass classifyVictim() {
  const int reflectivity = analogRead(REFLECTIVITY_PROBE_PIN);
  const bool conductive = digitalRead(CONDUCTIVITY_PROBE_PIN) == HIGH;
  return probeClassifier.classify(reflectivity, conductive);
}

// The raw reflectivity count, for the serial diagnostic menu.
int readReflectivity() { return analogRead(REFLECTIVITY_PROBE_PIN); }

// The raw conductivity level, for the serial diagnostic menu.
bool readConductivity() { return digitalRead(CONDUCTIVITY_PROBE_PIN) == HIGH; }

// The surface verdict the probe pair currently holds, for diagnostics.
SurfaceKind probeSurface() { return probeClassifier.surface(); }

// Forget the probe's surface verdict; the next sample re-proves it.
void resetProbe() { probeClassifier.reset(); }
