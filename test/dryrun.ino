// dryrun.ino — DRYRUN overlay skeleton for host-side module self-tests.
//
// Normal Arduino builds #include nothing from here. When compiled with
// -DDRYRUN this file compiles WITHOUT real hardware:
//   * AVR-only symbols are stubbed (pinMode/digitalWrite/analogRead/pulseIn...)
//   * Serial is a no-op test output stub that forwards to stdout on host
//   * setup() runs an in-sketch self-test wired to config.h/motors.h logic
//     as those modules land (guarded by __has_include so the skeleton
//     compiles before any module exists)
//   * the harness-readable global dryrun_result_ is set at the end
//
// This is a FOUNDATION SKELETON: grow the self-test body as modules land.
// It never contains mission behavior — it only feeds synthetic inputs and
// asserts outputs.

#ifdef DRYRUN

#include <cstdio>
#include <cstdint>

// --- harness-readable global: test asserts on this ---
int dryrun_result_ = -1;

extern "C" void setup();
extern "C" void loop();

// --- Serial output stub: mirrors printf on the host ---
namespace {
class SerialStub {
public:
    void print(const char* s) { std::printf("%s", s); }
    void println(const char* s) { std::printf("%s\n", s); }
    void println(long v) { std::printf("%ld\n", v); }
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
};
SerialStub Serial;
}  // namespace

// --- AVR-only hardware stubs: no-op under DRYRUN ---
typedef uint8_t byte;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline int analogRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}
inline unsigned long pulseIn(uint8_t, uint8_t, unsigned long) { return 0UL; }
inline unsigned long millis() { return 0UL; }
inline void delay(unsigned long) {}

// --- module inclusions, guarded so the skeleton compiles pre-modules ---
#if defined(__has_include)
#if __has_include("config.h")
#include "config.h"
#endif
#if __has_include("motors.h")
#include "motors.h"
#endif
#endif

// --- in-sketch self-test run from setup() ---
namespace dry_selftest {
namespace {
int passes = 0;
int fails = 0;

void check(bool ok, const char* msg) {
    if (ok) {
        ++passes;
    } else {
        ++fails;
        Serial.println("DRYRUN FAIL: ");
        Serial.println(msg);
    }
}

void run() {
    // Foundation checks: the stubs themselves answer sanely.
    check(millis() == 0UL, "stub millis starts at 0");
    check(analogRead(0) == 0, "stub analogRead returns 0");
    check(digitalRead(0) == 0, "stub digitalRead returns 0");
    check(pulseIn(0, 0, 0) == 0UL, "stub pulseIn returns 0");

    // Module wiring slots (activated as config.h/motors.h land):
    //   check(config_computed >= 0, "config parses under DRYRUN");
    //   check(motor_api_responds(), "motor layer builds under DRYRUN");
    (void)0;  // no behavior here by design
}
}  // namespace
}  // namespace dry_selftest

void setup() {
    Serial.begin(0);
    dry_selftest::run();
    dryrun_result_ = (dry_selftest::fails == 0) ? 0 : 1;
    Serial.println(dryrun_result_ == 0 ? "DRYRUN SELF-TEST PASS"
                                       : "DRYRUN SELF-TEST FAIL");
}

void loop() { (void)0; }

#endif  // DRYRUN