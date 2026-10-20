// avr/wdt.h — inert host stand-in for the AVR watchdog peripheral.
//
// The Mega delivery files include the real <avr/wdt.h> for wdt_reset() and the
// commented-out wdt_enable(). On the host there is no watchdog, so both become
// no-ops: the suite that drives fsm.ino proves the table and the tick, never any
// peripheral behaviour. Arming the hardware watchdog stays a bench decision
// (see the note in src/rescue_robot/fsm.ino).

#ifndef ARDUINO_STUB_AVR_WDT_H
#define ARDUINO_STUB_AVR_WDT_H

#define wdt_enable(timeout) ((void)(timeout))
#define wdt_reset() ((void)0)

#endif  // ARDUINO_STUB_AVR_WDT_H
