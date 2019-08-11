/*
  Based on:
  Ultra Low Power Led Flasher
  with AtTiny85 @ 1Mhz
  by Luca Soltoggio
  06/01/2014

  http://www.arduinoelettronica.com

*/
#define FW_VERSION  1.1.1

#include <avr/sleep.h>    // Sleep Modes
//#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ACTIVE_STATE LOW // HIGH - Normal closed relay, LOW - NMOS FET

#define PIN_RELAY 4  // relay or NMOS FET pin
#define PIN_OFF_TIME 3 // off time duration (mode) pin
#define PIN_TIMEOUT 0 // off time duration (mode) pin
#define PIN_DEBUG 4  // debug pulse on every wakeup
#define DEBUG_PULSE_DURATION 50  // debug pulse duration in ms

// WDT_S as follows:
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9=8sec
#define WDT_S  8
#define INCREMENT_SECS 4 // Must be in accordance with WDT_S

//volatile boolean f_wdt = 1;
volatile uint16_t secs_counter = 0;
uint8_t tout_mode, off_mode, test_mode;

//  -------- Jumper on PB0 to GND ---------
uint16_t timeout_secs[] = {14400 , 7200};
//                  PIN_TIMEOUT (LOW) (Jumper short) - 4h  14400s
//                  PIN_TIMEOUT (HIGH) (Jumper open) - 2h  7200s

uint16_t offtime_msecs[] = {15000, 10000};  // Jumper on PB3 to GND -> 15 secs OFF time, else 10 secs.

void setup() {
  wdt_reset();
  pinMode(PIN_TIMEOUT, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(PIN_OFF_TIME, INPUT_PULLUP);
  pinMode(PIN_RELAY, INPUT_PULLUP);

  tout_mode = digitalRead(PIN_TIMEOUT); // read timeout duration
  off_mode = digitalRead(PIN_OFF_TIME); // read OFF state duration
  delay(2000);
  test_mode = !digitalRead(PIN_RELAY); // read debug mode. During powering, if PIN_RElAY is connected to GND for 2s, device entering in test mode.

  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  cbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter OFF
  //sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  //  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  setup_watchdog(WDT_S);
}

void loop() {
  pinMode(PIN_RELAY, INPUT_PULLUP); // set all used ports to intput to save power

  system_sleep();

// In test mode the load should be replaced by LED or apropriate indicator for convinient observation.
// Every INCREMENT_SECS, LED will flashing for the number of the wake-ups since starting test mode.
// The device wakes-up every INCREMENT_SECS.

  if (test_mode) {
    pinMode(PIN_DEBUG, OUTPUT);
    for (uint16_t i = 0; i < (secs_counter / INCREMENT_SECS); i++) {
      digitalWrite(PIN_DEBUG, ACTIVE_STATE);
      delay(3 * DEBUG_PULSE_DURATION);
      digitalWrite(PIN_DEBUG, !ACTIVE_STATE);
      delay(5 * DEBUG_PULSE_DURATION);
    }
    pinMode(PIN_DEBUG, INPUT_PULLUP); // If PIN_DEBUG = PIN_RELAY, pinMode should be INPUT_PULLUP for stable work of the MOSFET. For Relay model this should be just INPUT.
  }


  if (secs_counter >= timeout_secs[tout_mode]) {
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, ACTIVE_STATE);
    delay(offtime_msecs[off_mode]);
    digitalWrite(PIN_RELAY, !ACTIVE_STATE);
    secs_counter = 0;
  }
}

// set system into the sleep state
// system wakes up when wtchdog is timed out
void system_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out
}


void setup_watchdog(int ii) {
  byte bb;
  int ww;

  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;

  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  secs_counter +=  INCREMENT_SECS;
}
