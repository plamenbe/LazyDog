/*
  Based on:
  Ultra Low Power Led Flasher
  with AtTiny85 @ 1Mhz
  by Luca Soltoggio
  06/01/2014

  http://www.arduinoelettronica.com

*/
#define FW_VERSION  1.1.2

//#include <avr/power.h>                                        // Power management
#include <avr/sleep.h>                                          // Sleep Modes
#include <avr/wdt.h>                                            // Watchdog timer

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ACTIVE_STATE LOW                                        // HIGH - Normal closed relay, LOW - NMOSFET

#define PIN_RELAY 4                                             // relay or NMOS FET control pin
#define PIN_OFF_TIME 3                                          // off time duration selection pin
#define PIN_TIMEOUT 0                                           // timeout period selection pin 
#define PIN_DEBUG 4                                             // debug pulse output pin
#define PIN_DEBUG_MODE 4                                        // test mode activation pin
#define DEBUG_PULSE_DURATION 50                                 // debug pulse duration base in ms

// WDT_S as follows:
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9=8sec
#define WDT_S  8
#define INCREMENT_SECS 4                                        // Must be in accordance with WDT_S

//volatile boolean f_wdt = 1;
volatile uint16_t secs_counter = 0;
uint8_t tout_mode, off_mode, test_mode;

//  -------- Jumper on PB0 to GND ---------
uint16_t timeout_secs[] = {14400 , 7200};
//                  PIN_TIMEOUT (LOW) (Jumper short) - 4h  14400s
//                  PIN_TIMEOUT (HIGH) (Jumper open) - 2h  7200s

//uint16_t timeout_secs[] = {15 , 10};                            // For development purposes

//  ------- Jumper on PB3 to GND
uint16_t offtime_msecs[] = {15000, 10000};                     // Jumper on PB3 to GND -> 15 secs OFF time, else 10 secs.
//                  PIN_OFF_TIME (LOW) (Jumper short) - 15s
//                  PIN_OFF_TIME (HIGH) (Jumper open) - 10s


void setup() {
  wdt_reset();
  pinMode(PIN_RELAY, INPUT_PULLUP);                           // should be configured ASAP after the start, to avoid start-up delay of the load
  pinMode(PIN_TIMEOUT, INPUT_PULLUP);
  pinMode(PIN_OFF_TIME, INPUT_PULLUP);
  pinMode(PIN_DEBUG_MODE, INPUT_PULLUP);
  pinMode(1, INPUT);                                          // saving power (unused pin)
  pinMode(2, INPUT);                                          // saving power (unused pin)

// ------ Read settings
  tout_mode = digitalRead(PIN_TIMEOUT);                       // read timeout duration
  off_mode = digitalRead(PIN_OFF_TIME);                       // read OFF state duration
  delay(2000);                                                // if PIN_DEBUG_MODE is holded for at least 2 second to GND, test mode begins
  test_mode = !digitalRead(PIN_DEBUG_MODE);                   // senses test mode

  pinMode(PIN_TIMEOUT, INPUT);                                // saving power
  pinMode(PIN_OFF_TIME, INPUT);
  
  cbi(ADCSRA, ADEN);                                          // switch Analog to Digital converter OFF
  setup_watchdog(WDT_S);
}

void loop() {
  pinMode(PIN_RELAY, INPUT_PULLUP);                           // must be INPUT_PULLUP for MOSFET

  system_sleep();                                             // execution continues after this line, every time when MCU wakes-up
                                                              // The device wakes-up every INCREMENT_SECS.


  // In test mode the load should be replaced by LED or apropriate indicator for convinient observation.
  // Every INCREMENT_SECS, LED will flashing for the number of the wake-ups since the beginning of the test mode.

  if (test_mode & digitalRead(PIN_DEBUG_MODE)) {              // check in case PIN_DEBUG_MODE = PIN_DEBUG, does PIN_DEBUG_MODE is still connected to ground,
    pinMode(PIN_DEBUG, OUTPUT);                               // for avoiding short circuit when PIN_DEBUG rises to HIGH
    for (uint16_t i = 0; i < (secs_counter / INCREMENT_SECS); i++) {
      digitalWrite(PIN_DEBUG, ACTIVE_STATE);
      delay(3 * DEBUG_PULSE_DURATION);
      digitalWrite(PIN_DEBUG, !ACTIVE_STATE);
      delay(5 * DEBUG_PULSE_DURATION);
    }
    pinMode(PIN_DEBUG, INPUT_PULLUP);                       // If PIN_DEBUG = PIN_RELAY, pinMode should be INPUT_PULLUP for stable work of the MOSFET. For Relay model this should be just INPUT.
  }

// ------ Check for changed settings on every wake-up
  pinMode(PIN_TIMEOUT, INPUT_PULLUP);
  pinMode(PIN_OFF_TIME, INPUT_PULLUP);

  if (tout_mode != digitalRead(PIN_TIMEOUT)) {
    secs_counter = 0;                                       // if timeout was changed, restart counter
  }
  tout_mode = digitalRead(PIN_TIMEOUT);                     // read timeout duration
  off_mode = digitalRead(PIN_OFF_TIME);                     // read OFF state duration

  pinMode(PIN_TIMEOUT, INPUT);                              // saving power
  pinMode(PIN_OFF_TIME, INPUT);

// ------ Check for timeout conditions and shuts down the load for off_mode seconds
  if ((secs_counter >= timeout_secs[tout_mode]) & !test_mode) { // also check for active test mode, if it's true - skip normal operation
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                    // sleep mode is set here
  sleep_enable();
  sleep_mode();                                           // System sleeps here
  sleep_disable();                                        // System continues execution here when watchdog timed out
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
