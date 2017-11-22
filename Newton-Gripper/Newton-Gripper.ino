/* Blue Robotics Newton Gripper Firmware
-----------------------------------------------------

Title: Blue Robotics Newton Gripper Firmware

Description: This code is the default firmware for the Blue Robotics
Newton Gripper, which...

The code is designed for the ATtiny84 microcontroller and can be compiled and
uploaded via the Arduino 1.0+ software.

-------------------------------
The MIT License (MIT)

Copyright (c) 2017 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#include <DiscreteFilter.h>

// HARDWARE PIN DEFINITIONS
#define PWM_IN      7
#define LED         4
#define CURRENT_IN  1   // from A4955 AIOUT
#define SLEEPN      3
#define OUT1        6   // OCR1A
#define OUT2        5   // OCR1B

// CURRENT SENSOR
#define R_SENSE     0.005f            // ohms

// PWM READ DEFINITIONS
#define PWM_FREQ    50                // Hz
#define PERIOD      1000000/PWM_FREQ  // us
#define PWM_MAX     1900              // us
#define PWM_MIN     1100              // us
#define PWM_NEUTRAL 1500              // us
#define HALF_RANGE  400               // us
#define INPUT_MAX   2500              // us
#define INPUT_MIN   500               // us
#define INPUT_DZ    25                // us
#define PRESCALE    1                 // must match TCCR1B settings
#define CNT_PER_US  (F_CPU/PRESCALE/1000000L) // timer counts
#define MAX_COUNT   0x0FFF          // sets output PWM frequency (~1 kHz)
// #define MAX_COUNT   0x03FFL           // max unsigned 10-bit integer

// PWM OUTPUT CHARACTERISTICS
#define OUTPUT_DZ   0.35f             // speed 0.0~1.0 (deadzone)
#define MAX_DUTY    0.90f             // Limits duty cycle to save (12 V) motor

// FILTER PARAMETERS
#define FILTER_DT   0.050f            // s
#define TAUP_OUT    0.500f            // sets lp filtering of lead-lag (s)
#define TAUN_OUT    TAUP_OUT/1.7f     // sets starting gain of lead-lag (s)
#define TAU_CURRENT 1.000f            // s

// Custom Enumerated Types
enum dir_t {
  NONE = 0,
  FORWARD,
  REVERSE
};

int16_t pulsein = PWM_NEUTRAL;
dir_t   limit   = NONE;
DiscreteFilter outputfilter;
DiscreteFilter currentfilter;


void setup() {
  // Set up pin modes
  pinMode(PWM_IN,     INPUT_PULLUP);  // Prevent floating input PWM w/ pullup
  pinMode(LED,        OUTPUT);
  pinMode(CURRENT_IN, INPUT);
  pinMode(SLEEPN,     OUTPUT);
  pinMode(OUT1,       OUTPUT);
  pinMode(OUT2,       OUTPUT);

  // Initialize PWM input reader
  initializePWMReader();

  // Initialize PWM output generator
  initializePWMOutput();

  // Initialize output low-pass filter
  outputfilter.createLeadLagCompensator(FILTER_DT, TAUP_OUT, TAUN_OUT);

  // Initialize current input low-pass filter
  currentfilter.createFirstOrderLowPassFilter(FILTER_DT, TAU_CURRENT);

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED (for testing)
  digitalWrite(LED, LOW);
}

void loop() {
//   cli();

  float rawvelocity, velocity;
  dir_t direction;

  // Reject signals that are way off (i.e. const. 0 V, const. +5 V, noise)
  if ( pulsein >= INPUT_MIN && pulsein <= INPUT_MAX ) {
    // Remove neutral PWM bias & clamp to [-HALF_RANGE, HALF_RANGE] pulse width
    int16_t pw = constrain(pulsein - PWM_NEUTRAL, -HALF_RANGE, HALF_RANGE);

    // Reject anything inside input deadzone
    if ( pw > INPUT_DZ ) {
      // Map valid PWM signals to (0.0 to 1.0]
      rawvelocity = ((float)(pw - INPUT_DZ)/(HALF_RANGE - INPUT_DZ)
                    *(MAX_DUTY - OUTPUT_DZ) + OUTPUT_DZ)*MAX_COUNT;
    } else if ( pw < -INPUT_DZ ) {
      // Map valid PWM signals to [-1.0 to 0.0)
      rawvelocity = ((float)(pw + INPUT_DZ)/(HALF_RANGE - INPUT_DZ)
                    *(MAX_DUTY - OUTPUT_DZ) - OUTPUT_DZ)*MAX_COUNT;
    } else {
      // Stop motor if input is within input deadzone
      rawvelocity = 0.0f;
    }
  } else {
    // Stop motor if input is invalid
    rawvelocity = 0.0f;
  }

  // Set filter gain based on input voltage
//   outputfilter.setGain();

  // Filter velocity
  velocity = constrain(outputfilter.step(rawvelocity), -MAX_COUNT,
                       MAX_COUNT);
//   velocity = constrain(rawvelocity, -MAX_DUTY*MAX_COUNT, MAX_DUTY*MAX_COUNT);

  // Determine direction
  if ( velocity > OUTPUT_DZ*MAX_COUNT) {
    direction = FORWARD;
  } else if ( velocity < -OUTPUT_DZ*MAX_COUNT) {
    direction = REVERSE;
  } else {
    direction = NONE;
  }

  // Current sensor resolution: ~65 mA
//   digitalWrite(LED, currentfilter->step(readCurrent()) > 0.130f);
//   if (currentfilter.step(readCurrent()) > currentLimit(abs(velocity))) {
  if (currentfilter.step(readCurrent()/currentSteadyState(abs(velocity))) > 2.0f) {
    limit = direction;
  } else if ( /*direction != NONE &&*/ limit != NONE && direction != limit ) {
    // We're going the opposite direction from the limit, so clear limit
    limit = NONE;
  }

  // Set output PWM timers
  if ( direction == FORWARD && limit != FORWARD) {
    OCR1A = abs(velocity);
    OCR1B = 0;
  } else if ( direction == REVERSE && limit != REVERSE) {
    OCR1A = 0;
    OCR1B = abs(velocity);
  } else {
    OCR1A = MAX_COUNT;
    OCR1B = MAX_COUNT;
  }

// digitalWrite(LED, pulsein <= PWM_NEUTRAL);

//   sei();
  delay(FILTER_DT*1000);
}

float currentLimit(float speed) {
  return map(speed, OUTPUT_DZ*MAX_COUNT, MAX_DUTY*MAX_COUNT, 400, 600)/1000.0f;
}

float currentSteadyState(float speed) {
  return map(speed, OUTPUT_DZ*MAX_COUNT, MAX_DUTY*MAX_COUNT, 120, 190)/1000.0f;
}

void initializePWMOutput() {
  // Stop interrupts while changing timer settings
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  // Set non-inverting PWM Mode
  bitSet(TCCR1A, COM1A1);
  bitSet(TCCR1A, COM1B1);

  // Set PWM. Phase & Freq. Correct (ICR1)
  bitClear(TCCR1A, WGM10);  // WGM10: 0
  bitClear(TCCR1A, WGM11);  // WGM11: 0
  bitClear(TCCR1B, WGM12);  // WGM12: 0
  bitSet  (TCCR1B, WGM13);  // WGM13: 1

  // Set timer1 clock source to prescaler 1
  bitSet  (TCCR1B, CS10);   // CS10: 1
  bitClear(TCCR1B, CS11);   // CS11: 0
  bitClear(TCCR1B, CS12);   // CS12: 0

  // Set timer1 TOP (stored in ICR1)
  ICR1 = MAX_COUNT;

  // Done setting timers -> allow interrupts again
  sei();
}

// Set up pin change interrupt for PWM reader
void initializePWMReader() {
  // Enable PCI1 (pins 0~7)
  bitSet(GIMSK, PCIE0);

  // Enable PCI for PWM input pin
  bitSet(PCMSK0, PWM_IN);
}

// Read current (in amperes)
float readCurrent() {
  // Reported voltage is 10x actual drop across the sense resistor
  return (analogRead(CURRENT_IN)*3.3f/1023.0f/10.0f)/R_SENSE;
}


////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

// Define global variables only used for input timer
namespace {
  uint32_t inputpulsestart = 0xFFFF;
}

// Read PWM input
SIGNAL(PCINT0_vect) {
  if (digitalRead(PWM_IN)) {
    // Record start of input pulse
    inputpulsestart = micros();
  } else {
    // Measure width of input pulse
    // Only use data for which the start time comes first (ignore rollovers)
    if ( inputpulsestart < micros() ) {
      pulsein = micros() - inputpulsestart;
    }
  }
}
