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
#define PWM_IN      8   // INT0
#define LED         4
#define CURRENT_IN  1   // from A4955 AIOUT
#define SLEEPN      3
#define OUT1        6   // OCR1A
#define OUT2        5   // OCR1B
#define VOLTAGE_IN  0   // from voltage divider

// CURRENT SENSOR
#define R_SENSE     0.005f            // ohms

// PWM INPUT TIMEOUT
#define INPUT_TOUT  0.050f            // s

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

// PWM OUTPUT CHARACTERISTICS
#define OUTPUT_DZ   0.60f             // speed 0.0~1.0 (deadzone)
#define MAX_DUTY    0.90f             // Limits duty cycle to save (12 V) motor

// FILTER PARAMETERS
#define FILTER_DT   0.050f            // s
#define TAUP_OUT    0.500f            // sets lp filtering of lead-lag (s)
#define TAUN_OUT    TAUP_OUT/1.5f     // sets starting gain of lead-lag (s)
#define TAU_CURRENT 0.03000f          // s
#define CURRENT_DT  0.00010f          // s

// STALL PARAMETERS
#define I_STALL_A0  6.47f             // stall current curve const. term
#define I_STALL_A1  -21.3f            // stall current curve linear term
#define I_STALL_A2  19.4f             // stall current curve quadratic term
#define I_LIMIT_IN  0.50f             // fraction of stall current (closing)
#define I_LIMIT_OUT 0.70f             // fraction of stall current (opening)

// Custom Enumerated Types
enum dir_t {
  NONE = 0,
  FORWARD,
  REVERSE
};

uint32_t lastpulsetime        = 0;
uint32_t updatefilterruntime  = 0;
uint32_t updatecurrentruntime = 0;

int16_t  pulsein  = PWM_NEUTRAL;
dir_t    limit    = NONE;
dir_t    lastdir  = NONE;
float    velocity = 0.0f;
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
  currentfilter.createFirstOrderLowPassFilter(CURRENT_DT, TAU_CURRENT);
  currentfilter.setSaturation(1.0f);

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED (for testing)
  digitalWrite(LED, LOW);
}

void loop() {
  // Make sure we're still receiving PWM inputs
  if ( (millis() - lastpulsetime)/1000.0f > INPUT_TOUT ) {
    // If it has been too long since the last input, shut off motor
    pulsein = PWM_NEUTRAL;

    // Force filters to update immediately
    updatefilterruntime = 0;
  } // end pwm input check

  // Run filters at specified interval
  if ( millis() > updatefilterruntime ) {
    // Set next filter runtime
    updatefilterruntime = millis() + FILTER_DT*1000;

    // Declare local variables
    float rawvelocity;
    dir_t direction;

    // Reject signals that are way off (i.e. const. 0 V, const. +5 V, noise)
    if ( pulsein >= INPUT_MIN && pulsein <= INPUT_MAX ) {
      // Remove neutral PWM bias & clamp to [-HALF_RANGE, HALF_RANGE]
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
        outputfilter.clear();
      }
    } else {
      // Stop motor if input is invalid
      rawvelocity = 0.0f;
      outputfilter.clear();
    }

    // Set filter gain based on input voltage
//     outputfilter.setGain();

    // Filter velocity
    velocity = constrain(outputfilter.step(rawvelocity), -MAX_COUNT,
                         MAX_COUNT);

    if (velocity > INPUT_DZ*CNT_PER_US) {
        direction = FORWARD;
    } else if (velocity < -INPUT_DZ*CNT_PER_US) {
        direction = REVERSE;
    } else {
        direction = NONE;
    }

    // Clear current filter if we change directions
    if (direction != lastdir) {
      currentfilter.clear();
    }


    // Current sensor resolution: ~65 mA
    if (direction == REVERSE && currentfilter.getLastOutput() > I_LIMIT_OUT) {
      limit = direction;
      digitalWrite(LED, LOW);
      // currentfilter.clear();
    } else if (direction == FORWARD && currentfilter.getLastOutput() > I_LIMIT_IN) {
      limit = direction;
      digitalWrite(LED, LOW);
      // currentfilter.clear();
    } else if ( /*direction != NONE &&*/ limit != NONE && direction != limit ) {
      // We're going the opposite direction from the limit, so clear limit
      limit = NONE;
      digitalWrite(LED, HIGH);
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

    // Save last direction
    lastdir = direction;
  } // end run filters

  // Current lp filter
  if ( micros() > updatecurrentruntime ) {
    // Set next current lp filter runtime
    updatecurrentruntime = micros() + CURRENT_DT*1000000;
    currentfilter.step(readCurrent()/stallCurrent(velocity));
  } // end current lp filter
}

float stallCurrent(float speed_counts) {
  float i_stall;
  float speed = abs(speed_counts/MAX_COUNT);
  if (speed > 0.9*OUTPUT_DZ && speed < 1.1) {
    i_stall = I_STALL_A0 + I_STALL_A1*speed + I_STALL_A2*speed*speed;
  } else {
    i_stall = INFINITY;
  }

  return i_stall;
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
  return (analogRead(CURRENT_IN)*3.3f)/(1023.0f*10.0f*R_SENSE);
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
    lastpulsetime = millis();
  }
}
