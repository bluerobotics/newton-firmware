/* Blue Robotics Newton Gripper Firmware
-----------------------------------------------------

Title: Blue Robotics Newton Gripper Firmware

Description: This code is the default firmware for the Blue Robotics
Newton Gripper.  A PWM signal dictates the speed at which the gripper opens or
closes, with speeds above 1500 (plus a deadzone) closing the jaws and those
below 1500 (minus a deadzone) openning the jaws.  A current sensor detects when
the endstops are reached or when an object is in the way of the jaws, and
disables the motor until commanded to stop or reverse direction.

The current detection code adjusts for different stall currents at different
input voltages between 9 and 18 volts DC.  Smooth operation is not guaranteed
outside this range and input voltages exceeding 21 VDC should be avoided as this
can cause damage to the microcontroller.

This code requires the following library:
https://github.com/dheideman/DiscreteFilter

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
#include "Newton-Gripper.h"

// Global Variables
uint32_t lastpulsetime        = 0;
uint32_t speedfilterruntime   = 0;
uint32_t currentfilterruntime = 0;

int16_t  pulsein  = PWM_NEUTRAL;
dir_t    limit    = NONE;
dir_t    lastdir  = NONE;
float    velocity = 0.0f;
DiscreteFilter speedfilter;
DiscreteFilter currentfilter;



///////////
// SETUP //
///////////

void setup() {
  // Set up pin modes
  pinMode(PWM_IN,     INPUT_PULLUP);  // Prevent floating input PWM w/ pullup
  pinMode(LED,        OUTPUT);
  pinMode(CURRENT_IN, INPUT);
  pinMode(SLEEPN,     OUTPUT);
  pinMode(OUT1,       OUTPUT);
  pinMode(OUT2,       OUTPUT);
  pinMode(VOLTAGE_IN, INPUT);

  // Initialize PWM input reader
  initializePWMReader();

  // Initialize PWM output generator
  initializePWMOutput();

  // Initialize output low-pass filter
  speedfilter.createLeadLagCompensator(FILTER_DT, TAUP_OUT, TAUN_OUT);

  // Initialize current input low-pass filter
  currentfilter.createFirstOrderLowPassFilter(CURRENT_DT, TAU_CURRENT);
  currentfilter.setSaturation(1.0f);

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED
  digitalWrite(LED, LOW);
}


//////////
// LOOP //
//////////

void loop() {
  // Make sure we're still receiving PWM inputs
  if ( (millis() - lastpulsetime)/1000.0f > INPUT_TOUT ) {
    // If it has been too long since the last input, shut off motor
    pulsein = PWM_NEUTRAL;
    lastpulsetime = millis();
  } // end pwm input check

  // Run speed filter at specified interval
  if ( millis() > speedfilterruntime ) {
    // Set next filter runtime
    speedfilterruntime = millis() + FILTER_DT*1000;

    // Update input filter
    runSpeedFilter();
  } // end run filters

  // Run current lp filter at specified interval
  if ( micros() > currentfilterruntime ) {
    // Set next current lp filter runtime
    currentfilterruntime = micros() + CURRENT_DT*1000000;

    // Update current filter
    currentfilter.step(readCurrent()/stallCurrent(velocity, readVoltage()));
  } // end current lp filter
}


///////////////
// Functions //
///////////////

/******************************************************************************
 * void runSpeedFilter()
 *
 * Calculates and generates output based on latest PWM input
 * Runs at 20 Hz
 ******************************************************************************/
void runSpeedFilter() {
  // Declare local variables
  float rawvelocity;
  dir_t direction;
  float voltage = readVoltage();
  float maxduty = constrain(maxDuty(voltage),0.0f,MAX_DUTY_ABS);
  float minduty = constrain(minDuty(voltage),0.0f,maxduty);

  // Reject signals that are way off (i.e. const. 0 V, const. +5 V, noise)
  if ( pulsein >= INPUT_MIN && pulsein <= INPUT_MAX ) {
    // Remove neutral PWM bias & clamp to [-HALF_RANGE, HALF_RANGE]
    int16_t pw = constrain(pulsein - PWM_NEUTRAL, -HALF_RANGE, HALF_RANGE);

    // Reject anything inside input deadzone
    if ( pw > INPUT_DZ ) {
    // Map valid PWM signals to (0.0 to 1.0]
      rawvelocity = (float)(pw - INPUT_DZ)/(HALF_RANGE - INPUT_DZ)
                    *(maxduty - minduty) + minduty;
    } else if ( pw < -INPUT_DZ ) {
      // Map valid PWM signals to [-1.0 to 0.0)
      rawvelocity = (float)(pw + INPUT_DZ)/(HALF_RANGE - INPUT_DZ)
                    *(maxduty - minduty) - minduty;
    } else {
      // Stop motor if input is within input deadzone
      rawvelocity = 0.0f;
    }
  } else {
    // Stop motor if input is invalid
    rawvelocity = 0.0f;
  }

  // Filter velocity
  velocity = constrain(speedfilter.step(rawvelocity), -maxduty, maxduty);

  // Figure out the current direction of travel
  if (velocity > MIN_DUTY_TOL*minduty) {
      direction = FORWARD;
  } else if (velocity < -MIN_DUTY_TOL*minduty) {
      direction = REVERSE;
  } else {
      direction = NONE;
  }

  // Clear current filter if we change directions
  if (direction != lastdir) {
    currentfilter.clear();
  }

  // Current sensor resolution: ~65 mA
  if (direction == FORWARD && currentfilter.getLastOutput() > I_LIMIT_OUT) {
    limit = direction;
    digitalWrite(LED, HIGH);
  } else if (direction == REVERSE && currentfilter.getLastOutput() > I_LIMIT_IN) {
    limit = direction;
    digitalWrite(LED, HIGH);
  } else if ( /*direction != NONE &&*/ limit != NONE && direction != limit ) {
    // We're going the opposite direction from the limit, so clear limit
    limit = NONE;
    digitalWrite(LED, LOW);
  }

  // Set output PWM timers
  if ( direction == FORWARD && limit != FORWARD) {
    OCR1A = abs(velocity)*MAX_COUNT;
    OCR1B = 0;
  } else if ( direction == REVERSE && limit != REVERSE) {
    OCR1A = 0;
    OCR1B = abs(velocity)*MAX_COUNT;
  } else {
    OCR1A = MAX_COUNT;
    OCR1B = MAX_COUNT;
  }

  // Save last direction
  lastdir = direction;
}


/******************************************************************************
 * float readCurrent()
 *
 * Reads current consumption (Amperes)
 ******************************************************************************/
float readCurrent() {
  // Reported voltage is 10x actual drop across the sense resistor
  return (analogRead(CURRENT_IN)*V_IN)/(1023.0f*10.0f*R_SENSE);
}

/******************************************************************************
 * float readVoltage()
 *
 * Reads input voltage (Volts)
 ******************************************************************************/
float readVoltage() {
  return (analogRead(VOLTAGE_IN)*V_IN)/(1023.0f*V_SENSE_DIV);
}

/******************************************************************************
 * float stallCurrent(float velocity, float voltage)
 *
 * Calculates stall current for given velocity and voltage (Amperes)
 ******************************************************************************/
float stallCurrent(float velocity, float voltage) {
  return (I_STALL_A0 + I_STALL_A1*voltage)*exp(I_STALL_B1*abs(velocity));
}

/******************************************************************************
 * float maxDuty(float voltage)
 *
 * Calculates maximum safe output duty cycle for given voltage [-1,1]
 ******************************************************************************/
float maxDuty(float voltage) {
  return MAX_DUTY_A0 + MAX_DUTY_A1*voltage + MAX_DUTY_A2*voltage*voltage;
}

/******************************************************************************
 * float minDuty(float voltage)
 *
 * Calculates minimum operational output duty cycle for given voltage [-1,1]
 ******************************************************************************/
float minDuty(float voltage) {
  return MIN_DUTY_A0 + MIN_DUTY_A1*voltage + MIN_DUTY_A2*voltage*voltage;
}


/******************************************************************************
 * void initializePWMOutput()
 *
 * Sets registers to run timers at the proper frequency for the PWM output
 ******************************************************************************/
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


/******************************************************************************
 * void initializePWMReader()
 *
 * Sets up external interrupt for PWM reader
 ******************************************************************************/
void initializePWMReader() {
  // Enable INT0
  bitSet(GIMSK, INT0);

  // Set interrupt sense control to "any logical change"
  bitSet  (MCUCR, ISC00);   // ISC00: 1
  bitClear(MCUCR, ISC01);   // ISC01: 0
}

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

// Define global variables only used for input timer
namespace {
  uint32_t inputpulsestart = 0xFFFF;
}

/******************************************************************************
 * SIGNAL(INT0_vect)
 *
 * Watches external interrupt to read PWM input
 ******************************************************************************/
SIGNAL(INT0_vect) {
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
