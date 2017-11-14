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

// HARDWARE PIN DEFINITIONS
#define PWM_IN      ICP1_PIN
#define LED         4
#define CURRENT_IN  1   // from A4955 AIOUT
#define SLEEPN      3
#define OUT1        6
#define OUT2        5

// HARDWARE TIMER PINS
#define ICP1_PIN    7

// PWM READ DEFINITIONS
#define PWM_FREQ    50                // Hz
#define PERIOD      1000000/PWM_FREQ  // us
#define PRESCALE    1                 // must match TCCR1B settings
#define CNT_PER_US  (F_CPU/PRESCALE/1000000L) // timer counts
#define MAX_COUNT   0x0FFF            // also sets output PWM frequency (~2 kHz)
// #define MAX_COUNT   0x03FFL           // max unsigned 10-bit integer

// PWM OUTPUT CHARACTERISTICS
#define PWM_MAX     1900              // us
#define PWM_MIN     1100              // us
#define PWM_NEUTRAL 1500              // us
#define HALF_RANGE  400               // us
#define INPUT_MAX   2500              // us
#define INPUT_MIN   500               // us
#define OUTPUT_DZ   0.10f             // speed 0.0~1.0 (deadzone)
#define MAX_DUTY    0.90f             // Limits duty cycle to save (12 V) motor

int32_t pulsein = PWM_NEUTRAL;
float   velocity  = 0;


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

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED (for testing)
  digitalWrite(LED, LOW);
}

void loop() {
  delay(10);
  // Reject signals that are way off (i.e. const. 0 V, const. +5 V, noise)
  if ( pulsein >= INPUT_MIN && pulsein <= INPUT_MAX ) {
    // Clamp PWM inputs to [PWM_MIN, PWM_MAX] range
    pulsein  = constrain(pulsein, PWM_MIN, PWM_MAX);

    // Map valid PWM signals to [-1 to 1]
    velocity = (float)(pulsein - PWM_NEUTRAL)/HALF_RANGE;
  } else {
    // Stop motor if input is invalid
    velocity = 0;
  }
  // Set timer (do not allow speeds above MAX_DUTY)
  float speed = abs(velocity);
  OCR1B = constrain(speed * MAX_DUTY * MAX_COUNT, 0, MAX_DUTY * MAX_COUNT);
}


void initializePWMReader() {
  // Stop interrupts while changing timer settings
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  // Set Clear Timer on Compare (CTC) (OCR1A)
  bitClear(TCCR1A, WGM10);  // WGM10: 0
  bitClear(TCCR1A, WGM11);  // WGM11: 0
  bitSet(TCCR1B, WGM12);    // WGM12: 1
  bitClear(TCCR1B, WGM13);  // WGM13: 0

  // Set timer1 clock source to prescaler 1
  bitSet(TCCR1B, CS10);     // CS10: 1
  bitClear(TCCR1B, CS11);   // CS11: 0
  bitClear(TCCR1B, CS12);   // CS12: 0

  // Enable timer1 Input Capture Interrupt
  bitSet(TIMSK1, ICIE1);

  // Enable timer1 Output Compare B Match Interrupt
  bitSet(TIMSK1, OCIE1B);

  // Enable timer1 Output Compare A Match Interrupt
  bitSet(TIMSK1, OCIE1A);

  // Enable timer1 Input Capture Noise Canceler
  bitSet(TCCR1B, ICNC1);

  // Set timer1 Input Capture Edge Select
    // 0: falling edge, 1: rising edge
  bitSet(TCCR1B, ICES1);  // detect rising edge

  // Set timer1 TOP (stored in OCR1A for this CTC mode)
  OCR1A = MAX_COUNT;

  // Done setting timers -> allow interrupts again
  sei();
}

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

// Define global variables only used in ISRs
namespace {
  uint16_t pwmstart = 0;
  uint8_t  ncycles  = 0;
}

// Triggered when a change is detected on ICP1
SIGNAL(TIM1_CAPT_vect) {          // (microcontroller-specific interrupt name)
  if ( digitalRead(ICP1_PIN) ) {
    // If we caught the rising edge
    // Save start time and clear ncycles
    pwmstart = ICR1;
    ncycles  = 0;

    // Reset for falling edge
    bitClear(TCCR1B, ICES1);  // detect falling edge
  } else {
    // If we caught the falling edge
    // Save input pulse width (take into account timer overflows)
    pulsein = (int32_t)( ICR1 + ncycles*(MAX_COUNT+1) - pwmstart )/CNT_PER_US;

    // Reset for rising edge
    bitSet(TCCR1B, ICES1);  // detect rising edge
  }
}

// Triggered when TCNT1 == OCR1A (TOP)
SIGNAL(TIM1_COMPA_vect) {         // (microcontroller-specific interrupt name)
  // Figure out which direction to turn
  if (velocity > OUTPUT_DZ) {
    // Start pulse on output pin 1 (forward)
      // Setting bits is faster than digitalWrite()
      // (port/pin combination is microcontroller-specific)
    bitSet(PORTA, OUT1); // turn on output pin 1
  }
  if ( velocity < -OUTPUT_DZ) {
    // Start pulse on output pin 2 (reverse)
      // Setting bits is faster than digitalWrite()
      // (port/pin combination is microcontroller-specific)
    bitSet(PORTA, OUT2); // turn on output pin 2
  }

  // Increment timer overflow count for input PWM detector
  ncycles++;
}

// Triggered when TCNT1 == OCR1B
SIGNAL(TIM1_COMPB_vect) {         // (microcontroller-specific interrupt name)
  // End pulse - clear both output pins
    // Setting bits is faster than digitalWrite()
    // (port/pin combination is microcontroller-specific)
  bitClear(PORTA, OUT1);
  bitClear(PORTA, OUT2);
}
