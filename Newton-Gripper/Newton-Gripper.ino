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
#define PWM         7   // ICP1
#define LED         4
#define CURRENT_IN  1   // from A4955 AIOUT
#define SLEEPN      3
#define OUT1        6
#define OUT2        5

// HARDWARE TIMER PINS
#define OC1A_PIN    6
#define OC1B_PIN    5

// PWM READ DEFINITIONS
#define PWM_FREQ    50                // Hz
#define PERIOD      1000000/PWM_FREQ  // us
#define PRESCALE    1                 // must match TCCR1B settings
#define CNT_PER_US  (F_CPU/PRESCALE/1000000L) // timer counts
#define MAX_COUNT   0xFFFF/0x10           // max unsigned 16-bit integer
// #define MAX_COUNT   0x03FFL           // max unsigned 10-bit integer

// PWM INPUT CHARACTERISTICS
#define PWM_MAX     2000              // us
#define PWM_MIN     1000              // us
#define PWM_NEUTRAL 1500              // us
#define HALF_RANGE  500               // us
#define DEADZONE    25                // us
#define INPUT_MAX   2500              // us
#define INPUT_MIN   500               // us

int32_t pulsein = PWM_NEUTRAL;
float   velocity  = 0;
bool    stopped   = true;


void setup() {
  // Set up pin modes
  pinMode(PWM,        INPUT_PULLUP);
  pinMode(LED,        OUTPUT);
  pinMode(CURRENT_IN, INPUT);
  pinMode(SLEEPN,     OUTPUT);
  pinMode(OC1A_PIN,   OUTPUT);
  pinMode(OC1B_PIN,   OUTPUT);

  // Initialize PWM input reader
  initializePWMReader();

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED (for testing)
  digitalWrite(LED, LOW);

// velocity = 0.1;
// OCR1B = (1.0f - abs(velocity)) * MAX_COUNT;
// while(1) { delay(100);}
}

void loop() {
  delay(10);
    // Constrain the PWM input
    if ( pulsein >= INPUT_MIN && pulsein <= INPUT_MAX ) {
      // Map valid PWM signals to [-1 to 1]
      pulsein  = constrain(pulsein, PWM_MIN, PWM_MAX);
      velocity = (float)(pulsein - PWM_NEUTRAL)/HALF_RANGE;
// velocity = 0.1;
      OCR1B = constrain(abs(velocity) * MAX_COUNT, 0, 0.9*MAX_COUNT);
    } else {
      // Stop motor if input is invalid
      velocity = 0;
      OCR1B = 0;
    }
}


void initializePWMReader() {
  // Stop interrupts while changing timer settings
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  // Set on compare match
//   bitSet(TCCR1A, COM1A0);
//   bitSet(TCCR1A, COM1A1);
//   bitSet(TCCR1A, COM1B0);
//   bitSet(TCCR1A, COM1B1);
  // Set CTC (OCR1A)
//   bitSet(TCCR1A, WGM10);
//   bitSet(TCCR1A, WGM11);
  bitSet(TCCR1B, WGM12);
//   bitSet(TCCR1B, WGM13);

  // Set timer1 clock source to prescaler 1
  bitSet(TCCR1B, CS10);
  bitClear(TCCR1B, CS11);
  bitClear(TCCR1B, CS12);

  // Enable timer1 Input Capture Interrupt
  bitSet(TIMSK1, ICIE1);

  // Enable timer1 Output Compare B Match Interrupt
  bitSet(TIMSK1, OCIE1B);

  // Enable timer1 Output Compare A Match Interrupt
  bitSet(TIMSK1, OCIE1A);

  // Enable timer1 Overflow Interrupt
//   bitSet(TIMSK1, TOIE1);

  // Enable timer1 Input Capture Noise Canceler
//   bitSet(TCCR1B, ICNC1);

  // Set timer1 Input Capture Edge Select
  // 0: falling edge, 1: rising edge
  bitSet(TCCR1B, ICES1);  // rising edge

  // Clear timer1 Input Capture Flag
  bitSet(TIFR1, ICF1);

  // Set timer1 TOP to max value
  OCR1A = MAX_COUNT;

  // Done setting timers -> allow interrupts again
  sei();
}

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

namespace {
  uint16_t pwmstart = 0;
  uint8_t  ncycles  = 0;
}

// Triggered when a change is detected on ICP1
///*
SIGNAL(TIM1_CAPT_vect) {
  if ( digitalRead(PWM) ) {
    // If we caught the rising edge
    // Save start time and clear ncycles
    pwmstart = ICR1;
    ncycles  = 0;

    // Reset for falling edge
    bitClear(TCCR1B, ICES1);  // detect falling edge

    // Reset Input Capture Flag
    bitSet(TIFR1, ICF1);
  } else {
    // If we caught the falling edge
    // Save input pulse width
    pulsein = (uint32_t)( ICR1 + ncycles*(MAX_COUNT+1) - pwmstart )/CNT_PER_US;

    // Reset for rising edge
    bitSet(TCCR1B, ICES1);  // detect rising edge

    // Reset Input Capture Flag
    bitSet(TIFR1, ICF1);
  }
}
//*/

// Triggered when TCNT1 == OCR1A (TOP)
SIGNAL(TIM1_COMPA_vect) {
  // Figure out which direction to turn
  if (velocity > 0.10) {
    // Start pulse on OC1A (forward)
    bitSet(PORTA,OC1A_PIN);
//     digitalWrite(OC1A_PIN, HIGH);
  }
  if ( velocity < -0.10) {
    // Start pulse on OC1B (reverse)
    bitSet(PORTA,OC1B_PIN);
//     digitalWrite(OC1B_PIN, HIGH);
  }

  // Increment ncycles
  ncycles++;
}

// Triggered when TCNT1 == OCR1B
SIGNAL(TIM1_COMPB_vect) {
  // End pulse
  PORTA &= B10011111;
//   digitalWrite(OC1A_PIN, LOW);
//   digitalWrite(OC1B_PIN, LOW);
}
