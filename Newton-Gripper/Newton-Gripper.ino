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
#define MAX_COUNT   0xFFFF            // max unsigned 16-bit integer
// #define MAX_COUNT   0x03FFL           // max unsigned 10-bit integer

// PWM INPUT CHARACTERISTICS
#define PWM_MAX     2000              // us
#define PWM_MIN     1000              // us
#define PWM_NEUTRAL 1500              // us
#define HALF_RANGE  500               // us
#define DEADZONE    25                // us

int32_t inputpulsewidth = PWM_NEUTRAL;
bool    ledon   = false;
bool    stopped = true;


void setup() {
  // Set up pin modes
  pinMode(PWM,        INPUT);
  pinMode(LED,        OUTPUT);
  pinMode(CURRENT_IN, INPUT);
  pinMode(SLEEPN,     OUTPUT);
  pinMode(OC1A_PIN,   OUTPUT);
  pinMode(OC1B_PIN,   OUTPUT);

  // Initialize PWM input reader
  initializePWMReader();

  // Wake up A4955
  digitalWrite(SLEEPN, HIGH);

  digitalWrite(LED, LOW);
digitalWrite(OC1A_PIN, LOW);

  OCR1A = MAX_COUNT;
  OCR1B = MAX_COUNT;
// while (1) {delay(100);}
//   digitalWrite(OUT1, HIGH);
//   digitalWrite(OUT2, LOW);
}

void loop() {
  float velocity;

  // Constrain the PWM input
  if ( inputpulsewidth >= PWM_MIN && inputpulsewidth <= PWM_MAX ) {
    // Map valid PWM signals to [-1 to 1]
    velocity = (float) (inputpulsewidth - PWM_NEUTRAL)/HALF_RANGE;
  } else {
    // Stop motor if input is invalid
    velocity = 0;
  }

// velocity = 0.5;
///*
  float speed   = abs(velocity);
  bool  forward = (velocity > 0);

  // Check deadband
  if ( speed < 0.05 ) {
    // Coast to a stop.  No point in hitting the brakes, there's no control
    OCR1A = MAX_COUNT;
    OCR1B = MAX_COUNT;
    stopped = true;
  } else {
    stopped = false;
    if (forward) {
      OCR1A = (1.0 - speed) * MAX_COUNT;
      OCR1B = MAX_COUNT;
    } else {
      OCR1A = MAX_COUNT;
      OCR1B = (1.0 - speed) * MAX_COUNT;
    }
  }
//*/
//   digitalWrite(LED, ledon);
  delay(100);
}


// Set the motor off and driving
// void setMotor(float drive) {
//   float speed = abs(drive);
//
//   if ( speed < 0.05 ) {
//     OCR1A = MAX_COUNT;  // coast
//     OCR1B = MAX_COUNT;  // coast
//   } else if ( speed <= 1.0 ) {
//     if ( drive > 0.0 ) {
//       OCR1A = speed * (MAX_COUNT + 1) - 1;
//       OCR1B = 0;
//     } else {
//       OCR1A = 0;
//       OCR1B = speed * (MAX_COUNT + 1) - 1;
//     }
//   }
// }

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
  // Set normal mode (10-bit)
//   bitSet(TCCR1A, WGM10);
//   bitSet(TCCR1A, WGM11);
//   bitSet(TCCR1B, WGM12);
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
  bitSet(TIMSK1, TOIE1);

  // Enable timer1 Input Capture Noise Canceler
  bitSet(TCCR1B, ICNC1);

  // Set timer1 Input Capture Edge Select
  // 0: falling edge, 1: rising edge
  bitSet(TCCR1B, ICES1);  // rising edge

  // Clear timer1 Input Capture Flag
  bitSet(TIFR1, ICF1);

  // Set timer1 TOP to max value
//   OCR1A = MAX_COUNT;

  // Done setting timers -> allow interrupts again
  sei();
}

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

namespace {
  int16_t pwmstart = 0;
}

// Triggered when a change is detected on ICP1
///*
SIGNAL(TIM1_CAPT_vect) {
  if ( digitalRead(PWM) ) {
    // If we caught the rising edge
    // Save start time and clear ncycles
    pwmstart = ICR1;

    // Reset for falling edge
    bitClear(TCCR1B, ICES1);  // detect falling edge

    // Reset Input Capture Flag
    bitSet(TIFR1, ICF1);

// digitalWrite(LED, HIGH);
  } else {
    // If we caught the falling edge
    // Save input pulse width
    if(ICR1 > pwmstart) {
      inputpulsewidth = (ICR1 - pwmstart)/CNT_PER_US;
    } else {
      inputpulsewidth = ((MAX_COUNT - pwmstart) + ICR1 + 1)/CNT_PER_US;
    }

    // Reset for rising edge
    bitSet(TCCR1B, ICES1);  // detect rising edge

    // Reset Input Capture Flag
    bitSet(TIFR1, ICF1);

//  digitalWrite(LED, LOW);
  }
}
//*/

// Triggered when TCNT1 == OCR1B
SIGNAL(TIM1_COMPB_vect) {
  // Start pulse
  if (OCR1B < 0.95*MAX_COUNT) {
    digitalWrite(OC1B_PIN, HIGH);
  }
}

// Triggered when TCNT1 == OCR1A
SIGNAL(TIM1_COMPA_vect) {
  // Start pulse
  if (OCR1A < 0.95*MAX_COUNT) {
    digitalWrite(OC1A_PIN, HIGH);
  }
}

// Triggered when timer1 overflows
SIGNAL(TIM1_OVF_vect) {
    // End pulse
// ledon = true;
  digitalWrite(OC1A_PIN, LOW);
  digitalWrite(OC1B_PIN, LOW);
}
