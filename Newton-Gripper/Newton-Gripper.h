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

#include <Arduino.h>

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

// VOLTAGE SENSOR
#define V_IN        3.3f              // volts
#define V_SENSE_DIV 0.155f            // v_sense voltage divider (3.3/(3.3+18))

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
#define INPUT_DZ    30                // us
#define PRESCALE    1                 // must match TCCR1B settings
#define CNT_PER_US  (F_CPU/PRESCALE/1000000L) // timer counts
#define MAX_COUNT   0x0FFF          // sets output PWM frequency (~1 kHz)

// PWM OUTPUT CHARACTERISTICS
#define OUTPUT_DZ   0.60f             // speed 0.0~1.0 (deadzone)
#define MAX_DUTY    1.00f             // sets maximum output @ reference voltage

// MAX/MIN DUTY PARAMETERS
#define MAX_DUTY_A0 1.55f             // max duty curve constant term
#define MAX_DUTY_A1 -0.060f           // max duty curve linear term
#define MAX_DUTY_A2 0.0011f           // max duty curve quadratic term
#define MIN_DUTY_A0 0.87f             // min duty curve constant term
#define MIN_DUTY_A1 -0.027f           // min duty curve linear term
#define MIN_DUTY_A2 0.00048f          // min duty curve quadratic term

// FILTER PARAMETERS
#define FILTER_DT   0.050f            // s
#define TAUP_OUT    0.500f            // sets lp filtering of lead-lag (s)
#define TAUN_OUT    TAUP_OUT/1.5f     // sets starting gain of lead-lag (s)
#define TAU_CURRENT 0.03000f          // s
#define CURRENT_DT  0.00010f          // s

// STALL PARAMETERS
#define I_STALL_A0  0.00514f          // stall current const. coefficient term
#define I_STALL_A1  0.00294f          // stall current linear coefficient term
#define I_STALL_B1  4.69f             // stall current exponential term
#define I_LIMIT_IN  0.50f             // fraction of stall current (closing)
#define I_LIMIT_OUT 0.70f             // fraction of stall current (opening)

// CURRENT/VOLTAGE PARAMETERS
#define V_BASE      12.0f             // voltage at which measurements were made
#define I_RATIO_A1  0.0833f           // current ratio linear term


// Custom Enumerated Types
enum dir_t {
  NONE = 0,
  FORWARD,
  REVERSE
};
