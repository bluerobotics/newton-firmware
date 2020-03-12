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

#include <Arduino.h>

// HARDWARE PIN DEFINITIONS
#define PWM_IN        8                 // INT0
#define LED           4
#define CURRENT_IN    1                 // from A4955 AIOUT
#define SLEEPN        3
#define OUT1          6                 // OCR1A
#define OUT2          5                 // OCR1B
#define VOLTAGE_IN    0                 // from voltage divider
#define OCL           2                 // OCP and OVP output flag, open drain

// PWM READ DEFINITIONS
#define PWM_FREQ      50                // Hz
#define PERIOD        1000000/PWM_FREQ  // us
#define PWM_MAX       1900              // us
#define PWM_MIN       1100              // us
#define PWM_NEUTRAL   1500              // us
#define HALF_RANGE    400               // us
#define INPUT_MAX     2500              // us
#define INPUT_MIN     500               // us
#define INPUT_DZ      30                // us
#define INPUT_TOUT    0.050f            // s

//Fault Detection Parameters
#define OCL_DT        0.010f            // s 100Hz

// VOLTAGE SENSOR AND FILTER DEFINITIONS
#define V_IN          3.3f              // volts
#define V_SENSE_DIV   0.155f            // v_sense voltage divider: 3.3/(3.3+18)

// CURRENT SENSOR AND FILTER PARAMETERS
#define R_SENSE       0.005f            // ohms
#define TAU_CURRENT   0.0050f          // s
#define CURRENT_DT    0.00010f          // s

// VELOCITY FILTER PARAMETERS
#define FILTER_DT     0.0050f           // s
#define TAUP_OUT      0.015f            // sets rise time of lead-lag (s) increase to slow rise time
#define TAUN_OUT      TAUP_OUT/0.35f     // sets starting gain of lead-lag. Gain(t=0) = TAUN_OUT/TAUP_OUT = TAUP_OUT/(0.35f*TAUP_OUT)
#define BRAKE         0.94f             // 0.0 to 1.0 fraction of MAX_COUNT for braking. Default = 0.94   

//PWM OUTPUT DEFINITIONS
#define PRESCALE      1                 // must match TCCR1B settings
#define CNT_PER_US    (F_CPU/PRESCALE/1000000L) // timer counts
#define MAX_COUNT     0x0FFF            // sets output PWM freq. (~1 kHz)

// STALL PARAMETERS
#define MOT_FS        1.50f             // Factor to guarantee that the ESC will shutoff Motor
#define R_MOT         3.16f             // Motor Resistance (Ohms)
#define MOT_I_MAX     3.8f              // Sets the upper bound of stall current. Usefull when gripper is driven outside of rated voltage. 
#define I_LIMIT_CLOSE 0.70f             // fraction of stall current (closing)
#define I_LIMIT_OPEN  0.95f             // fraction of stall current (opening)


// Custom Enumerated Types
enum dir_t {
  NONE = 0,
  OPEN,
  CLOSE
};
