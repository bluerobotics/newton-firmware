/* Blue Robotics Newton Gripper Firmware
-----------------------------------------------------

Title: Blue Robotics Newton Gripper Firmware - Low-Pass Filter

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

#include "LPFilter.h"

//////////////////
// Constructors //
//////////////////

// Default Constructor
LPFilter::LPFilter()
{
  this->_initialized = false;
  this->_dt   = 0.0f;
  this->_tau  = 0.0f;
  this->clear();
  this->_updateTF();
}

// Constructor: period & time constant
LPFilter::LPFilter(float dt, float tau)
{
  this->_initialized = false;
  this->_dt   = dt;
  this->_tau  = tau;
  this->clear();
  this->_updateTF();
}

// Destructor
LPFilter::~LPFilter()
{
  delete this->_inputs;
  delete this->_outputs;
  delete this->_num;
  delete this->_den;
}


////////////////////
// Public Methods //
////////////////////

// Move filter along one timestep, return filtered output
float LPFilter::step(float input)
{
//   return input;
  if ( this->_initialized ) {
    // Update _inputs
    this->_inputs[1]  = this->_inputs[0];
    this->_inputs[0]  = input;

    // Update _outputs
    this->_outputs[1] = this->_outputs[0];

    // Initialize output value
    float output = 0;

    // Run filter
    // Handle input
    for ( int i = 0; i < 2; i++ ) {
      output += this->_num[i] * this->_inputs[i];
    }

    // Handle output (skip i=0, since that's what we're calculating)
    for ( int i = 1; i < 2; i++ ) {
      output -= this->_den[i] * this->_outputs[i];
    }

    // Save latest output
    this->_outputs[0] = output/(this->_den[0]);
  }

  return this->_outputs[0];
}

// Getter: _dt
float LPFilter::getPeriod()
{
  return this->_dt;
}

// Getter: _tau
float LPFilter::getTimeConstant()
{
  return this->_tau;
}

// Getter: _output[0]
float LPFilter::getLastOutput()
{
  return this->_outputs[0];
}

// Setter: _dt
void  LPFilter::setPeriod(float dt)         // dt in seconds
{
  this->_dt = dt;
  this->_updateTF();
}

// Setter: _tau
void  LPFilter::setTimeConstant(float tau)  // tau in seconds
{
  this->_tau = tau;
  this->_updateTF();
}

// Set all inputs and outputs to prefill value (starting value)
void  LPFilter::prefill(float fillval)
{
  this->_inputs[0]  = fillval;
  this->_inputs[1]  = fillval;
  this->_outputs[0] = fillval;
  this->_outputs[1] = fillval;
}

// Clear input and output arrays
void  LPFilter::clear()
{
  this->prefill(0);
}


/////////////////////
// Private Methods //
/////////////////////

// Update _num and _den based on _dt and _tau
void  LPFilter::_updateTF()
{
  if (this->_dt != 0.0f && this->_tau != 0.0f) {
    this->_num[0] = (this->_dt)/(this->_tau);
    this->_num[1] = 0.0f;
    this->_den[0] = 1.0f;
    this->_den[1] = (this->_dt)/(this->_tau) - 1.0f;
    this->_initialized = true;
  } else {
    this->_initialized = false;
  }
}
