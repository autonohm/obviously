/**
* @file     PID_Controller.cpp
* @author   Christian Pfitzner
* @date     09.11.2012
*
*
*/

#include "PID_Controller.h"
using namespace obvious;

PID_Controller::PID_Controller(void)
: _p(0), _i(0), _d(0), _awu(0),
  _maxOutput(-100000), _minOutput(100000),
  _setValue(0), _isValue(0), _debug(false)
{

}

float PID_Controller::controll(const float& isValue)
{
  static float oldError;
  static float i_ctrl_output;

  _isValue = isValue;

  float error = _setValue - _isValue;

  // calculate single controller values
  float  p_ctrl_output  = error * _p;
  float  d_ctrl_output  = (error - oldError) * _d;
  i_ctrl_output         = _integrator.integrate(error);

  // calculate controller value
  _setValue = p_ctrl_output + i_ctrl_output + d_ctrl_output;

  // check borders
  if (_setValue >= _maxOutput)
    _setValue = _maxOutput;
  if (_setValue <= _minOutput)
    _setValue = _minOutput;

  if (_debug)
  {
    std::cout << "Error : "   << error         << std::endl;
    std::cout << "p value: " << p_ctrl_output << std::endl;
    std::cout << "i value: " << i_ctrl_output << std::endl;
    std::cout << "d value: " << d_ctrl_output << std::endl;
    std::cout << "Output: " << _setValue      << std::endl;
  }
  return (_setValue);
}






