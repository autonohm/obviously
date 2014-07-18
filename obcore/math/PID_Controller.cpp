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
{
  _p         = 0;
  _i         = 0;
  _d         = 0;
  _awu       = 0;
  _maxOutput = -1e6;
  _minOutput = 1e6;
  _setValue  = 0;
  _isValue   = 0;
  _debug     = false;

  _timer.start();
}

double PID_Controller::control(const double& isValue)
{
  static double oldError = 0.0;

  // determine deltaT in seconds
  const double deltaT = _timer.reset();

  _isValue = isValue;

  double error = _setValue - _isValue;

  double p_ctrl_output = error * _p;
  double i_ctrl_output = _i * _integrator.integrate(error, deltaT);

  // ensure numerical stability
  if(deltaT<1e-6)
  {
    return (p_ctrl_output + i_ctrl_output);
  }

  double d_ctrl_output = (error - oldError) / deltaT * _d;

  double ctrl_output = p_ctrl_output + i_ctrl_output + d_ctrl_output;

  // check limits
  if (ctrl_output >= _maxOutput)
    ctrl_output = _maxOutput;
  if (ctrl_output <= _minOutput)
    ctrl_output = _minOutput;

  if (_debug)
  {
    std::cout << "Error : "   << error        << std::endl;
    std::cout << "p value: " << p_ctrl_output << std::endl;
    std::cout << "i value: " << i_ctrl_output << std::endl;
    std::cout << "d value: " << d_ctrl_output << std::endl;
    std::cout << "Output: " << ctrl_output      << std::endl;
  }
  return (ctrl_output);
}






