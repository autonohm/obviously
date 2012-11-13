/**
* @file  PID_Controller.h
* @autor Christian Pfitzner
* @date  09.11.2012
*/

#include <iostream>
#include <stdio.h>

#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

/**
 * @namespace of obvious library
 */
namespace obvious
{

/**
 * @class PID_Controller
 *
 * This class implements an pid controller. With the standard constructor
 * all neccessary parameters are initialized with zero. To get a p controller
 * just set the p value. To get a pi controller you have to set up p and i
 * value.
 */
class PID_Controller
{
public:
  /**
   * Standard constructor with default values
   */
  PID_Controller(void)
    : _p(0), _i(0), _d(0), _awu(0),
      _isValue(0), _setValue(0), _debug(false),
      _minOutput(-100000), _maxOutput(100000) {};

  /*---------------------------------------------------------------------------
   * Functions to set
   */
  /**
   * Function to set proportional value
   * @param[in]     p_value     proportional value
   */
  void setP(const float& p_value)         {_p   = p_value; }
  /**
   * Function to set integration value
   * @param[in]     i_value     integration value
   */
  void setI(const float& i_value)         {_i   = i_value; }
  /**
   * Function to set derivation value
   * @param[in]         d_value     derivation value
   */
  void setD(const float& d_value)         {_d   = d_value; }
  /**
   * Function to set up anti wind up border for integration controller
   * @param[in]         awu_value   anti wind up value
   */
  void setAWU(const float& awu_value)     {_awu = awu_value; }
  /**
   * Function to set up set value of controller
   * @param[in]     setValue    set value of controller
   */
  void setSetValue(const float& setValue) {_setValue = setValue; }
  /**
   * Function to set up minimal output
   * @param[in]         minOutput   minimal output of controller
   */
  void setMinValue(const float& minOutput){_minOutput = minOutput; }
  /**
   * Function to set up maximum output
   * @param[in]         maxOutput   maximal output of controller
   */
  void setMaxValue(const float& maxOutput){_maxOutput = maxOutput; }
  /**
   * Function set up debug mode
   * @param[in]         debug       activate debug mode
   */
  void setDebug(const bool debug = true){_debug = debug; }

  /*---------------------------------------------------------------------------
   * Functions to get
   */
  /**
   * Function to get p value
   * @return        p value of controller
   */
  float getP()   const       { return _p; }
  /**
   * Function to get i value
   * @return        i value of controller
   */
  float getI()   const       { return _i; }
  /**
   * Function to get d value
   * @return        d value of controller
   */
  float getD()   const       { return _d; }
  /**
   * Function to get anti wind up value
   * @return        anti wind up border
   */
  float getAWU() const       { return _awu; }
  /**
   * Function to get set value
   * @return        set value of controller
   */
  float getSetValue()  const { return _setValue; }
  /**
   * Function to get minimal output of controller
   * @return        minimal border
   */
  float getMinOutput() const { return _minOutput; }
  /**
   * Function to get maximum output of controller
   * @return        maximum border
   */
  float getMaxOutput() const { return _maxOutput; }
  /**
   * Function to calculate the next output of the controller
   * @param[in]       isValue
   * @return output value of contoller
   */
  float controll(const float& isValue);

private:
  float _p;               ///< proportional value of pid controller
  float _i;               ///< integrational value of pid controller
  float _d;               ///< derivation value of pid controller
  float _awu;             ///< anti wind up value

  float _maxOutput;       ///< maximum output of controller
  float _minOutput;       ///< minimal output of controller

  float _setValue;        ///< current set value of controller
  float _isValue;         ///< is value of controller

  bool  _debug;           ///< TRUE for output of debug messages to terminal
};

} // end namespace obvious

#endif /* __PIDCONTROLLER__ */

