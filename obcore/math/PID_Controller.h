/**
* @file   PID_Controller.h
* @author Christian Pfitzner
* @date   09.11.2012
*/

#include <iostream>
#include <stdio.h>

#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include "IntegratorSimpson.h"
#include "obcore/base/Timer.h"

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
  PID_Controller(void);

  /**
   * Function to set proportional value
   * @param[in]     p_value     proportional value
   */
  void setP(const double& p_value)         {_p   = p_value; }

  /**
   * Function to set integration value
   * @param[in]     i_value     integration value
   */
  void setI(const double& i_value)         {_i   = i_value; }

  /**
   * Function to set derivation value
   * @param[in]         d_value     derivation value
   */
  void setD(const double& d_value)         {_d   = d_value; }

  /**
   * Function to set up anti wind up border for integration controller
   * @param[in]         awu_value   anti wind up value
   */
  void setAWU(const double& awu_value)     {_awu = awu_value; }

  /**
   * Function to set up set value of controller
   * @param[in]     setValue    set value of controller
   */
  void setSetValue(const double& setValue) {_setValue = setValue; }

  /**
   * Function to set up minimal output
   * @param[in]         minOutput   minimal output of controller
   */
  void setMinValue(const double& minOutput){_minOutput = minOutput; }

  /**
   * Function to set up maximum output
   * @param[in]         maxOutput   maximal output of controller
   */
  void setMaxValue(const double& maxOutput){_maxOutput = maxOutput; }

  /**
   * Function set up debug mode
   * @param[in]         debug       activate debug mode
   */
  void setDebug(const bool debug = true){_debug = debug; }

  /**
   * Function to get p value
   * @return        p value of controller
   */
  double getP()   const       { return _p; }

  /**
   * Function to get i value
   * @return        i value of controller
   */
  double getI()   const       { return _i; }

  /**
   * Function to get d value
   * @return        d value of controller
   */
  double getD()   const       { return _d; }

  /**
   * Function to get anti wind up value
   * @return        anti wind up border
   */
  double getAWU() const       { return _awu; }

  /**
   * Function to get set value
   * @return        set value of controller
   */
  double getSetValue()  const { return _setValue; }

  /**
   * Function to get minimal output of controller
   * @return        minimal border
   */
  double getMinOutput() const { return _minOutput; }

  /**
   * Function to get maximum output of controller
   * @return        maximum border
   */
  double getMaxOutput() const { return _maxOutput; }

  /**
   * Function to calculate the next output of the controller
   * @param[in]       isValue
   * @return output value of controller
   */
  double control(const double& isValue);

private:
  IntegratorSimpson      _integrator;
  double _p;               ///< proportional value of pid controller
  double _i;               ///< integration value of pid controller
  double _d;               ///< derivation value of pid controller
  double _awu;             ///< anti wind up value

  double _maxOutput;       ///< maximum output of controller
  double _minOutput;       ///< minimal output of controller

  double _setValue;        ///< current set value of controller
  double _isValue;         ///< is value of controller

  bool  _debug;           ///< TRUE for output of debug messages to terminal

  Timer                 _timer;                //!< timer necessary to get delta t
};

} // end namespace obvious

#endif /* __PIDCONTROLLER__ */

