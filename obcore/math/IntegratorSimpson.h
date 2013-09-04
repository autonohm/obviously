/*
 * IntegratorSimpson.h
 *
 *  Created on: 04.09.2013
 *      Author: chris
 */

#ifndef INTEGRATORSIMPSON_H_
#define INTEGRATORSIMPSON_H_

#include "obcore/base/Timer.h"

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class       IntegratorSimpson
 * @date        2013-09-05
 * @author      Christian Pfitzner
 */
class IntegratorSimpson
{
public:
  /**
   * Standard constructor
   */
  IntegratorSimpson(void);
  /**
   * Default destructor
   */
  ~IntegratorSimpson(void);
  /**
   * Function to set rest time of integrator
   * @param T_n                 rest time in seconds
   */
  void setRestTime(const double& T_n);
  /**
   * Function to set anti wind up rules
   * @param upperLimit          upper limit of output
   * @param lowerLimit          lower limit of output
   */
  void setAntiWindup(const double& upperLimit =  10000,
                     const double& lowerLimit = -10000);
  /**
   * Function to update integrator with a new value
   * @param     y_n             new value
   * @return    output
   */
  const double& integrate(const double& error);
  /**
   * Function to reset integrator
   */
  void resetIntegrator(const double& setValue = 0.0);

private:
  double                _upperLimit;           //!< upper limit of integrator
  double                _lowerLimit;           //!< lower limit of integrator
  double                _T_n;                  //!< rest time

  double                _e_n, _e_n_1, _e_n_2;  //!< error variables
  double                _y_n, _y_n_1, _y_n_2;  //!< output variables

  Timer                 _timer;                //!< timer necessary to get delta t
};

} // namespace


#endif /* INTEGRATORSIMPSON_H_ */
