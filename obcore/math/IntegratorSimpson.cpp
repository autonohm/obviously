/*
 * IntegratorSimpson.cpp
 *
 *  Created on: 04.09.2013
 *      Author: chris
 */

#include "IntegratorSimpson.h"
#include "stdio.h"
#include <iostream>
#include <cmath>

using namespace obvious;

IntegratorSimpson::IntegratorSimpson(void)
{
  _upperLimit = NAN;
  _lowerLimit = -NAN;
  _T_n = 1.0;
  _e_n = _e_n_1 = _e_n_2 = 0.0;
  _y_n = _y_n_1 = _y_n_2 = 0.0;
}

IntegratorSimpson::~IntegratorSimpson(void)
{

}

void IntegratorSimpson::setRestTime(const double& T_n)
{
  _T_n = T_n;
}

void IntegratorSimpson::setAntiWindup(const double& upperLimit, const double& lowerLimit)
{
  _upperLimit = upperLimit;
  _lowerLimit = lowerLimit;
}

const double& IntegratorSimpson::integrate(const double& error, const double& deltaT)
{
  _e_n_2 = _e_n_1;
  _e_n_1 = _e_n;
  _e_n   = error;

  _y_n_2 = _y_n_1;
  _y_n_1 = _y_n;
  _y_n   = _y_n_1 + (1.0/_T_n)*(_e_n/6.0 + 4.0*_e_n_1/6.0 + _e_n_2/6.0) * deltaT;

  return(_y_n);
}

void IntegratorSimpson::resetIntegrator(const double& setValue)
{
  _y_n = setValue;
}

