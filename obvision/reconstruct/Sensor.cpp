#ifndef SENSOR_H
#define SENSOR_H

#include "obcore/math/Matrix.h"
#include "Sensor.h"

namespace obvious
{

Sensor::Sensor()
{

}

Sensor::~Sensor()
{

}

void Sensor::transform(Matrix* T)
{
  (*_Pose) *= (*T);
}

Matrix* Sensor::getPose()
{
  return _Pose;
}

}

#endif
