#ifndef SENSOR_H
#define SENSOR_H

#include "obcore/math/Matrix.h"
#include "Sensor.h"
#include <string.h>

namespace obvious
{

Sensor::Sensor(unsigned int dim)
{
  _dim = dim;

  _Pose = new Matrix(_dim+1, _dim+1);
  _Pose->setIdentity();

  _rgb = NULL;
}

Sensor::~Sensor()
{
  delete _Pose;
  if(_rgb) delete [] _rgb;
}

void Sensor::transform(Matrix* T)
{
  (*_Pose) *= (*T);
}

Matrix* Sensor::getPose()
{
  return _Pose;
}

void Sensor::getPosition(double* tr)
{
  tr[0] = (*_Pose)[0][_dim];
  tr[1] = (*_Pose)[1][_dim];
  if(_dim==3)
    tr[2] = (*_Pose)[2][_dim];
}

unsigned int Sensor::getRealMeasurementSize()
{
  return _size;
}

void Sensor::setRealMeasurementData(double* data, double scale)
{
  if(scale==1.0)
    memcpy(_data, data, _size*sizeof(*data));
  else
  {
    for(unsigned int i=0; i<_size; i++)
      _data[i] = data[i] * scale;
  }
}

double* Sensor::getRealMeasurementData()
{
  return _data;
}

void Sensor::setRealMeasurementMask(bool* mask)
{
  memcpy(_mask, mask, _size*sizeof(*mask));
}

bool* Sensor::getRealMeasurementMask()
{
  return _mask;
}

bool Sensor::hasRealMeasurmentRGB()
{
  return (_rgb!=NULL);
}

void Sensor::setRealMeasurementRGB(unsigned char* rgb)
{
  if(!_rgb) _rgb = new unsigned char[_size*3];
  memcpy(_rgb, rgb, _size*3*sizeof(*rgb));
}

unsigned char* Sensor::getRealMeasurementRGB()
{
  return _rgb;
}

}

#endif
