#ifndef SENSOR_H
#define SENSOR_H

#include "obcore/math/Matrix.h"
#include "obcore/base/Logger.h"
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
  _accuracy = NULL;
}

Sensor::~Sensor()
{
  delete _Pose;
  if(_rgb) delete [] _rgb;
  if(_accuracy) delete [] _accuracy;
}

unsigned int Sensor::getWidth()
{
  return _width;
}

unsigned int Sensor::getHeight()
{
  if(_dim<3)
  {
    LOGMSG(DBG_ERROR, "Sensor does not provide a two-dimensional measurement array");
    return 0;
  }
  return _height;
}

void Sensor::transform(Matrix* T)
{
  (*_Pose) *= (*T);
}

void Sensor::translate(double* tr)
{
  (*_Pose)[0][_dim] += tr[0];
  (*_Pose)[1][_dim] += tr[1];
  if(_dim==3)
    (*_Pose)[2][_dim] += tr[2];
}

void Sensor::setPose(Matrix* T)
{
  (*_Pose) = (*T);
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

void Sensor::setRealMeasurementData(vector<float> data, float scale)
{
  if(data.size()!=_size)
  {
    LOGMSG(DBG_WARN, "Size of measurement array wrong, expected " << _size << " obtained: " << data.size());
  }

  for(unsigned int i=0; i<data.size(); i++)
    _data[i] = data[i] * scale;
}

double* Sensor::getRealMeasurementData()
{
  return _data;
}

void Sensor::setRealMeasurementAccuracy(double* accuracy)
{
  if(!_accuracy) _accuracy = new double[_size];
  memcpy(_accuracy, accuracy, _size*sizeof(*accuracy));
}

double* Sensor::getRealMeasurementAccuracy()
{
  return _accuracy;
}

bool Sensor::hasRealMeasurmentAccuracy()
{
  return (_accuracy!=NULL);
}

void Sensor::setRealMeasurementMask(bool* mask)
{
  memcpy(_mask, mask, _size*sizeof(*mask));
}

void Sensor::setRealMeasurementMask(vector<unsigned char> mask)
{
  for(unsigned int i=0; i<mask.size(); i++)
    _mask[i] = mask[i];
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

void Sensor::calcRayFromCurrentPose(unsigned int u, unsigned int v, double ray[3])
{
  LOGMSG(DBG_WARN, "Method for determining measurement rays are not overwritten");
}

}

#endif
