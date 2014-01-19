#ifndef SENSOR_H
#define SENSOR_H

#include "obcore/base/Logger.h"
#include "Sensor.h"
#include <string.h>

namespace obvious
{

Sensor::Sensor(unsigned int dim, double maxRange, double minRange)
{
  _dim = dim;
  _maxRange = maxRange;
  _minRange = minRange;

  //_Pose = new Matrix(_dim+1, _dim+1);
  //_Pose->setIdentity();

  _Position = new Matrix(_dim+1, 1);

  for(unsigned int i=0; i<_dim; i++)
    (*_Position)(i, 0) = 0.0;
  (*_Position)(_dim, 0) = 1.0;

  _View = new Matrix(_dim, _dim);
  _View->setIdentity();

  _rgb = NULL;
  _accuracy = NULL;

  _rayNorm = 1.0;
}

Sensor::~Sensor()
{
  //delete _Pose;
  delete _Position;
  delete _View;
  if(_rgb) delete [] _rgb;
  if(_accuracy) delete [] _accuracy;
}

Matrix* Sensor::getNormalizedRayMap(double norm)
{
  if(norm != _rayNorm)
  {
    for(unsigned int i=0; i<_size; i++)
    {
      for(unsigned int j=0; j<_dim; j++)
        (*_rays)(j, i) *= (norm/_rayNorm);
    }
    _rayNorm = norm;
  }
  return _rays;
}

void Sensor::transform(Matrix* T)
{
  //(*_Pose) = (*_Pose) * (*T);

  Matrix R(*T, 0, 0, _dim, _dim);

  //(*_Position) = (*T) * (*_Position);
  for(unsigned int r=0; r<_dim; r++)
    (*_Position)(r, 0) += (*T)(r, _dim);
  (*_View) = R * (*_View);
  (*_rays) = R * (*_rays);
}

void Sensor::translate(double* tr)
{
  //Matrix T(_dim+1, _dim+1);
  //T.setIdentity();
  for(unsigned int r=0; r<_dim; r++)
    (*_Position)(r, 0) += tr[r];
    //T(r, _dim) = tr[r];
  //(*_Pose) = T * (*_Pose);
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

double Sensor::getMaximumRange()
{
  return _maxRange;
}

double Sensor::getMinimumRange()
{
  return _minRange;
}


Matrix Sensor::getTransformation()
{
  Matrix T(_dim+1, _dim+1);
  T.setIdentity();
  for(unsigned int r=0; r<_dim; r++)
  {
    for(unsigned int c=0; c<_dim; c++)
      T(r, c) = (*_View)(r, c);
    T(r, _dim) = (*_Position)(r, 0);
  }
  return T;
}

void Sensor::resetTransformation()
{
  for(unsigned int i=0; i<_dim; i++)
    (*_Position)(i, 0) = 0.0;
  _View->setIdentity();
}

/*void Sensor::setPose(Matrix* T)
{
  _Pose->setIdentity();
  Matrix Pinv = (*_Pose);
  Pinv.invert();

  // 1) set pose to identity
  // 2) rotate rays back
  transform(&Pinv);

  transform(T);
}*/

/*Matrix* Sensor::getPose()
{
  return _Pose;
}*/

void Sensor::getPosition(double* tr)
{
  for(unsigned int i=0; i<_dim; i++)
    //tr[i] = (*_Pose)(i, _dim);
    tr[i] = (*_Position)(i, 0);
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

}

#endif
