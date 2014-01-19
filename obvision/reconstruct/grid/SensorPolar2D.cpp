#include "SensorPolar2D.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar2D::SensorPolar2D(unsigned int size, double angularRes, double phiMin, double maxRange, double minRange) : Sensor(2, maxRange, minRange)
{
  _size = size;
  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  _angularRes = angularRes;
  _phiMin = phiMin;

  // smallest angle that lies in bounds (must be negative)
  _phiLowerBound = -0.5*_angularRes + _phiMin;

  // if angle is too large, it might be projected with modulo 2 PI to a valid index
  _phiUpperBound = _phiMin + ((double)size)*_angularRes;

  if(_phiMin>=180.0)
  {
    LOGMSG(DBG_ERROR, "Valid minimal angle < 180 degree");
  }

  _rays = new Matrix(2, _size);

  for(unsigned int i=0; i<_size; i++)
  {
    Matrix Rh(2, 1);
    double phi = _phiMin + ((double)i) * _angularRes;
    Rh(0,0) = cos(phi);
    Rh(1,0) = sin(phi);
    (*_rays)(0, i) = Rh(0,0);
    (*_rays)(1, i) = Rh(1,0);
  }
}

SensorPolar2D::~SensorPolar2D()
{
  delete [] _data;
  delete [] _mask;

  delete _rays;
}

int SensorPolar2D::backProject(double data[2])
{
  Matrix xh(3, 1);
  xh(0,0) = data[0];
  xh(1,0) = data[1];
  xh(2,0) = 1.0;
  Matrix PoseInv = getTransformation();
  //Matrix PoseInv = (*_Pose);
  PoseInv.invert();
  xh = PoseInv * xh;

  double phi = atan2(xh(1,0), xh(0,0));
  // ensure angle to lie in valid bounds
  if(phi<=_phiLowerBound) return -1;
  if(phi>=_phiUpperBound) return -1;
  return round((phi-_phiMin) /_angularRes);
}

void SensorPolar2D::backProject(Matrix* M, int* indices, Matrix* T)
{
  Timer t;
  //Matrix PoseInv = (*_Pose);
  Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  Matrix coords2D = Matrix::multiply(PoseInv, *M, false, true);

  const double angularResInv = 1.0 / _angularRes;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    const double phi = atan2(coords2D(1,i), coords2D(0,i));
    if(phi<=_phiLowerBound) indices[i] = -1;
    else if(phi>=_phiUpperBound) indices[i] = -1;
    else indices[i] = round((phi-_phiMin) * angularResInv);
  }
}

double SensorPolar2D::getAngularResolution(void)
{
  return _angularRes;
}

double SensorPolar2D::getPhiMin(void)
{
  return _phiMin;
}
}
