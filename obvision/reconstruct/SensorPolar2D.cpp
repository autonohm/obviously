#include "SensorPolar2D.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar2D::SensorPolar2D(unsigned int size, double angularRes, double phiMin) : Sensor(2)
{
  _Pose = new Matrix(3, 3);
  _Pose->setIdentity();

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
  _phiUpperBound = 2.0*M_PI + _phiLowerBound;

  if(_phiMin>=180.0)
  {
    LOGMSG(DBG_ERROR, "Valid minimal angle < 180 degree");
  }
}

SensorPolar2D::~SensorPolar2D()
{
  delete _Pose;
  delete [] _data;
  delete [] _mask;
}

void SensorPolar2D::calcRay(unsigned int beam, double ray[2])
{
  Matrix Rh(3, 1);
  double phi = _phiMin + ((double)beam) * _angularRes;
  Rh[0][0] = cos(phi);
  Rh[1][0] = sin(phi);
  Rh[2][0] = 0.0;
  Rh = (*_Pose) * Rh;
  ray[0] = Rh[0][0];
  ray[1] = Rh[1][0];
}

int SensorPolar2D::backProject(double data[2])
{
  Matrix xh(3, 1);
  xh[0][0] = data[0];
  xh[1][0] = data[1];
  xh[2][0] = 1.0;
  Matrix PoseInv = (*_Pose);
  PoseInv.invert();
  xh = PoseInv * xh;

  double phi = atan2(xh[1][0], xh[0][0]);
  return phi2Index(phi);
}

void SensorPolar2D::backProject(Matrix* M, int* indices)
{
  Timer t;
  Matrix PoseInv = (*_Pose);
  PoseInv.invert();
  Matrix coords2D(3, M->getRows());
  Matrix Mt = M->getTranspose();
  coords2D = PoseInv * Mt;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    indices[i] = phi2Index(atan2(coords2D[1][i], coords2D[0][i]));
  }
}

int SensorPolar2D::phi2Index(double phi)
{
  // ensure angle to lie in valid bounds
  if(phi<=_phiLowerBound) return -1;
  if(phi>=_phiUpperBound) phi -= 2.0*M_PI;

  double phiAligned = phi-_phiMin;

  int index = round(phiAligned /_angularRes);

  if(index >= (int)_size) index = -1;

  return index;
}

double SensorPolar2D::angularRes(void)
{
  return(_angularRes);
}

double SensorPolar2D::phiMin(void)
{
  return(_phiMin);
}
}
