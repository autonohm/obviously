#include "SensorPolar2D.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar2D::SensorPolar2D(unsigned int size, double angularRes, double minPhi)
{
  _Pose = new Matrix(3, 3);
  _Pose->setIdentity();

  _size = size;
  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  _angularRes = angularRes;
  _minPhi = minPhi;

  if(minPhi>=180.0)
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

void SensorPolar2D::transform(Matrix* T)
{
  (*_Pose) *= (*T);
}

Matrix* SensorPolar2D::getPose()
{
  return _Pose;
}

void SensorPolar2D::getPosition(double tr[2])
{
  tr[0] = (*_Pose)[0][2];
  tr[1] = (*_Pose)[1][2];
}

unsigned int SensorPolar2D::getRealMeasurementSize()
{
  return _size;
}

void SensorPolar2D::setRealMeasurementData(double* data)
{
  memcpy(_data, data, _size*sizeof(*data));
}

double* SensorPolar2D::getRealMeasurementData()
{
  return _data;
}

void SensorPolar2D::setRealMeasurementMask(bool* mask)
{
  memcpy(_mask, mask, _size*sizeof(*mask));
}

bool* SensorPolar2D::getRealMeasurementMask()
{
  return _mask;
}

void SensorPolar2D::calcRay(unsigned int beam, double ray[2])
{
  Matrix Rh(3, 1);
  double phi = _minPhi + ((double)beam) * _angularRes;
  double cphi = cos(phi);
  double sphi = sin(phi);
  Rh[0][0] = cos(phi);
  Rh[1][0] = sin(phi);
  Rh[2][0] = 0.0;
  Rh = (*_Pose) * Rh;
  ray[0] = Rh[0][0];
  ray[1] = Rh[1][0];
}

int SensorPolar2D::backProject(double* data)
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

int SensorPolar2D::phi2Index(double phi)
{
  double phiAligned = phi-_minPhi;

  // ensure angle to lie in valid bounds
  if(phiAligned<=-0.5*_angularRes) return -1;
  if(phiAligned>=(2.0*M_PI-0.5*_angularRes)) phiAligned -= 2.0*M_PI;

  int index = round(phiAligned /_angularRes);

  if(index >= _size) index = -1;

  return index;
}

}
