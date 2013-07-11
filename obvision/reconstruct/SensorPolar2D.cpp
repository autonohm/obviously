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

  // smallest angle to lie in bounds (must be negative)
  _phiLowerBound = -0.5*_angularRes + _minPhi;

  // if angle is too large, it might be projected with modulo 2 PI to a valid index
  _phiUpperBound = 2.0*M_PI + _phiLowerBound;

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

void SensorPolar2D::backProject(Matrix* M, int* indices)
{
  Timer t;
  Matrix PoseInv = (*_Pose);
  PoseInv.invert();
  gsl_matrix* coords2D = gsl_matrix_alloc(3, M->getRows());

  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, PoseInv.getBuffer(), M->getBuffer(), 0.0, coords2D);

  double* x = gsl_matrix_ptr(coords2D, 0, 0);
  double* y = gsl_matrix_ptr(coords2D, 1, 0);

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    //const double x = gsl_matrix_get(coords2D, 0, i);
    //const double y = gsl_matrix_get(coords2D, 1, i);
    //indices[i] = phi2Index(atan2(y, x));

    indices[i] = phi2Index(atan2(*(y+i), *(x+i)));
  }

  gsl_matrix_free(coords2D);
}

int SensorPolar2D::phi2Index(double phi)
{
  // ensure angle to lie in valid bounds
  if(phi<=_phiLowerBound) return -1;
  if(phi>=_phiUpperBound) phi -= 2.0*M_PI;

  double phiAligned = phi-_minPhi;

  int index = round(phiAligned /_angularRes);

  if(index >= (int)_size) index = -1;

  return index;
}

}
