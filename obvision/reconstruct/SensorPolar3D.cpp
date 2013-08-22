#include "SensorPolar3D.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar3D::SensorPolar3D(unsigned int beams, double thetaRes, double thetaMin) : Sensor(3)
{
  _Pose = new Matrix(4, 4);
  _Pose->setIdentity();

  _beams = beams;
  _size = beams * 1000;
  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  _thetaRes = thetaRes;
  _thetaMin = thetaMin;

  // smallest angle that lies in bounds (must be negative)
  _thetaLowerBound = -0.5*_thetaRes + _thetaMin;

  // if angle is too large, it might be projected with modulo 2 PI to a valid index
  _thetaUpperBound = 2.0*M_PI + _thetaLowerBound;

  if(_thetaMin>=M_PI)
  {
    LOGMSG(DBG_ERROR, "Valid minimal angle < M_PI");
  }
}

SensorPolar3D::~SensorPolar3D()
{
  delete _Pose;
  delete [] _data;
  delete [] _mask;
}

void SensorPolar3D::calcRay(unsigned int beam, unsigned int plane, double ray[3])
{
  Matrix Rh(4, 1);
  double theta = _thetaMin + ((double)beam) * _thetaRes;
  double x = sin(theta);
  double z = cos(theta);

  double phi = ((double)plane) / ((double)_planes) * M_PI - M_PI;

  Rh[0][0] = cos(phi) * x;
  Rh[1][0] = sin(phi) * x;
  Rh[2][0] = z;
  Rh[3][0] = 0.0;
  Rh = (*_Pose) * Rh;
  ray[0] = Rh[0][0];
  ray[1] = Rh[1][0];
  ray[2] = Rh[2][0];
}

void SensorPolar3D::setPlanes(unsigned int planes)
{
  _planes = planes;
  cout << _planes << endl;
}

void SensorPolar3D::backProject(Matrix* M, int* indices)
{
  Timer t;
  Matrix PoseInv = (*_Pose);
  PoseInv.invert();
  PoseInv.print();
  gsl_matrix* coords3D = gsl_matrix_alloc(4, M->getRows());

  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, PoseInv.getBuffer(), M->getBuffer(), 0.0, coords3D);

  double* x = gsl_matrix_ptr(coords3D, 0, 0);
  double* y = gsl_matrix_ptr(coords3D, 1, 0);
  double* z = gsl_matrix_ptr(coords3D, 2, 0);

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    double phi = atan2(*(y+i), *(x+i)) - M_PI;
    if(phi>M_PI) phi -= M_PI;
    if(phi<-M_PI) phi += M_PI;
    //double r2 = sqrt(*(x+i) * *(x+i) + *(y+i) * *(y+i));
    //double phi = -M_PI + acos(*(x+i) / r2);


    double r = sqrt(*(x+i) * *(x+i) + *(y+i) * *(y+i) + *(z+i) * *(z+i));
    double theta = acos(*(z+i) / r);
    if(*(y+i)>0)
      theta = -theta;

    //if(i%24==0) cout << *(x+i) << " " << *(y+i) << " " << *(z+i) << " " << r << " " << theta*180/M_PI << endl;


    indices[i] = thetaPhi2Index(theta, phi);
  }

  //exit(1);
  gsl_matrix_free(coords3D);
}

int SensorPolar3D::thetaPhi2Index(double theta, double phi)
{
  double thetaAligned = theta-_thetaMin;

  if(phi<-M_PI || phi>0)
    cout << "Warning: wrong phi " << phi << endl;

  if(thetaAligned<0 || thetaAligned>(1.5*M_PI))
    return -1;

  //if(phi>(-M_PI+0.1*M_PI)) return -1;

  int scan = (int)((M_PI+phi) / M_PI * (double)_planes);
  int index = round(thetaAligned /_thetaRes) +  scan * _beams;

  //cout << index  << " " << thetaAligned << " " << _thetaRes << endl;
  //if(index >= (int)100000) index = -1;

  return index;
}

}
