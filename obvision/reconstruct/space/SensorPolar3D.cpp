#include "SensorPolar3D.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"
#include "obcore/base/tools.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar3D::SensorPolar3D(unsigned int beams, double thetaRes, double thetaMin, double phiRes, double maxRange, double minRange) : Sensor(3, maxRange, minRange)
{
  _thetaRes = thetaRes;
  _thetaMin = thetaMin;
  _phiRes = phiRes;

  _width = beams;
  _height = M_PI/_phiRes;

  _size = _width*_height;
  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  if(_thetaMin>=M_PI)
  {
    LOGMSG(DBG_ERROR, "Valid minimal angle < M_PI");
  }

  System<double>::allocate(_height, _width, _distanceMap);
  System<int>::allocate(_height, _width, _indexMap);

  unsigned int i=0;
  _rays = new Matrix(3, _size);
  for(unsigned int p=0; p<_height; p++)
  {
    for(unsigned int b=0; b<beams; b++, i++)
    {
      Matrix ray(3, 1);
      double theta = _thetaMin + ((double)b) * _thetaRes;
      double x = sin(theta);
      double z = cos(theta);

      double phi = ((double)p) / ((double)_height) * M_PI - M_PI;

      ray(0,0) = cos(phi) * x;
      ray(1,0) = sin(phi) * x;
      ray(2,0) = z;
      double len = sqrt(ray(0,0)*ray(0,0)+ray(1,0)*ray(1,0)+ray(2,0)*ray(2,0));
      ray = (*_View) * ray;

      (*_rays)(0, i) = ray(0, 0) / len;
      (*_rays)(1, i) = ray(1, 0) / len;
      (*_rays)(2, i) = ray(2, 0) / len;
    }
  }
}

SensorPolar3D::~SensorPolar3D()
{
  delete [] _data;
  delete [] _mask;
  System<double>::deallocate(_distanceMap);
  System<int>::deallocate(_indexMap);

  delete _rays;
}

/*void SensorPolar3D::calcRayFromCurrentPose(unsigned int beam, unsigned int plane, double ray[3])
{
  Matrix Rh(4, 1);
  double theta = _thetaMin + ((double)beam) * _thetaRes;
  double x = sin(theta);
  double z = cos(theta);

  double phi = ((double)plane) / ((double)_width) * M_PI - M_PI;

  Rh(0,0) = cos(phi) * x;
  Rh(1,0) = sin(phi) * x;
  Rh(2,0) = z;
  Rh(3,0) = 0.0;
  Rh = (*_Pose) * Rh;
  ray[0] = Rh(0,0);
  ray[1] = Rh(1,0);
  ray[2] = Rh(2,0);
}*/

void SensorPolar3D::setDistanceMap(vector<float> phi, vector<float> dist)
{
  LOGMSG(DBG_DEBUG, "SensorPolar3D::setDistanceMap");

  if((_width*phi.size()) != dist.size())
  {
    LOGMSG(DBG_WARN, "SensorPolar3D::setDistanceMap: invalid size of vectors ... skipping");
    return;
  }

  double phi_corr = (M_PI / (double)phi.size() / _width) * 270.0/360.0;
  for(unsigned int i=0; i<_width*_height; i++)
  {
    _distanceMap[0][i] = -1.0;
    _indexMap[0][i] = -1;
  }

  for(unsigned int r=0; r<phi.size(); r++)
  {
    unsigned int rp = phi[r] / _phiRes;
    for(unsigned int c=0; c<_width; c++)
    {
      unsigned int rpr = rp + (unsigned int)(phi_corr/_phiRes * (double)c);
      if(rpr>=_height) continue;
      _distanceMap[rpr][c] = dist[r*_width+c];
      _indexMap[rpr][c] = r*_width+c;
    }
  }

  /*char filename[128];
  static int cnt = 0;
  sprintf(filename, "/tmp/map%05d.pbm", cnt++);
  unsigned char* map = new unsigned char[_width*_height];
  for(unsigned int r=0; r<_height; r++)
    for(unsigned int c=0; c<_width; c++)
      map[r*_width+c] = (_indexMap[r][c]!=-1 ? 0 : 255);
  serializePBM(filename, map, _width, _height);
  delete [] map;*/
}

void SensorPolar3D::backProject(Matrix* M, int* indices, Matrix* T)
{
  Timer t;
  //Matrix PoseInv = (*_Pose);
  Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  Matrix coords3D = Matrix::multiply(PoseInv, *M, false, true);

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    double phi = atan2(coords3D(2,i), coords3D(0,i)) - M_PI;
    if(phi>M_PI) phi -= M_PI;
    if(phi<-M_PI) phi += M_PI;

    double r = sqrt(coords3D(0,i) * coords3D(0,i) + coords3D(1,i) * coords3D(1,i) + coords3D(2,i) * coords3D(2,i));
    double theta = acos(coords3D(1,i) / r);
    if(coords3D(2,i)>0)
      theta = -theta;

    double t = theta-_thetaMin;
    if(t>0)
    {
      unsigned int c = round(t / _thetaRes);
      if(c<_width)
      {
        unsigned int r = (unsigned int)((M_PI+phi) / M_PI * (double)_height);
        indices[i] = _indexMap[r][c];
      }
      else
        indices[i] = -1;
    }
    else
    {
      //cout << "alert: " << t << " " << coords3D(0,i) << " " << coords3D(1,i) << " " << coords3D(2,i) << endl;
      //abort();
      indices[i] = -1;
    }
  }
}

}
