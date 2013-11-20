#include "ProjectionFilter.h"

#include "obcore/base/System.h"
#include "obcore/base/tools.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

namespace obvious
{

ProjectionFilter::ProjectionFilter(double* P, unsigned int width, unsigned int height)
{
  memcpy(_P, P, 12*sizeof(*_P));
  _w       = width;
  _h       = height;
  _faultRemovement = false;
  _zFar    = 0.0;

  System<unsigned char>::allocate(_h, _w, _mask);
  System<double>::allocate(_h, _w, _zbuffer);
  memset(_mask[0], 0, _h*_w*sizeof(**_mask));
  memset(_zbuffer[0], 0, _h*_w*sizeof(**_zbuffer));
};

void ProjectionFilter::setModel(CartesianCloud3D* cloud)
{
  unsigned char* proj = new unsigned char[3*_w*_h];
  unsigned char* msk = new unsigned char[_w*_h];
  double* zbuffer = new double[_w*_h];
  Matrix M(3, 4);
  M.setData(_P);

  cloud->createProjection(proj, msk, &M, _w, _h);
  cloud->createZBuffer(proj, zbuffer, &M, _w, _h);

  memcpy(_mask[0], msk, _w*_h*sizeof(*msk));
  memcpy(_zbuffer[0], zbuffer, _w*_h*sizeof(*zbuffer));

  double zmax = 0.0;
  for(unsigned int i=0; i<_w*_h; i++)
    if(zbuffer[i]>zmax) zmax = zbuffer[i];
  _zFar = zmax;

#ifndef NDEBUG
  serializePPM("/tmp/proj.ppm", proj, 640, 480);
  serializePBM("/tmp/msk.pbm", msk, 640, 480);
#endif

  delete [] proj;
  delete [] msk;
  delete [] zbuffer;
}

ProjectionFilter::~ProjectionFilter()
{
  System<unsigned char>::deallocate(_mask);
  System<double>::deallocate(_zbuffer);
};

void ProjectionFilter::update(CartesianCloud3D* cloud, double* P)
{
  memcpy(_P, P, 12*sizeof(*_P));
  setModel(cloud);
}

void ProjectionFilter::update(double* P, unsigned char* mask, double* zbuffer)
{
  memcpy(_P, P, 12*sizeof(*_P));
  memcpy(_mask[0], mask, _w*_h*sizeof(*mask));
  memcpy(_zbuffer[0], zbuffer, _w*_h*sizeof(*zbuffer));
  double zmax = 0.0;
  for(unsigned int i=0; i<_w*_h; i++)
    if(zbuffer[i]>zmax) zmax = zbuffer[i];
  _zFar = zmax;
}

void ProjectionFilter::enableFaultRemovement()
{
  _faultRemovement = true;
}

void ProjectionFilter::disableFaultRemovement()
{
  _faultRemovement = false;
}

void ProjectionFilter::filter(double** scene, unsigned int size, bool* mask)
{
  if(!_active) return;

  unsigned int cnt = 0;

  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i] == 0) continue;
    if(scene[i][2]>=_zFar)
    {
      mask[i] = 0;
      continue;
    }
    double dw = _P[8] * scene[i][0] + _P[9] * scene[i][1] + _P[10] * scene[i][2] + _P[11];
    if(dw>1e-10)
    {
      mask[i] = 0;
      double du = (_P[0] * scene[i][0] + _P[1] * scene[i][1] + _P[2] * scene[i][2] + _P[3]) / dw;
      double dv = (_P[4] * scene[i][0] + _P[5] * scene[i][1] + _P[6] * scene[i][2] + _P[7]) / dw;
      int u = (int)(du + 0.5);
      int v = _h-1-(int)(dv + 0.5);
      if((u>=0) && (u<(int)_w) && (v>=0) && (v<(int)_h))
      {
        if(_faultRemovement)
          mask[i] = (_zbuffer[v][u] > 10e-6);
        else
        {
          mask[i] = 1;
          cnt++;
        }
      }
    }
    else
      mask[i] = 0;
  }
}

}
