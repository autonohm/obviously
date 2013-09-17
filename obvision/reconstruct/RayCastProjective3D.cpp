#include "RayCastProjective3D.h"

#include <string.h>

#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCastProjective3D::RayCastProjective3D(const unsigned int cols, const unsigned int rows, SensorProjective3D* sensor, TsdSpace* space) : RayCast3D(space)
{
  _cols = cols;
  _rows = rows;
  _sensor = sensor;
}

RayCastProjective3D::~RayCastProjective3D()
{

}

void RayCastProjective3D::calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* ctr, unsigned int subsampling)
{
  Timer t;
  *ctr = 0;

  Matrix* T = _sensor->getPose();
  Matrix Tinv(4, 4);
  Tinv = T->getInverse();

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[_rows*_cols*3];
    double* n_tmp            = new double[_rows*_cols*3];
    unsigned char* color_tmp = new unsigned char[_rows*_cols*3];
    unsigned int cnt_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for (unsigned int row = 0; row < _rows; row+=subsampling)
    {
      for (unsigned int col = 0; col < _cols; col+=subsampling)
      {
        double ray[3];
        _sensor->calcRayFromCurrentPose(row, col, ray);
        if(rayCastFromSensorPose(ray, c, n, color, &depth, _sensor)) // Ray returned with coordinates
        {
          M[0][0] = c[0];
          M[1][0] = c[1];
          M[2][0] = c[2];
          N[0][0] = n[0];
          N[1][0] = n[1];
          N[2][0] = n[2];
          M       = Tinv * M;
          N       = Tinv * N;
          for (unsigned int i = 0; i < 3; i++)
          {
            c_tmp[cnt_tmp]      = M[i][0];
            color_tmp[cnt_tmp]  = color[i];
            n_tmp[cnt_tmp++]    = N[i][0];
          }
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[*ctr],  c_tmp,     cnt_tmp*sizeof(double));
      memcpy(&normals[*ctr], n_tmp,     cnt_tmp*sizeof(double));
      memcpy(&rgb[*ctr],     color_tmp, cnt_tmp*sizeof(unsigned char));
      *ctr += cnt_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    delete[] color_tmp;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *ctr << " coordinates");
}

void RayCastProjective3D::calcCoordsFromCurrentViewMask(double* coords, double* normals, unsigned char* rgb, bool* mask)
{
  Timer t;

  Matrix* T = _sensor->getPose();
  Matrix Tinv(4, 4);
  Tinv = T->getInverse();
  unsigned int ctr = 0;

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[_rows*_cols*3];
    double* n_tmp            = new double[_rows*_cols*3];
    unsigned char* color_tmp = new unsigned char[_rows*_cols*3];
    bool* mask_tmp           = new bool[_rows*_cols];
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals
    unsigned int cnt_tmp = 0;

#pragma omp for schedule(dynamic)
    for (unsigned int row = 0; row < _rows; row++)
    {
      for (unsigned int col = 0; col < _cols; col++)
      {
        double ray[3];
        _sensor->calcRayFromCurrentPose(row, col, ray);
        if(rayCastFromSensorPose(ray, c, n, color, &depth, _sensor)) // Ray returned with coordinates
        {
          M[0][0] = c[0];
          M[1][0] = c[1];
          M[2][0] = c[2];
          N[0][0] = n[0];
          N[1][0] = n[1];
          N[2][0] = n[2];
          M       = Tinv * M;
          N       = Tinv * N;
          mask_tmp[cnt_tmp/3] = true;
          for (unsigned int i = 0; i < 3; i++)
          {
            c_tmp[cnt_tmp]      = M[i][0];
            color_tmp[cnt_tmp]  = color[i];
            n_tmp[cnt_tmp++]    = N[i][0];
          }
        }
        else
        {
          mask_tmp[cnt_tmp/3] = false;
          for (unsigned int i = 0; i < 3; i++)
          {
            c_tmp[cnt_tmp]      = 0;
            color_tmp[cnt_tmp]  = 0;
            n_tmp[cnt_tmp++]    = 0;
          }
        }

      }
    }
#pragma omp critical
    {
      memcpy(&coords[ctr],  c_tmp,     cnt_tmp*sizeof(double));
      memcpy(&normals[ctr], n_tmp,     cnt_tmp*sizeof(double));
      memcpy(&rgb[ctr],     color_tmp, cnt_tmp*sizeof(unsigned char));
      memcpy(&mask[ctr/3],  mask_tmp,  cnt_tmp/3*sizeof(bool));
      ctr += cnt_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    delete[] color_tmp;
    delete[] mask_tmp;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << ctr << " coordinates");
}

}
