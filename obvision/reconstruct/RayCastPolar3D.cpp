#include "RayCastPolar3D.h"

#include <string.h>

#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCastPolar3D::RayCastPolar3D(SensorPolar3D* sensor, TsdSpace* space) : RayCast3D(space)
{
  _sensor = sensor;
}

RayCastPolar3D::~RayCastPolar3D()
{

}

void RayCastPolar3D::calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* size)
{
  Timer t;
  *size = 0;

  Matrix* T = _sensor->getPose();

  Matrix Tinv(4, 4);
  Tinv = T->getInverse();

  unsigned int beams = _sensor->getBeams();
  unsigned int planes = _sensor->getPlanes();

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[beams*planes*3];
    double* n_tmp            = new double[beams*planes*3];
    unsigned char* color_tmp = new unsigned char[beams*planes*3];
    unsigned int size_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for (unsigned int beam = 0; beam < beams; beam++)
    {
      for (unsigned int plane = 0; plane < planes; plane++)
      {
        double ray[3];
        _sensor->calcRayFromCurrentPose(beam, plane, ray);

        ray[0] *= _space->getVoxelSize();
        ray[1] *= _space->getVoxelSize();
        ray[2] *= _space->getVoxelSize();

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
            c_tmp[size_tmp]      = M[i][0];
            color_tmp[size_tmp]  = color[i];
            n_tmp[size_tmp++]    = N[i][0];
          }
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[*size],  c_tmp,     size_tmp*sizeof(double));
      memcpy(&normals[*size], n_tmp,     size_tmp*sizeof(double));
      memcpy(&rgb[*size],     color_tmp, size_tmp*sizeof(unsigned char));
      *size += size_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    delete[] color_tmp;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *size << " coordinates");
}

}
