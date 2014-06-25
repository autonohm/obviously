#include "RayCast3D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

// Beware: this will slow down the application
#define PRINTSTATISTICS 0

/**
 *  Variables for statistics
 */
unsigned int _skipped = 0;

unsigned int _traversed = 0;

RayCast3D::RayCast3D()
{
  _xmin   = NAN;
  _ymin   = NAN;
  _zmin   = NAN;

  _xmax   = NAN;
  _ymax   = NAN;
  _zmax   = NAN;
}

RayCast3D::~RayCast3D()
{

}

void RayCast3D::calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size)
{
  Timer t;
  *size = 0;

  Matrix Tinv = sensor->getTransformation();
  Tinv.invert();

  double tr[3];
  sensor->getPosition(tr);

  Matrix* R = sensor->getNormalizedRayMap(space->getVoxelSize());
  unsigned int count = sensor->getWidth() * sensor->getHeight();

  if(space->isInsideSpace(sensor))
  {
    _xmin   = 0.0;
    _ymin   = 0.0;
    _zmin   = 0.0;

    _xmax   = 10e9;
    _ymax   = 10e9;
    _zmax   = 10e9;
  }
  else
  {
    // prevent rays to be casted parallel to a plane outside of space
    // if we are outside, we might loose some reprojections
    _xmin   = 10e9;
    _ymin   = 10e9;
    _zmin   = 10e9;

    _xmax   = 0.0;
    _ymax   = 0.0;
    _zmax   = 0.0;
  }

#if PRINTSTATISTICS
  _skipped = 0;
  _traversed = 0;
#endif

  _idxMin = sensor->getMinimumRange() / space->getVoxelSize();
  _idxMax = sensor->getMaximumRange() / space->getVoxelSize();


#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[count*3];
    double* n_tmp            = new double[count*3];
    //unsigned char* color_tmp = new unsigned char[count*3];
    unsigned int size_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M(3,0) = 1.0;
    N(3,0) = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for(unsigned int i=0; i<count; i++)
    {
      double ray[3];
      ray[0] = (*R)(0, i);
      ray[1] = (*R)(1, i);
      ray[2] = (*R)(2, i);

      // Raycast returns with coordinates in world coordinate system
      if(rayCastFromSensorPose(space, tr, ray, c, n, color, &depth)) // Ray returned with coordinates
      {
        M(0,0) = c[0];
        M(1,0) = c[1];
        M(2,0) = c[2];
        N(0,0) = n[0];
        N(1,0) = n[1];
        N(2,0) = n[2];
        // Transform data to sensor coordinate system
        M = Tinv * M;
        N = Tinv * N;
        for (unsigned int i = 0; i < 3; i++)
        {
          c_tmp[size_tmp]      = M(i,0);
          //color_tmp[size_tmp]  = color[i];
          n_tmp[size_tmp++]    = N(i,0);
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[*size],  c_tmp,     size_tmp*sizeof(double));
      memcpy(&normals[*size], n_tmp,     size_tmp*sizeof(double));
      //memcpy(&rgb[*size],     color_tmp, size_tmp*sizeof(unsigned char));
      *size += size_tmp;
    }
    delete[] c_tmp;     c_tmp = NULL;
    delete[] n_tmp;     n_tmp = NULL;
   // delete[] color_tmp; color_tmp = NULL;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *size << " coordinates");

#if PRINTSTATISTICS
  LOGMSG(DBG_DEBUG, "Traversed: " << _traversed << ", skipped: " << _skipped);
#endif
}

void RayCast3D::calcCoordsFromCurrentPoseMask(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, bool* mask, unsigned int* size)
{
  Timer t;

  Matrix Tinv = sensor->getTransformation();
  Tinv.invert();

  double tr[3];
  sensor->getPosition(tr);
  unsigned int ctr = 0;

  unsigned int width = sensor->getWidth();
  unsigned int height = sensor->getHeight();

  Matrix* R = sensor->getNormalizedRayMap(space->getVoxelSize());
  unsigned int count = sensor->getWidth() * sensor->getHeight();

  if(space->isInsideSpace(sensor))
  {
    _xmin   = 0.0;
    _ymin   = 0.0;
    _zmin   = 0.0;

    _xmax   = 10e9;
    _ymax   = 10e9;
    _zmax   = 10e9;
  }
  else
  {
    // prevent rays to be casted parallel to a plane outside of space
    // if we are outside, we might loose some reprojections
    _xmin   = 10e9;
    _ymin   = 10e9;
    _zmin   = 10e9;

    _xmax   = 0.0;
    _ymax   = 0.0;
    _zmax   = 0.0;
  }

#if PRINTSTATISTICS
  _skipped = 0;
  _traversed = 0;
#endif

  _idxMin = sensor->getMinimumRange() / space->getVoxelSize();
  _idxMax = sensor->getMaximumRange() / space->getVoxelSize();

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[width*height*3];
    double* n_tmp            = new double[width*height*3];
    //unsigned char* color_tmp = new unsigned char[width*height*3];
    bool* mask_tmp           = new bool[width*height];
    Matrix M(4,1);
    Matrix N(4,1);
    M(3,0) = 1.0;
    N(3,0) = 0.0; // no translation for normals
    unsigned int cnt_tmp = 0;

#pragma omp for schedule(dynamic)
    for (unsigned int i=0; i<count; i++)
    {
      double ray[3];
      ray[0] = (*R)(0, i);
      ray[1] = (*R)(1, i);
      ray[2] = (*R)(2, i);

      if(rayCastFromSensorPose(space, tr, ray, c, n, color, &depth)) // Ray returned with coordinates
      {
        M(0,0) = c[0];
        M(1,0) = c[1];
        M(2,0) = c[2];
        N(0,0) = n[0];
        N(1,0) = n[1];
        N(2,0) = n[2];
        M       = Tinv * M;
        N       = Tinv * N;
        mask_tmp[cnt_tmp/3] = true;
        for (unsigned int i = 0; i < 3; i++)
        {
          c_tmp[cnt_tmp]      = M(i,0);
          //color_tmp[cnt_tmp]  = color[i];
          n_tmp[cnt_tmp++]    = N(i,0);
        }
      }
      else
      {
        mask_tmp[cnt_tmp/3] = false;
        for (unsigned int i = 0; i < 3; i++)
        {
          c_tmp[cnt_tmp]      = 0;
          //color_tmp[cnt_tmp]  = 0;
          n_tmp[cnt_tmp++]    = 0;
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[ctr],  c_tmp,     cnt_tmp*sizeof(double));
      memcpy(&normals[ctr], n_tmp,     cnt_tmp*sizeof(double));
      //memcpy(&rgb[ctr],     color_tmp, cnt_tmp*sizeof(unsigned char));
      memcpy(&mask[ctr/3],  mask_tmp,  cnt_tmp/3*sizeof(bool));
      ctr += cnt_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    //delete[] color_tmp;
    delete[] mask_tmp;
  }

  *size = ctr;
  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << ctr << " coordinates");

#if PRINTSTATISTICS
  LOGMSG(DBG_DEBUG, "Traversed: " << _traversed << ", skipped: " << _skipped);
#endif
}

/*bool RayCast3D::calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, const std::vector<TsdSpace*>& spaces,
    const std::vector<double>& offsets, const unsigned int u, const unsigned int v)
{
  double depthVar = 0.0;
  double coordVar[3];
  double normalVar[3];
  unsigned char colorVar[3]   = {255, 255, 255};
  double ray[3];
  Matrix Tinv(4, 4);
  Tinv = *sensor->getPose();
  double tr[3];
  sensor->getPosition(tr);
  unsigned int spaceCtr = 0;
  Matrix M(4,1);
  Matrix N(4,1);
  M(3,0) = 1.0;
  N(3,0) = 0.0; // no translation for normals  -> no homogeneous coordinates???
  bool found = false;
  std::vector<double>::const_iterator offIter = offsets.begin();
  //unsigned int offCtr = 0;
  double offset[3];
  obvious::Matrix T(4,4);
  for(std::vector<TsdSpace*>::const_iterator spaIter = spaces.begin(); spaIter != spaces.end(); spaIter++)
  {
    offset[0] = *offIter;
    offIter++;
    offset[1] = *offIter;
    offIter++;
    offset[2] = *offIter;
    offIter++;
    T.setIdentity();
    T(0,3) = (-1.0) * offset[0];
    T(1,3) = (-1.0) * offset[1];
    T(2,3) = (-1.0) * offset[2];
    sensor->transform(&T);
    sensor->calcRayFromCurrentPose(v, u, ray);
    ray[0] *= space->getVoxelSize();
    ray[1] *= space->getVoxelSize();
    ray[2] *= space->getVoxelSize();
    if(rayCastFromSensorPose(*spaIter, tr, ray, coordVar, normalVar, colorVar, &depthVar)) // Ray returned with coordinates
    {
      found = true;
      break;
    }
    T.invert();
    sensor->transform(&T);
    spaceCtr++;
  }
  if(!found)
    return(false);
  M(0,0) = coordVar[0];
  M(1,0) = coordVar[1];
  M(2,0) = coordVar[2];
  N(0,0) = normalVar[0];
  N(1,0) = normalVar[1];
  N(2,0) = normalVar[2];
  T.invert();
  Tinv.invert();
  M       =  T * M;
  M       = Tinv * M;
  N       = Tinv * N;
  for (unsigned int i = 0; i < 3; i++)
  {
    coords[i]  = M(i,0);
    rgb[i]     = colorVar[i];
    normals[i] = N(i,0);
  }

  sensor->transform(&T);
  return(true);
}*/

bool RayCast3D::rayCastFromSensorPose(TsdSpace* space, double pos[3], double ray[3], double coordinates[3], double normal[3], unsigned char rgb[3], double* depth)
{
  double position[3];

  int xDim = space->getXDimension();
  double voxelSize = space->getVoxelSize();

  // Interpolation weight
  double interp;

  double xmin   = _xmin;
  double ymin   = _ymin;
  double zmin   = _zmin;

  double xmax   = _xmax;
  double ymax   = _ymax;
  double zmax   = _zmax;

  // Leave out outmost cells in order to prevent access to invalid neighbors
  double minSpaceCoord = 1.5*voxelSize;
  double maxSpaceCoord = (((double)xDim)-1.5)*voxelSize;

  // Calculate minimum number of steps to reach bounds in each dimension
  if(ray[0]>10e-6)
  {
    xmin = (minSpaceCoord - pos[0]) / ray[0];
    xmax = (maxSpaceCoord - pos[0]) / ray[0];
  }
  else if(ray[0]<-10e-6)
  {
    xmin = (maxSpaceCoord - pos[0]) / ray[0];
    xmax = (minSpaceCoord - pos[0]) / ray[0];
  }

  if(ray[1]>10e-6)
  {
    ymin = (minSpaceCoord - pos[1]) / ray[1];
    ymax = (maxSpaceCoord - pos[1]) / ray[1];
  }
  else if(ray[1]<-10e-6)
  {
    ymin = (maxSpaceCoord - pos[1]) / ray[1];
    ymax = (minSpaceCoord - pos[1]) / ray[1];
  }

  if(ray[2]>10e-6)
  {
    zmin = (minSpaceCoord - pos[2]) / ray[2];
    zmax = (maxSpaceCoord - pos[2]) / ray[2];
  }
  else if(ray[2]<-10e-6)
  {
    zmin = (maxSpaceCoord - pos[2]) / ray[2];
    zmax = (minSpaceCoord - pos[2]) / ray[2];
  }

  // At least the entry bounds of each dimension needs to be crossed
  double idxMin = max(max(xmin, ymin), zmin);
  idxMin = ceil(idxMin);

  // No exit bound must be crossed
  double idxMax = min(min(xmax, ymax), zmax);
  idxMax = floor(idxMax);

  // clip steps to sensor modalities, i.e., the working range
  idxMin = max(idxMin, _idxMin);
  idxMax = min(idxMax, _idxMax);

  if (idxMin >= idxMax)
    return false;

#if PRINTSTATISTICS
  int idxMinTmp = idxMin;
#endif

  // Traverse partitions roughly to clip minimum index
  double partitionSize = space->getPartitionSize();
  for(double i=idxMin; i<idxMax; i+=partitionSize)
  {
    position[0] = pos[0] + i * ray[0];
    position[1] = pos[1] + i * ray[1];
    position[2] = pos[2] + i * ray[2];
    if(space->isPartitionInitialized(position))
    {
      break;
    }
    else
      idxMin = i+1.0;
  }

  // Traverse in single steps with quick test
  for(double i=idxMin; i<idxMax; i+=1.0)
  {
    position[0] = pos[0] + i * ray[0];
    position[1] = pos[1] + i * ray[1];
    position[2] = pos[2] + i * ray[2];
    double tsd;
    EnumTsdSpaceInterpolate retval = space->getTsd(position, &tsd);
    if(retval==INTERPOLATE_SUCCESS && fabs(tsd)<1.0)
      break;
    else
      idxMin++;
  }

#if PRINTSTATISTICS
#pragma omp critical
{
  if((int)idxMin != idxMinTmp)
    _skipped += (idxMin-idxMinTmp);
}
#endif

  double tsd_prev;

  if(space->interpolateTrilinear(position, &tsd_prev)!=INTERPOLATE_SUCCESS)
    tsd_prev = NAN;

  bool found = false;

  double i;

  for(i=idxMin; i<idxMax; i+=1.0)
  {
    double tsd;
    EnumTsdSpaceInterpolate retval = space->interpolateTrilinear(position, &tsd);
    if (retval!=INTERPOLATE_SUCCESS)
    {
      tsd_prev = NAN;
      position[0] += ray[0];
      position[1] += ray[1];
      position[2] += ray[2];
      continue;
    }

    // check sign change
    if(tsd_prev > 0 && tsd < 0)
    {
      interp = tsd_prev / (tsd_prev - tsd);
      //if(sensor->hasRealMeasurmentRGB()) space->interpolateTrilinearRGB(position, rgb);
      found = true;
      break;
    }

    position[0] += ray[0];
    position[1] += ray[1];
    position[2] += ray[2];
    tsd_prev = tsd;
  }

#if PRINTSTATISTICS
#pragma omp critical
{
  _traversed += i-idxMin;
}
#endif

  if(!found) return false;

  // interpolate between voxels when sign changes
  coordinates[0] = position[0] + ray[0] * (interp-1.0);
  coordinates[1] = position[1] + ray[1] * (interp-1.0);
  coordinates[2] = position[2] + ray[2] * (interp-1.0);

  if(!space->interpolateNormal(coordinates, normal))
    return false;

  return true;
}

}
