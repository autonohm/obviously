#include "RayCast3D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCast3D::RayCast3D(TsdSpace* space)
{
  _space = space;
}

RayCast3D::~RayCast3D()
{

}

void RayCast3D::calcCoordsFromCurrentPose(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size)
{
  Timer t;
  *size = 0;

  Matrix* T = sensor->getPose();

  Matrix Tinv(4, 4);
  Tinv = T->getInverse();

  unsigned int width = sensor->getWidth();
  unsigned int height = sensor->getHeight();

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[width*height*3];
    double* n_tmp            = new double[width*height*3];
    unsigned char* color_tmp = new unsigned char[width*height*3];
    unsigned int size_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for (unsigned int v = 0; v < height; v++)
    {
      for (unsigned int u = 0; u < width; u++)
      {
        double ray[3];
        sensor->calcRayFromCurrentPose(v, u, ray);

        ray[0] *= _space->getVoxelSize();
        ray[1] *= _space->getVoxelSize();
        ray[2] *= _space->getVoxelSize();

        if(rayCastFromSensorPose(ray, c, n, color, &depth, sensor)) // Ray returned with coordinates
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

void RayCast3D::calcCoordsFromCurrentPose(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, const std::vector<TsdSpace*>& spaces,
                                               const std::vector<unsigned int>& coloums, const std::vector<unsigned int>& rows)
{
  if((spaces.size() != coloums.size()) || (coloums.size() != rows.size()) || (spaces.size() != rows.size()))
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Input vectors differ in size!\n";
    return;
  }
  std::vector<TsdSpace*>::const_iterator spIter   = spaces.begin();
  std::vector<unsigned int>::const_iterator uIter = coloums.begin();
  std::vector<unsigned int>::const_iterator vIter = rows.begin();
  double depthVar = 0.0;
  double coordVar[3];
  double normalVar[3];
  unsigned char colorVar[3]   = {255, 255, 255};
  double ray[3];
  Matrix* T = sensor->getPose();
  Matrix Tinv(4, 4);
  Tinv = T->getInverse();
  Matrix M(4,1);
  Matrix N(4,1);
  M[3][0] = 1.0;
  N[3][0] = 0.0; // no translation for normals  -> no homogenous coordinates???
  bool found = false;

  while(spIter != spaces.end())
  {
    this->setSpace(*spIter);
    sensor->calcRayFromCurrentPose(*vIter, *uIter, ray);
    ray[0] *= _space->getVoxelSize();
    ray[1] *= _space->getVoxelSize();
    ray[2] *= _space->getVoxelSize();
    if(rayCastFromSensorPose(ray, coordVar, normalVar, colorVar, &depthVar, sensor)) // Ray returned with coordinates
    {
      found = true;
      break;
    }
  }
  if(!found)
  {
    std::cout << __PRETTY_FUNCTION__ << " no coordinates found!\n";
    return;
  }
  M[0][0] = coordVar[0];
  M[1][0] = coordVar[1];
  M[2][0] = coordVar[2];
  N[0][0] = normalVar[0];
  N[1][0] = normalVar[1];
  N[2][0] = normalVar[2];
  M       = Tinv * M;
  N       = Tinv * N;
  for (unsigned int i = 0; i < 3; i++)
  {
    coords[i]  = M[i][0];
    rgb[i]     = colorVar[i];
    normals[i] = N[i][0];
  }
}

void RayCast3D::calcCoordsFromCurrentPoseMask(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, bool* mask, unsigned int* size)
{
  Timer t;

  Matrix* T = sensor->getPose();
  Matrix Tinv(4, 4);
  Tinv = T->getInverse();
  unsigned int ctr = 0;

  unsigned int width = sensor->getWidth();
  unsigned int height = sensor->getHeight();

#pragma omp parallel
  {
    double depth = 0.0;
    double c[3];
    double n[3];
    unsigned char color[3]   = {255, 255, 255};
    double* c_tmp            = new double[width*height*3];
    double* n_tmp            = new double[width*height*3];
    unsigned char* color_tmp = new unsigned char[width*height*3];
    bool* mask_tmp           = new bool[width*height];
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals
    unsigned int cnt_tmp = 0;

#pragma omp for schedule(dynamic)
    for (unsigned int v = 0; v < height; v++)
    {
      for (unsigned int u = 0; u < width; u++)
      {
        double ray[3];

        sensor->calcRayFromCurrentPose(v, u, ray);

        ray[0] *= _space->getVoxelSize();
        ray[1] *= _space->getVoxelSize();
        ray[2] *= _space->getVoxelSize();

        if(rayCastFromSensorPose(ray, c, n, color, &depth, sensor)) // Ray returned with coordinates
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

  *size = ctr;
  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << ctr << " coordinates");
}

bool RayCast3D::calcCoordsAxisParallel(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size)
{
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();

  std::vector<double> vCloud;
  std::vector<double> vNormals;
  std::vector<unsigned char> vRGB;

  double footPoint[3];
  double dirVec[3];
  unsigned int steps;

  cout << "X_AXIS" << endl;
  for (int row=0; row<yDim; row++)
  {
    for (int col=0; col<zDim; col++)
    {
      calcRayParallelAxis(row, col, footPoint, dirVec, &steps, X_AXS);
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
    }
  }

  cout << "Y_AXIS" << endl;
  for (int row = 0; row < zDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Y_AXS);
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
    }
  }

  cout << "Z_AXIS" << endl;
  for (int row=0; row<yDim; row++)
  {
    for (int col=0; col<xDim; col++)
    {
      calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Z_AXS);
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
    }
  }

  if(vCloud.size()!=vNormals.size())
  {
    cout << "generatePointCloud: Error! Number of normals != number of points!" << endl;
    return(false);
  }
  else
  {
    *pointCloud   = new double[vCloud.size()];
    *cloudNormals = new double[vNormals.size()];
    *cloudRgb     = new unsigned char[vCloud.size()];
  }

  for(unsigned int i=0;i<vCloud.size();i++)
  {
    (*pointCloud)[i]   = vCloud[i];
    (*cloudNormals)[i] = vNormals[i];
    (*cloudRgb)[i]     = vRGB[i];
  }

  *size=vCloud.size();

  return(true);
}

bool RayCast3D::rayCastFromSensorPose(double ray[3], double coordinates[3], double normal[3], unsigned char rgb[3], double* depth, Sensor* sensor)
{
  double tr[3];

  sensor->getPosition(tr);

  double position[3];
  double position_prev[3];

  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  // Interpolation weight
  double interp;

  double xmin   = -10e9;
  double ymin   = -10e9;
  double zmin   = -10e9;
  if(fabs(ray[0])>10e-6) xmin = ((double)(ray[0] > 0.0 ? 0 : (xDim-1)*voxelSize) - tr[0]) / ray[0];
  if(fabs(ray[1])>10e-6) ymin = ((double)(ray[1] > 0.0 ? 0 : (yDim-1)*voxelSize) - tr[1]) / ray[1];
  if(fabs(ray[2])>10e-6) zmin = ((double)(ray[2] > 0.0 ? 0 : (zDim-1)*voxelSize) - tr[2]) / ray[2];
  double idxMin = max(max(xmin, ymin), zmin);
  idxMin        = max(idxMin, 0.0);

  double xmax   = 10e9;
  double ymax   = 10e9;
  double zmax   = 10e9;
  if(fabs(ray[0])>10e-6) xmax = ((double)(ray[0] > 0.0 ? (xDim-1)*voxelSize : 0) - tr[0]) / ray[0];
  if(fabs(ray[1])>10e-6) ymax = ((double)(ray[1] > 0.0 ? (yDim-1)*voxelSize : 0) - tr[1]) / ray[1];
  if(fabs(ray[2])>10e-6) zmax = ((double)(ray[2] > 0.0 ? (zDim-1)*voxelSize : 0) - tr[2]) / ray[2];
  double idxMax = min(min(xmax, ymax), zmax);

  if (idxMin >= idxMax)
    return false;

  double tsdf_prev;
  position[0] = tr[0] + idxMin * ray[0];
  position[1] = tr[1] + idxMin * ray[1];
  position[2] = tr[2] + idxMin * ray[2];
  if(!_space->interpolateTrilinear(position, &tsdf_prev))
    tsdf_prev = NAN;

  bool found = false;
  for(int i=idxMin; i<idxMax; i++)
  {
    // calculate current position
    memcpy(position_prev, position, 3 * sizeof(*position));

    position[0] += ray[0];
    position[1] += ray[1];
    position[2] += ray[2];

    double tsdf;
    bool retval = _space->interpolateTrilinear(position, &tsdf);
    if (!retval)
    {
      tsdf_prev = tsdf;
      continue;
    }

    // check sign change
    if(tsdf_prev > 0 && tsdf < 0)
    {
      interp = tsdf_prev / (tsdf_prev - tsdf);
      if(sensor->hasRealMeasurmentRGB()) _space->interpolateTrilinearRGB(position, rgb);
      found = true;
      break;
    }

    tsdf_prev = tsdf;
  }

  if(!found) return false;

  // interpolate between voxels when sign changes
  for (unsigned int i = 0; i < 3; i++)
    coordinates[i] = position_prev[i] + ray[i] * interp;

  if(!_space->interpolateNormal(coordinates, normal))
    return false;

  return true;
}

bool RayCast3D::rayCastParallelAxis(double* footPoint, double* dirVec,std::vector<double>* pointCloud,std::vector<double>* cloudNormals, std::vector<unsigned char>* cloudRgb, const unsigned int steps)
{
  double tsdf = 0.0;
  double tsdfPrev = 0.0;
  double curPrevInterp = 0.0;
  double curPosition[3] = {0.0};
  double prevPosition[3] = {0.0};
  double zeroCrossing[3] = {0.0};
  double normal[3] = {0.0};
  unsigned char zerCrossingRgb[3] = {0.0};
  memcpy(curPosition, footPoint, 3*sizeof(double));

  if (!_space->interpolateTrilinear(curPosition, &tsdfPrev))
    tsdfPrev=NAN;

  for(unsigned int i=0;i<=steps;i++)
  {
    //calculate new position, store previous position
    memcpy(prevPosition, curPosition, 3*sizeof(double));
    for(unsigned int i=0; i<3; i++)
      curPosition[i] += dirVec[i];

    if (!_space->interpolateTrilinear(curPosition, &tsdf))
    {
      tsdfPrev = tsdf;
      continue;
    }
    // check sign change
    if((tsdfPrev > 0 && tsdf < 0) || (tsdfPrev < 0 && tsdf > 0))
    {
      // interpolate between voxels when sign change happens
      curPrevInterp = tsdfPrev / (tsdfPrev - tsdf);
      for (unsigned int i = 0; i < 3; i++)
        zeroCrossing[i] = prevPosition[i] + dirVec[i] * curPrevInterp;

      if(!_space->interpolateNormal(zeroCrossing, normal))
      {
        tsdfPrev = tsdf;
        continue;
      }

      if(!_space->interpolateTrilinearRGB(curPosition, zerCrossingRgb))
      {
        tsdfPrev = tsdf;
        continue;
      }

      for(unsigned int i=0;i<3;i++)    //found zero crossing with normal -> store in cloud
      {
        pointCloud->push_back(zeroCrossing[i]);
        cloudNormals->push_back(normal[i]);
        cloudRgb->push_back(zerCrossingRgb[i]);
      }
    }
    tsdfPrev = tsdf;

  }
  return(true);
}

void RayCast3D::calcRayParallelAxis(const unsigned int row, const unsigned int col, double* footPoint, double* dirVec, unsigned int* steps, AXSPARMODE mode)
{
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  if(mode==X_AXS)
  {
    footPoint[1] = ((double) row + 0.5) * voxelSize;
    footPoint[2] = ((double) col + 0.5) * voxelSize;
    dirVec[1] = 0.0;
    dirVec[2] = 0.0;
    *steps = xDim-1;
    footPoint[0] = 0.5*voxelSize;
    dirVec[0] = voxelSize;
  }
  else if(mode==Y_AXS)
  {
    footPoint[0] = ((double) col + 0.5) * voxelSize;
    footPoint[2] = ((double) row + 0.5) * voxelSize;
    dirVec[0] = 0.0;
    dirVec[2] = 0.0;
    *steps = yDim-1;
    footPoint[1] = 0.5*voxelSize;
    dirVec[1] = voxelSize;
  }
  else if(mode==Z_AXS)
  {
    footPoint[0] = ((double) col + 0.5) * voxelSize;
    footPoint[1] = ((double) row + 0.5) * voxelSize;
    dirVec[0] = 0.0;
    dirVec[1] = 0.0;
    *steps = zDim-1;
    footPoint[2] = 0.5*voxelSize;
    dirVec[2] = voxelSize;
  }
  else
  {
    cout << "ERROR. Mode " << mode << " not found!" << endl;
    abort();
  }
}

}
