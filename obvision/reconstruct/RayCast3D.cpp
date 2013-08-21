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

bool RayCast3D::generatePointCloud(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size)
{

  // X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
  // X_AXS_N parallel to X-Axis negative direction
  // Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
  // Y_AXS_N parallel to Y-Axis negative direction
  // Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
  // Z_AXS_N parallel to Z-Axis negative direction
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();

  std::vector<double> vCloud;
  std::vector<double> vNormals;
  std::vector<unsigned char> vRGB;

  double *footPoint = new double[3];
  double *dirVec = new double[3];
  unsigned int steps;

  //parallel to x-axis
  cout << "X_AXIS" << endl;
  for (int row=0; row<yDim; row++)
  {
    for (int col=0; col<zDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, X_AXS)))
      {
        cout << "generatePointCloud: Error calculating X-axis Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, X_AXS_N)))
      {
        cout << "generatePointCloud: Error calculating X-axis negative Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);

    }
  }

  //parallel to y-axis
  cout << "Y_AXIS" << endl;
  for (int row=0; row<zDim; row++)
  {
    for (int col=0; col<xDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Y_AXS)))
      {
        cout << "generatePointCloud: Error calculating Y-axis Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Y_AXS_N)))
      {
        cout << "generatePointCloud: Error calculating Y-axis negative Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint,dirVec,&vCloud,&vNormals,&vRGB,steps);
    }
  }

  //parallel to z-axis
  cout << "Z_AXIS" << endl;
  for (int row = 0; row < yDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Z_AXS)))
      {
        cout << "generatePointCloud: Error calculating Z-axis Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Z_AXS_N)))
      {
        cout << "generatePointCloud: Error calculating Z-axis negative Raycaster!" << endl;
        return(false);
      }
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
    *pointCloud = new double[vCloud.size()];
    *cloudNormals = new double[vNormals.size()];
    *cloudRgb = new unsigned char[vCloud.size()];
  }

  for(unsigned int i=0; i<vCloud.size(); i++)
  {
    (*pointCloud)[i] = vCloud[i];
    (*cloudNormals)[i] = vNormals[i];
    (*cloudRgb)[i] = vRGB[i];
  }

  *size = vCloud.size();

  return(true);
}

bool RayCast3D::generatePointCloudPositive(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size)
{
  // X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
  // X_AXS_N parallel to X-Axis negative direction
  // Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
  // Y_AXS_N parallel to Y-Axis negative direction
  // Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
  // Z_AXS_N parallel to Z-Axis negative direction
  //
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();

  std::vector<double> vCloud;
  std::vector<double> vNormals;
  std::vector<unsigned char> vRGB;

  double *footPoint = new double[3];
  double *dirVec = new double[3];
  unsigned int steps;

  //x_parallel
  cout << "X_AXIS" << endl;
  for (int row=0; row<yDim; row++)
  {
    for (int col=0; col<zDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, X_AXS)))
      {
        cout << "generatePointCloud: Error calculating X-axis Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
    }
  }

  //y_parallel
  cout << "Y_AXIS" << endl;
  for (int row = 0; row < zDim; row++)
  {
    for (int col = 0; col < xDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Y_AXS)))
      {
        cout << "generatePointCloud: Error calculating Y-axis Raycaster!" << endl;
        return(false);
      }
      rayCastParallelAxis(footPoint, dirVec, &vCloud, &vNormals, &vRGB, steps);
    }
  }

  //z_parallel
  cout << "Z_AXIS" << endl;
  for (int row=0; row<yDim; row++)
  {
    for (int col=0; col<xDim; col++)
    {
      if(!(calcRayParallelAxis(row, col, footPoint, dirVec, &steps, Z_AXS)))
      {
        cout << "generatePointCloud: Error calculating Z-axis Raycaster!" << endl;
        return(false);
      }
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
    *pointCloud=new double[vCloud.size()];
    *cloudNormals=new double[vNormals.size()];
    *cloudRgb=new unsigned char[vCloud.size()];
  }

  for(unsigned int i=0;i<vCloud.size();i++)
  {
    (*pointCloud)[i] = vCloud[i];
    (*cloudNormals)[i] = vNormals[i];
    (*cloudRgb)[i] = vRGB[i];
  }

  *size=vCloud.size();

  return(true);
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
    tsdfPrev=1.0;

  for(unsigned int i=0;i<=steps;i++)
  {
    //calculate new position, store old position
    memcpy(prevPosition, curPosition, 3*sizeof(double));
    for(unsigned int i=0; i<3; i++)
      curPosition[i] += dirVec[i];

    if (!_space->interpolateTrilinear(curPosition, &tsdf))
      continue;

    // check sign change
    if(tsdfPrev > 0 && tsdfPrev < 0.99999 && tsdf < 0)
    {
      // interpolate between voxels when sign change happened
      curPrevInterp = tsdfPrev / (tsdfPrev - tsdf);
      for (unsigned int i = 0; i < 3; i++)
        zeroCrossing[i] = prevPosition[i] + dirVec[i] * curPrevInterp;

      if(!_space->interpolateNormal(zeroCrossing, normal))
        continue;
      if(!_space->interpolateTrilinearRGB(curPosition, zerCrossingRgb))
      {
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

bool RayCast3D::calcRayParallelAxis(const unsigned int row, const unsigned int col, double* footPoint, double* dirVec, unsigned int* steps, AXSPARMODE mode)
{
  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  if((mode==X_AXS)||(mode==X_AXS_N))       //parallel to x_axis
  {
    footPoint[1] = ((double) row + 0.5) * voxelSize;
    footPoint[2] = ((double) col + 0.5) * voxelSize;
    dirVec[1] = 0.0;
    dirVec[2] = 0.0;
    *steps = xDim-1;
    if(mode==X_AXS)
    {
      footPoint[0] = 0.5*voxelSize;
      dirVec[0] = voxelSize;
    }
    else
    {
      footPoint[0] = ((double)(xDim-1) + 0.5) * voxelSize;
      dirVec[0] = (-1.0)*voxelSize;
    }
  }

  else if((mode==Y_AXS)||(mode==Y_AXS_N))       //parallel to x_axis
  {
    footPoint[0] = ((double) col + 0.5) * voxelSize;
    footPoint[2] = ((double) row + 0.5) * voxelSize;
    dirVec[0] = 0.0;
    dirVec[2] = 0.0;
    *steps = yDim-1;
    if(mode==Y_AXS)
    {
      footPoint[1] = 0.5*voxelSize;
      dirVec[1] = voxelSize;
    }
    else
    {
      footPoint[1] = ((double)(yDim-1) + 0.5) * voxelSize;
      dirVec[1] = (-1.0)*voxelSize;
    }
  }

  else if((mode==Z_AXS)||(mode==Z_AXS_N))       //parallel to x_axis
  {
    footPoint[0] = ((double) col + 0.5) * voxelSize;
    footPoint[1] = ((double) row + 0.5) * voxelSize;
    dirVec[0] = 0.0;
    dirVec[1] = 0.0;
    *steps = zDim-1;
    if(mode==Z_AXS)
    {
      footPoint[2] = 0.5*voxelSize;
      dirVec[2] = voxelSize;
    }
    else
    {
      footPoint[2] = ((double)(zDim-1) + 0.5) * voxelSize;
      dirVec[2] = (-1.0)*voxelSize;
    }
  }
  else
  {
    cout << "ERROR. Mode " << mode << " not found!" << endl;
    return(false);
  }

  return (true);
}

}
