#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/Matrix.h"
#include "obcore/math/mathbase.h"
#include "TsdSpace.h"
#include "SensorProjective3D.h"

#include <cstring>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 32.0
#define RGB_MAX 255

TsdSpace::TsdSpace(const unsigned int height, const unsigned int width, const unsigned int depth, const double voxelSize)
{
  _voxelSize = voxelSize;
  _invVoxelSize = 1.0 / _voxelSize;

  // determine number of voxels in each dimension
  _xDim = ((double)width * _invVoxelSize + 0.5);
  _yDim = ((double)height * _invVoxelSize + 0.5);
  _zDim = ((double)depth * _invVoxelSize + 0.5);
  _sizeOfSpace = _zDim * _yDim * _xDim;

  _height = height;
  _width = width;
  _depth = depth;
  _maxTruncation = 2*voxelSize;

  LOGMSG(DBG_DEBUG, "Dimensions are (x/y/z) (" << _xDim << "/" << _yDim << "/" << _zDim << ")");
  LOGMSG(DBG_DEBUG, "Creating TsdVoxel Space...");

  System<TsdVoxel>::allocate(_zDim, _yDim, _xDim, _space);
  LOGMSG(DBG_DEBUG, "TSDVoxel Space allocated, " << _zDim*_yDim*_xDim << " voxels");

  _minX = 0.0;
  _maxX = ((double)_xDim + 0.5) * _voxelSize;
  _minY = 0.0;
  _maxY = ((double)_yDim + 0.5) * _voxelSize;
  _minZ = 0.0;
  _maxZ = ((double)_zDim + 0.5) * _voxelSize;

  reset();
}

TsdSpace::~TsdSpace(void)
{
  System<TsdVoxel>::deallocate(_space);
}

void TsdSpace::reset()
{
  for (int z = 0; z < _zDim; z++)
  {
    for (int y = 0; y < _yDim; y++)
    {
      for (int x = 0; x < _xDim; x++)
      {
        _space[z][y][x].tsdf   = 0.0;
        _space[z][y][x].weight = 0.0;
      }
    }
  }
}

unsigned int TsdSpace::getXDimension()
{
  return _xDim;
}

unsigned int TsdSpace::getYDimension()
{
  return _yDim;
}

unsigned int TsdSpace::getZDimension()
{
  return _zDim;
}

double TsdSpace::getVoxelSize()
{
  return _voxelSize;
}

double TsdSpace::getMinX()
{
  return _minX;
}

double TsdSpace::getMaxX()
{
  return _maxX;
}

double TsdSpace::getMinY()
{
  return _minY;
}

double TsdSpace::getMaxY()
{
  return _maxY;
}

double TsdSpace::getMinZ()
{
  return _minZ;
}

double TsdSpace::getMaxZ()
{
  return _maxZ;
}

void TsdSpace::setMaxTruncation(double val)
{
  if(val < 2 * _voxelSize)
  {
    LOGMSG(DBG_WARN, "Truncation radius must be at 2 x voxel dimension. Setting minimum size.");
    val = 2 * _voxelSize;
  }

  _maxTruncation = val;
}

double TsdSpace::getMaxTruncation()
{
  return _maxTruncation;
}

void TsdSpace::push(Sensor* sensor)
{
  Timer t;

  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();
  unsigned char* rgb = sensor->getRealMeasurementRGB();

  double tr[3];
  sensor->getPosition(tr);

#pragma omp parallel
  {
    Matrix* V = new Matrix(_yDim*_xDim, 4);
    int* indices = new int[_yDim*_xDim];

    int i=0;
    for (int y = 0; y < _yDim; y++)
    {
      for (int x = 0; x < _xDim; x++, i++)
      {
        (*V)[i][0] = ((double)x + 0.5) * _voxelSize;
        (*V)[i][1] = ((double)y + 0.5) * _voxelSize;
        (*V)[i][3] = 1.0;
      }
    }
#pragma omp for schedule(dynamic)
    for(int z=0; z<_zDim; z++)
    {
      double zVoxel = ((double)z + 0.5) * _voxelSize;
      for (i = 0; i < _yDim*_xDim; i++)
        (*V)[i][2] = zVoxel;

      sensor->backProject(V, indices);

      i = 0;
      for(int y=0; y<_yDim; y++)
      {
        for(int x=0; x<_xDim; x++, i++)
        {
          // Measurement index
          int index = indices[i];

          if(index>=0)
          {
            if(mask[index])
            {
              // calculate distance of current cell to sensor
              double distance = euklideanDistance<double>(tr, (*V)[i], 3);
              double sdf = data[index] - distance;

              unsigned char* color = NULL;
              if(rgb) color = &(rgb[3*index]);
              addTsdfValue(x, y, z, sdf, color);
            }
            else
              cout << "filtered" << endl;
          }
        }
      }
    }
    delete V;
    delete [] indices;
  }

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");

}

bool TsdSpace::interpolateNormal(const double* coord, double* normal)
{
  double neighbor[3];
  double depthVarInc = 0;
  double depthVarDec = 0;

  //interpolate around Voxel in x+1 direction
  neighbor[0] = coord[0] + _voxelSize;
  neighbor[1] = coord[1];
  neighbor[2] = coord[2];
  if(!interpolateTrilinear(neighbor, &depthVarInc))
    return false;

  //interpolate around Voxel in x-1 direction
  neighbor[0] = coord[0] - _voxelSize;
  //neighbor[1] = coord[1];
  //neighbor[2] = coord[2];
  if(!interpolateTrilinear(neighbor, &depthVarDec))
    return false;

  //x-coordinate of normal vector
  normal[0] = depthVarInc - depthVarDec;

  //interpolate around Voxel in y+1 direction
  neighbor[0] = coord[0];
  neighbor[1] = coord[1] + _voxelSize;
  //neighbor[2] = coord[2];
  if(!interpolateTrilinear(neighbor, &depthVarInc))
    return false;

  //interpolate around Voxel in y-1 direction
  //neighbor[0] = coord[0];
  neighbor[1] = coord[1] - _voxelSize;
  //neighbor[2] = coord[2];
  if(!interpolateTrilinear(neighbor, &depthVarDec))
    return false;

  //y-coordinate of normal vector
  normal[1] = depthVarInc - depthVarDec;

  //interpolate around Voxel in z+1 direction
  //neighbor[0] = coord[0];
  neighbor[1] = coord[1];
  neighbor[2] = coord[2] + _voxelSize;
  if(!interpolateTrilinear(neighbor, &depthVarInc))
    return false;

  //interpolate around Voxel in z-1 direction
  //neighbor[0] = coord[0];
  //neighbor[1] = coord[1];
  neighbor[2] = coord[2] - _voxelSize;
  if(!interpolateTrilinear(neighbor, &depthVarDec))
    return false;

  //z-coordinate of normal vector
  normal[2] = depthVarInc - depthVarDec;

  norm3<double>(normal);

  return true;
}

bool TsdSpace::interpolateTrilinear(double coord[3], double* tsdf)
{
  int x, y, z;
  Point p;
  if(!coord2Voxel(coord, &x, &y, &z, &p)) return false;

  // get weights
  double wX = (coord[0] - p.x) * _invVoxelSize;
  double wY = (coord[1] - p.y) * _invVoxelSize;
  double wZ = (coord[2] - p.z) * _invVoxelSize;

  // Interpolate
  *tsdf =   _space[z + 0][y + 0][x + 0].tsdf * (1. - wX) * (1. - wY) * (1. - wZ)
                      + _space[z + 1][y + 0][x + 0].tsdf * (1. - wX) * (1. - wY) * wZ
                      + _space[z + 0][y - 1][x + 0].tsdf * (1. - wX) * wY * (1. - wZ)
                      + _space[z + 1][y - 1][x + 0].tsdf * (1. - wX) * wY * wZ
                      + _space[z + 0][y + 0][x + 1].tsdf * wX * (1. - wY) * (1. - wZ)
                      + _space[z + 1][y + 0][x + 1].tsdf * wX * (1. - wY) * wZ
                      + _space[z + 0][y - 1][x + 1].tsdf * wX * wY * (1. - wZ)
                      + _space[z + 1][y - 1][x + 1].tsdf * wX * wY * wZ;

  return true;
}

bool TsdSpace::interpolateTrilinearRGB(double coord[3], unsigned char rgb[3])
{
  int x, y, z;
  Point p;
  if(!coord2Voxel(coord, &x, &y, &z, &p)) return false;

  double wX = (coord[0] - p.x) * _invVoxelSize;
  double wY = (coord[1] - p.y) * _invVoxelSize;
  double wZ = (coord[2] - p.z) * _invVoxelSize;

  unsigned char pRGB[8][3];

  pRGB[0][0] = _space[z + 0][y + 0][x + 0].rgb[0];
  pRGB[0][1] = _space[z + 0][y + 0][x + 0].rgb[1];
  pRGB[0][2] = _space[z + 0][y + 0][x + 0].rgb[2];

  pRGB[1][0] = _space[z + 1][y + 0][x + 0].rgb[0];
  pRGB[1][1] = _space[z + 1][y + 0][x + 0].rgb[1];
  pRGB[1][2] = _space[z + 1][y + 0][x + 0].rgb[2];

  pRGB[2][0] = _space[z + 0][y - 1][x + 0].rgb[0];
  pRGB[2][1] = _space[z + 0][y - 1][x + 0].rgb[1];
  pRGB[2][2] = _space[z + 0][y - 1][x + 0].rgb[2];

  pRGB[3][0] = _space[z + 1][y - 1][x + 0].rgb[0];
  pRGB[3][1] = _space[z + 1][y - 1][x + 0].rgb[1];
  pRGB[3][2] = _space[z + 1][y - 1][x + 0].rgb[2];

  pRGB[4][0] = _space[z + 0][y - 0][x + 1].rgb[0];
  pRGB[4][1] = _space[z + 0][y - 0][x + 1].rgb[1];
  pRGB[4][2] = _space[z + 0][y - 0][x + 1].rgb[2];

  pRGB[5][0] = _space[z + 1][y - 0][x + 1].rgb[0];
  pRGB[5][1] = _space[z + 1][y - 0][x + 1].rgb[1];
  pRGB[5][2] = _space[z + 1][y - 0][x + 1].rgb[2];

  pRGB[6][0] = _space[z + 0][y - 1][x + 1].rgb[0];
  pRGB[6][1] = _space[z + 0][y - 1][x + 1].rgb[1];
  pRGB[6][2] = _space[z + 0][y - 1][x + 1].rgb[2];

  pRGB[7][0] = _space[z + 1][y - 1][x + 1].rgb[0];
  pRGB[7][1] = _space[z + 1][y - 1][x + 1].rgb[1];
  pRGB[7][2] = _space[z + 1][y - 1][x + 1].rgb[2];

  double pw[8];
  pw[0] = (1. - wX) * (1. - wY) * (1. - wZ);
  pw[1] = (1. - wX) * (1. - wY) * wZ;
  pw[2] = (1. - wX) * wY * (1. - wZ);
  pw[3] = (1. - wX) * wY * wZ;
  pw[4] = wX * (1. - wY) * (1. - wZ);
  pw[5] = wX * (1. - wY) * wZ;
  pw[6] = wX * wY * (1. - wZ);
  pw[7] = wX * wY * wZ;

  memset(rgb,0,3);
  for(int i=0; i<8; i++)
  {
    rgb[0] += pRGB[i][0] * pw[i];
    rgb[1] += pRGB[i][1] * pw[i];
    rgb[2] += pRGB[i][2] * pw[i];
  }

  return true;
}

void TsdSpace::addTsdfValue(const unsigned int col, const unsigned int row, const unsigned int z, double sdf, unsigned char* rgb)
{

  if(sdf >= -_maxTruncation) // Voxel is in front of an object
  {
    TsdVoxel* voxel = &_space[z][(_yDim - 1) - row][col];

    // determine whether sdf/max_truncation = ]-1;1[
    double tsdf = sdf / _maxTruncation;
    tsdf = min(tsdf, 1.0);

    voxel->weight += 1.0;
    const double invWeight = 1.0 / voxel->weight;
    voxel->tsdf   = (voxel->tsdf * (voxel->weight - 1.0) + tsdf) * invWeight;

    if(rgb)
    {
      voxel->rgb[0] = (unsigned char)( (((double)voxel->rgb[0]) * (voxel->weight-1.0) + ((double)rgb[0]) ) * invWeight);
      voxel->rgb[1] = (unsigned char)( (((double)voxel->rgb[1]) * (voxel->weight-1.0) + ((double)rgb[1]) ) * invWeight);
      voxel->rgb[2] = (unsigned char)( (((double)voxel->rgb[2]) * (voxel->weight-1.0) + ((double)rgb[2]) ) * invWeight);
    }

    voxel->weight = min(voxel->weight, MAXWEIGHT);
  }
}

inline bool TsdSpace::coord2Voxel(double coord[3], int* x, int* y, int* z, Point* p)
{
  // initialize
  // get voxel indices
  int xIdx = (int) (coord[0] * _invVoxelSize);
  int yIdx = (int) (coord[1] * _invVoxelSize);
  int zIdx = (int) (coord[2] * _invVoxelSize);

  // check edges / 0 is edge because of voxel fine tuning
  if ((xIdx >= (_xDim - 2)) || (xIdx < 1) || (yIdx >= (_yDim - 2)) || (yIdx < 1) || (zIdx >= (_zDim - 2)) || (zIdx < 1))
    return false;

  // get center point of current voxel
  p->x = (double(xIdx) + 0.5) * _voxelSize;
  p->y = (double(yIdx) + 0.5) * _voxelSize;
  p->z = (double(zIdx) + 0.5) * _voxelSize;

  // voxel fine tuning -> shift to lower-left-front edge
  if (coord[0] < p->x)
  {
    xIdx--;
    p->x -= _voxelSize;
  }
  if (coord[1] < p->y)
  {
    yIdx--;
    p->y -= _voxelSize;
  }
  if (coord[2] < p->z)
  {
    zIdx--;
    p->z -= _voxelSize;
  }

  // turn y-axis
  yIdx = (_yDim - 1) - yIdx;

  *x = xIdx;
  *y = yIdx;
  *z = zIdx;

  return true;
}

bool TsdSpace::buildSliceImage(const unsigned int depthIndex, unsigned char* image)
{
  unsigned char R[_xDim * _yDim];
  unsigned char G[_xDim * _yDim];
  unsigned char B[_xDim * _yDim];
  unsigned int ctr = 0;
  unsigned im_ctr = 0;
  double cTsdf;

  // initialize arrays for RGB
  for (int i = 0; i < _xDim * _yDim; i++)
  {
    R[i] = 0;
    G[i] = 0;
    B[i] = 0;
  }

  // iterate over given slice, generate 2D-Picture
  for (int row = 0; row < _yDim; row++)
  {
    for (int col = 0; col < _xDim; col++)
    {
      // Get current tsdf
      cTsdf = _space[depthIndex][row][col].tsdf;

      // Blue for depth behind Voxel
      if (cTsdf > 0)
      {
        if (cTsdf > 0.999)
          G[im_ctr++] = (unsigned char) 150; //row*_xDim+col
        else
          B[im_ctr++] = (unsigned char) (cTsdf * RGB_MAX + 0.5); //row*_xDim+col
      }

      // Red for depth in front of Voxel
      else if (cTsdf < 0)
      {
        if (cTsdf < -0.9999)
          G[im_ctr++] = (unsigned char) 50;
        else
          R[im_ctr++] = (unsigned char) ((cTsdf * -1.0) * RGB_MAX + 0.5); //row*_xDim+col
      }
    }
  }

  //put components together to complete picture
  for (int i = 0; i < _xDim * _yDim * 3; i++)
  {
    image[i]   = R[ctr];
    image[++i] = G[ctr];
    image[++i] = B[ctr];
    ctr++;
  }

  return (true);
}

}
