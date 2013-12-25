#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGrid.h"

#include <cstring>
#include <cmath>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 32.0

TsdGrid::TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize, const unsigned int partitionSize)
{
  _cellSize = cellSize;
  _invCellSize = 1.0 / _cellSize;

  _cellsX = ((double)dimX * _invCellSize + 0.5);
  _cellsY = ((double)dimY * _invCellSize + 0.5);

  _dimPartition = partitionSize;
  _partitionsInX = _cellsX/_dimPartition;
  _partitionsInY = _cellsY/_dimPartition;

  if(_partitionsInX*_dimPartition != _cellsX || _partitionsInY*_dimPartition != _cellsY)
  {
    LOGMSG(DBG_ERROR, "Insufficient partition size : " << _dimPartition << "x" << _dimPartition << " in "
                                                       << _cellsX << "x" << _cellsY << "grid");
    return;
  }

  _sizeOfGrid = _cellsY * _cellsX;

  _dimX = dimX;
  _dimY = dimY;
  _maxTruncation = 2*cellSize;

  LOGMSG(DBG_DEBUG, "Grid dimensions are (x/y) (" << _cellsX << "/" << _cellsY << ")");

  _minX = 0.0;
  _maxX = ((double)_cellsX + 0.5) * _cellSize;
  _minY = 0.0;
  _maxY = ((double)_cellsY + 0.5) * _cellSize;

  LOGMSG(DBG_DEBUG, "Allocating " << _partitionsInX << "x" << _partitionsInY << " partitions");
  System<TsdGridPartition*>::allocate(_partitionsInY, _partitionsInX, _partitions);

  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      _partitions[py][px] = new TsdGridPartition(px*_dimPartition, py*_dimPartition, _dimPartition, _dimPartition, cellSize);
    }
  }
}

TsdGrid::~TsdGrid(void)
{
  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      delete _partitions[py][px];
    }
  }

  System<TsdGridPartition>::deallocate(_partitions);
}

double& TsdGrid::operator () (unsigned int y, unsigned int x)
{
  // Partition index
  unsigned int py = y / _dimPartition;
  unsigned int px = x / _dimPartition;

  // Cell index
  unsigned int cx = x % _dimPartition;
  unsigned int cy = y % _dimPartition;

  return (*_partitions[py][px])(cy, cx);
}

unsigned int TsdGrid::getCellsX()
{
  return _cellsX;
}

unsigned int TsdGrid::getCellsY()
{
  return _cellsY;
}

double TsdGrid::getCellSize()
{
  return _cellSize;
}

double TsdGrid::getMinX()
{
  return _minX;
}

double TsdGrid::getMaxX()
{
  return _maxX;
}

double TsdGrid::getMinY()
{
  return _minY;
}

double TsdGrid::getMaxY()
{
  return _maxY;
}

void TsdGrid::setMaxTruncation(double val)
{
  if(val < 2 * _cellSize)
  {
    LOGMSG(DBG_WARN, "Truncation radius must be at 2 x cell size. Setting minimum size.");
    val = 2 * _cellSize;
  }

  _maxTruncation = val;
}

double TsdGrid::getMaxTruncation()
{
  return _maxTruncation;
}

void TsdGrid::push(SensorPolar2D* sensor)
{
  Timer t;
  double* data = sensor->getRealMeasurementData();
  bool*   mask = sensor->getRealMeasurementMask();

  double tr[2];
  sensor->getPosition(tr);

#pragma omp parallel
  {
  unsigned int partSize = (_partitions[0][0])->getSize();
  int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
  for(unsigned int i=0; i<(unsigned int)(_partitionsInX*_partitionsInY); i++)
  {
    TsdGridPartition* part = _partitions[0][i];
    Matrix* edgeCoordsHom = part->getEdgeCoordsHom();
    Matrix* partCoords = part->getPartitionCoords();

    // Project back edge coordinates of partition
    int idxEdge[4];
    sensor->backProject(edgeCoordsHom, idxEdge);

    int minIdx;
    int maxIdx;
    minmaxArray<int>(idxEdge, 4, &minIdx, &maxIdx);

    // Check whether non of the cells are in the field of view
    if(maxIdx<0) continue;

    minIdx = 0;

    double* crd = part->getCentroid();
    double edge[2];
    edge[0] = (*edgeCoordsHom)(0,0);
    edge[1] = (*edgeCoordsHom)(0,1);

    // Radius of circumcircle
    double radius = euklideanDistance<double>(edge, crd, 2);

    // Centroid-to-sensor distance
    double distance = euklideanDistance<double>(tr, crd, 2);

    double minDist = distance - radius;
    double maxDist = distance + radius;

    // Check if any cell comes closer than the truncation radius
    bool isVisible = false;
    for(int j=minIdx; j<maxIdx; j++)
    {
      double sdf = data[j] - minDist;
      isVisible = isVisible || (sdf >= -_maxTruncation);
    }
    if(!isVisible) continue;

    // Check if all cells are in empty space
    bool isEmpty = true;
    for(int j=minIdx; j<maxIdx; j++)
    {
      double sdf = data[j] - maxDist;
      isEmpty = isEmpty && (sdf > _maxTruncation);
    }

    if(isEmpty)
    {
      part->increaseEmptiness();
      continue;
    }

    part->init();

    Matrix* cellCoordsHom = part->getCellCoordsHom();
    sensor->backProject(cellCoordsHom, idx);

    for(unsigned int c=0; c<partSize; c++)
    {
      // Index of laser beam
      int index = idx[c];

      if(index>=0)
      {
        if(mask[index])
        {
          // calculate distance of current cell to sensor
          double crd[2];
          crd[0] = (*cellCoordsHom)(c,0);
          crd[1] = (*cellCoordsHom)(c,1);
          double distance = euklideanDistance<double>(tr, crd, 2);
          double sdf = data[index] - distance;

          part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), sdf, _maxTruncation);
        }
      }
    }
  }
  delete [] idx;
  }

  propagateBorders();

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

void TsdGrid::propagateBorders()
{
  unsigned int width  = _partitions[0][0]->getWidth();
  unsigned int height = _partitions[0][0]->getHeight();

  // Copy valid tsd values of neighbors to borders of each partition.
  // Skip outmost partitions for the moment, they are negligible.
  for(int py=1; py<_partitionsInY-1; py++)
  {
    for(int px=1; px<_partitionsInX-1; px++)
    {
      TsdGridPartition* partCur       = _partitions[py][px];
      TsdGridPartition* partLeft      = _partitions[py][px-1];
      TsdGridPartition* partRight     = _partitions[py][px+1];
      TsdGridPartition* partUp        = _partitions[py+1][px];
      TsdGridPartition* partDown      = _partitions[py-1][px];
      TsdGridPartition* partUpLeft    = _partitions[py+1][px-1];
      TsdGridPartition* partDownLeft  = _partitions[py-1][px-1];
      TsdGridPartition* partUpRight   = _partitions[py+1][px+1];
      TsdGridPartition* partDownRight = _partitions[py-1][px+1];

      if(!partCur->isInitialized()) continue;

      if(partLeft->isInitialized())
      {
        // Copy left border
        for(unsigned int i=1; i<height+1; i++)
        {
          partCur->_grid[i][0].tsd = partLeft->_grid[i][width].tsd;
          partCur->_grid[i][0].weight = partLeft->_grid[i][width].weight;
        }
      }

      if(partRight->isInitialized())
      {
        // Copy right border
        for(unsigned int i=1; i<height+1; i++)
        {
          partCur->_grid[i][width+1].tsd = partRight->_grid[i][1].tsd;
          partCur->_grid[i][width+1].weight = partRight->_grid[i][1].weight;
        }
      }

      if(partUp->isInitialized())
      {
        // Copy upper border
        for(unsigned int i=1; i<width+1; i++)
        {
          partCur->_grid[height+1][i].tsd = partUp->_grid[1][i].tsd;
          partCur->_grid[height+1][i].weight = partUp->_grid[1][i].weight;
        }
      }

      if(partDown->isInitialized())
      {
        // Copy lower border
        for(unsigned int i=1; i<width+1; i++)
        {
          partCur->_grid[0][i].tsd = partDown->_grid[height][i].tsd;
          partCur->_grid[0][i].weight = partDown->_grid[height][i].weight;
        }
      }

      if(partUpLeft->isInitialized())
      {
        // Copy upper left corner
        partCur->_grid[height+1][0].tsd = partUpLeft->_grid[1][width].tsd;
        partCur->_grid[height+1][0].weight = partUpLeft->_grid[1][width].weight;
      }

      if(partDownLeft->isInitialized())
      {
        // Copy lower left corner
        partCur->_grid[0][0].tsd = partDownLeft->_grid[height][width].tsd;
        partCur->_grid[0][0].weight = partDownLeft->_grid[height][width].weight;
      }

      if(partUpRight->isInitialized())
      {
        // Copy upper right corner
        partCur->_grid[height+1][width+1].tsd = partUpRight->_grid[1][1].tsd;
        partCur->_grid[height+1][width+1].weight = partUpRight->_grid[1][1].weight;
      }

      if(partDownRight->isInitialized())
      {
        // Copy lower right corner
        partCur->_grid[0][width+1].tsd = partDownRight->_grid[height][1].tsd;
        partCur->_grid[0][width+1].weight = partDownRight->_grid[height][1].weight;
      }
    }
  }
}

void TsdGrid::grid2ColorImage(unsigned char* image, unsigned int width, unsigned int height)
{
  const double MAX_DIST = 10.0;
  unsigned char rgb[3];

  double stepW = getMaxX() / (double)width;
  double stepH = getMaxY() / (double)height;

  double py = 0.0;
  unsigned int i = 0;
  for(unsigned int h=0; h<height; h++)
  {
    double px = 0.0;
    for(unsigned int w=0; w<width; w++, i++)
    {
      double coord[2];
      coord[0] = px;
      coord[1] = py;
      int p, x, y;
      double dx, dy;
      double tsd = NAN;
      if(coord2Cell(coord, &p, &x, &y, &dx, &dy))
      {
        if(_partitions[0][p]->isInitialized())
          tsd = _partitions[0][p]->_grid[y][x].tsd;
      }
      if(tsd>0.0)
      {
        rgb[0] = static_cast<unsigned char>(tsd * (255.0 / MAX_DIST));
        rgb[1] = 255; //max showable distance MAX_DIST m
        rgb[2] = static_cast<unsigned char>(tsd * (255.0 / MAX_DIST));
      }
      else if(tsd<0.0 && tsd>-0.999)
      {
        rgb[0] = static_cast<unsigned char>(tsd * (255.0));
        rgb[1] = 0;
        rgb[2] = 0;
      }
      else
      {
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
      }
      memcpy(&image[3*i], rgb, 3*sizeof(unsigned char));
      px += stepW;
    }
    py += stepH;
  }
}

bool TsdGrid::interpolateNormal(const double* coord, double* normal)
{
  double neighbor[3];
  double depthInc = 0;
  double depthDec = 0;

  neighbor[0] = coord[0] + _cellSize;
  neighbor[1] = coord[1];
  if(!interpolateBilinear(neighbor, &depthInc)) return false;

  neighbor[0] = coord[0] - _cellSize;
  // neighbor[1] = coord[1];
  if(!interpolateBilinear(neighbor, &depthDec)) return false;

  normal[0] = depthInc - depthDec;

  neighbor[0] = coord[0];
  neighbor[1] = coord[1] + _cellSize;
  if(!interpolateBilinear(neighbor, &depthInc)) return false;

  // neighbor[0] = coord[0];
  neighbor[1] = coord[1] - _cellSize;
  if(!interpolateBilinear(neighbor, &depthDec)) return false;

  normal[1] = depthInc - depthDec;

  norm2<double>(normal);

  return true;
}

bool TsdGrid::interpolateBilinear(double coord[2], double* tsdf)
{
  int p;
  int x;
  int y;
  double dx;
  double dy;

  if(!coord2Cell(coord, &p, &x, &y, &dx, &dy)) return false;

  double wx = fabs((coord[0] - dx) / (_cellSize));
  double wy = fabs((coord[1] - dy) / (_cellSize));

  *tsdf = _partitions[0][p]->interpolateBilinear(x, y, wx, wy);

  return (!isnan(*tsdf));
}

bool TsdGrid::coord2Cell(double coord[2], int* p, int* x, int* y, double* dx, double* dy)
{
  // Get cell indices
  double dCoordX = coord[0] * _invCellSize;
  double dCoordY = coord[1] * _invCellSize;

  int xIdx = floor(dCoordX);
  int yIdx = floor(dCoordY);

  // Get center point of current cell
  *dx = (double(xIdx) + 0.5) * _cellSize;
  *dy = (double(yIdx) + 0.5) * _cellSize;

  // Ensure that query point has 4 neighbors for bilinear interpolation
  if (coord[0] < *dx)
  {
    xIdx--;
    (*dx) -= _cellSize;
  }
  if (coord[1] > *dy)
  {
    yIdx++;
    (*dy) += _cellSize;
  }

  // Check boundaries
  if ((xIdx >= _cellsX) || (xIdx < 0) || (yIdx >= _cellsY) || (yIdx < 0))
    return false;

  *p = yIdx / _dimPartition * _partitionsInX + xIdx / _dimPartition;
  if(!(_partitions[0][*p]->isInitialized())) return false;

  *x = xIdx % _dimPartition;
  *y = yIdx % _dimPartition;

  return true;
}

}
