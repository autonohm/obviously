#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGrid.h"
#include "TsdGridBranch.h"

#include <cstring>
#include <cmath>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 32.0

TsdGrid::TsdGrid(const double cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid)
{
  _cellSize = cellSize;
  _invCellSize = 1.0 / _cellSize;

  _cellsX = (unsigned int)pow(2.0,layoutGrid);
  _cellsY = _cellsX;

  _dimPartition = (unsigned int)pow(2.0,layoutPartition);

  if(_dimPartition > _cellsX)
  {
    LOGMSG(DBG_ERROR, "Insufficient partition size : " << _dimPartition << "x" << _dimPartition << " in "
                                                       << _cellsX << "x" << _cellsY << " grid");
    return;
  }

  _partitionsInX = _cellsX/_dimPartition;
  _partitionsInY = _cellsY/_dimPartition;

  _sizeOfGrid = _cellsY * _cellsX;

  _maxTruncation = 2.0*cellSize;

  LOGMSG(DBG_DEBUG, "Grid dimensions: " << _cellsX << "x" << _cellsY << " cells" <<
                    " = " << ((double)_cellsX)*cellSize << "x" << ((double)_cellsY)*cellSize << " sqm");

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

  int depthTree = layoutGrid-layoutPartition;
  if(depthTree == 0)
  {
    _tree = _partitions[0][0];
  }
  else
  {
    TsdGridBranch* tree = new TsdGridBranch((TsdGridComponent***)_partitions, 0, 0, depthTree);
    _tree = tree;
  }
}

TsdGrid::~TsdGrid(void)
{
  delete _tree;
  System<TsdGridPartition*>::deallocate(_partitions);
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

unsigned int TsdGrid::getPartitionSize()
{
  return _partitions[0][0]->getWidth();
}

void TsdGrid::getCentroid(double centroid[2])
{
  centroid[0] = (_minX + _maxX) / 2.0;
  centroid[1] = (_minY + _maxY) / 2.0;
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
  double* data     = sensor->getRealMeasurementData();
  bool*   mask     = sensor->getRealMeasurementMask();

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
    if(!part->isInRange(tr, sensor, _maxTruncation)) continue;

    part->init();

    Matrix* partCoords = part->getPartitionCoords();
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
          double sd = data[index] - distance;

          part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), sd, _maxTruncation);
        }
      }
    }
  }
  delete [] idx;
  }

  propagateBorders();

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

void TsdGrid::pushTree(SensorPolar2D* sensor)
{
  Timer t;
  double* data     = sensor->getRealMeasurementData();
  bool*   mask     = sensor->getRealMeasurementMask();

  double tr[2];
  sensor->getPosition(tr);

  TsdGridComponent* comp = _tree;
  vector<TsdGridPartition*> partitionsToCheck;
  pushRecursion(sensor, tr, comp, partitionsToCheck);

  LOGMSG(DBG_DEBUG, "Partitions to check: " << partitionsToCheck.size());

#pragma omp parallel
  {
  unsigned int partSize = (_partitions[0][0])->getSize();
  int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
  for(unsigned int i=0; i<partitionsToCheck.size(); i++)
  {
    TsdGridPartition* part = partitionsToCheck[i];
    part->init();

    Matrix* partCoords = part->getPartitionCoords();
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

void TsdGrid::pushRecursion(SensorPolar2D* sensor, double pos[2], TsdGridComponent* comp, vector<TsdGridPartition*> &partitionsToCheck)
{
  if(comp->isInRange(pos, sensor, _maxTruncation))
  {
    if(comp->isLeaf())
        partitionsToCheck.push_back((TsdGridPartition*)comp);
    else
    {
      vector<TsdGridComponent*> children = ((TsdGridBranch*)comp)->getChildren();
      for(unsigned int i=0; i<children.size(); i++)
        pushRecursion(sensor, pos, children[i], partitionsToCheck);
    }
  }
}

void TsdGrid::propagateBorders()
{
  unsigned int width  = _partitions[0][0]->getWidth();
  unsigned int height = _partitions[0][0]->getHeight();

  // Copy valid tsd values of neighbors to borders of each partition.
  // Skip outmost partitions for the moment, they are negligible.
  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      TsdGridPartition* partCur       = _partitions[py][px];

      if(!partCur->isInitialized()) continue;

      if(px<(_partitionsInX-1))
      {
        TsdGridPartition* partRight     = _partitions[py][px+1];
        if(partRight->isInitialized())
        {
          // Copy right border
          for(unsigned int i=0; i<height; i++)
          {
            partCur->_grid[i][width].tsd = partRight->_grid[i][0].tsd;
            partCur->_grid[i][width].weight = partRight->_grid[i][0].weight;
          }
        }
      }

      if(py<(_partitionsInY-1))
      {
        TsdGridPartition* partUp        = _partitions[py+1][px];
        if(partUp->isInitialized())
        {
          // Copy upper border
          for(unsigned int i=0; i<width; i++)
          {
            partCur->_grid[height][i].tsd = partUp->_grid[0][i].tsd;
            partCur->_grid[height][i].weight = partUp->_grid[0][i].weight;
          }
        }
      }

      if(px<(_partitionsInX-1) && py<(_partitionsInY-1))
      {
        TsdGridPartition* partUpRight   = _partitions[py+1][px+1];
        if(partUpRight->isInitialized())
        {
          // Copy upper right corner
          partCur->_grid[height][width].tsd = partUpRight->_grid[0][0].tsd;
          partCur->_grid[height][width].weight = partUpRight->_grid[0][0].weight;
        }
      }
    }
  }
}

void TsdGrid::grid2ColorImage(unsigned char* image, unsigned int width, unsigned int height)
{
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
      bool isEmpty = false;

      if(coord2Cell(coord, &p, &x, &y, &dx, &dy))
      {
        if(_partitions[0][p]->isInitialized())
          tsd = _partitions[0][p]->_grid[y][x].tsd;

        isEmpty = _partitions[0][p]->isEmpty();
      }

      if(tsd>0.0)
      {
        rgb[0] = static_cast<unsigned char>(tsd * 150.0);
        rgb[1] = 255;
        rgb[2] = static_cast<unsigned char>(tsd * 150.0);
      }
      else if(tsd<0.0)
      {
        rgb[0] = static_cast<unsigned char>(tsd * 255.0);
        rgb[1] = 0;
        rgb[2] = 0;
      }
      else if(isEmpty)
      {
        rgb[0] = 255;
        rgb[1] = 255;
        rgb[2] = 255;
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

void TsdGrid::getData(std::vector<double>& data)
{
  double coordVar[2] = {0.0};
  int p = 0;
  int x = 0;
  int y = 0;
  double dx = 0.0;
  double dy = 0.0;
  double tsd = 0.0;
  for(coordVar[1] = 0.0; coordVar[1] < _maxY; coordVar[1] += _cellSize)
  {
    for(coordVar[0] = 0.0; coordVar[0] < _maxX; coordVar[0] += _cellSize)
    {
      if(this->coord2Cell(coordVar, &p, &x, &y, &dx, &dy))
      {
        if(_partitions[0][p]->isInitialized())
          tsd = _partitions[0][p]->_grid[y][x].tsd;
        else
          tsd = NAN;
      }
      else
        tsd = NAN;
      data.push_back(tsd);
    }
  }
}

bool TsdGrid::interpolateNormal(const double* coord, double* normal)
{
  double neighbor[3];
  double depthInc = 0;
  double depthDec = 0;

  neighbor[0] = coord[0] + _cellSize;
  neighbor[1] = coord[1];
  if(interpolateBilinear(neighbor, &depthInc)!=INTERPOLATE_SUCCESS) return false;

  neighbor[0] = coord[0] - _cellSize;
  // neighbor[1] = coord[1];
  if(interpolateBilinear(neighbor, &depthDec)!=INTERPOLATE_SUCCESS) return false;

  normal[0] = depthInc - depthDec;

  neighbor[0] = coord[0];
  neighbor[1] = coord[1] + _cellSize;
  if(interpolateBilinear(neighbor, &depthInc)!=INTERPOLATE_SUCCESS) return false;

  // neighbor[0] = coord[0];
  neighbor[1] = coord[1] - _cellSize;
  if(interpolateBilinear(neighbor, &depthDec)!=INTERPOLATE_SUCCESS) return false;

  normal[1] = depthInc - depthDec;

  norm2<double>(normal);

  return true;
}

EnumTsdGridInterpolate TsdGrid::interpolateBilinear(double coord[2], double* tsd)
{
  int p;
  int x;
  int y;
  double dx;
  double dy;

  if(!coord2Cell(coord, &p, &x, &y, &dx, &dy)) return INTERPOLATE_INVALIDINDEX;
  if(!_partitions[0][p]->isInitialized()) return INTERPOLATE_EMPTYPARTITION;

  double wx = fabs((coord[0] - dx) / (_cellSize));
  double wy = fabs((coord[1] - dy) / (_cellSize));

  *tsd = _partitions[0][p]->interpolateBilinear(x, y, wx, wy);

  if(isnan(*tsd))
    return INTERPOLATE_ISNAN;

  return INTERPOLATE_SUCCESS;
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
  if (coord[1] < *dy)
  {
    yIdx--;
    (*dy) -= _cellSize;
  }

  // Check boundaries
  if ((xIdx >= _cellsX) || (xIdx < 0) || (yIdx >= _cellsY) || (yIdx < 0))
    return false;

  *p = yIdx / _dimPartition * _partitionsInX + xIdx / _dimPartition;
  *x = xIdx % _dimPartition;
  *y = yIdx % _dimPartition;

  return true;
}

TsdGridPartition*** TsdGrid::getPartitions()
{
  return _partitions;
}

bool TsdGrid::isInsideGrid(Sensor* sensor)
{
  double coord[2];
  sensor->getPosition(coord);
  return (coord[0]>_minX && coord[0]<_maxX && coord[1]>_minY && coord[1]<_maxY);
}

}
