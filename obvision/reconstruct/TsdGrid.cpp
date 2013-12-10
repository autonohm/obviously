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

TsdGrid::TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize, const unsigned int dimPartition)
{
  _partitions.clear();

  _cellSize = cellSize;
  _invCellSize = 1.0 / _cellSize;

  _cellsX = ((double)dimX * _invCellSize + 0.5);
  _cellsY = ((double)dimY * _invCellSize + 0.5);
  _sizeOfGrid = _cellsY * _cellsX;

  _dimX = dimX;
  _dimY = dimY;
  _maxTruncation = 2*cellSize;

  LOGMSG(DBG_DEBUG, "Grid dimensions are (x/y) (" << _cellsX << "/" << _cellsY << ")");

  System<TsdCell>::allocate(_cellsY, _cellsX, _grid);

  int i=0;
  for (int y = 0; y < _cellsY; y++)
  {
    for (int x = 0; x < _cellsX; x++, i++)
    {
      _grid[y][x].tsdf   = NAN;
      _grid[y][x].weight = 0.0;
    }
  }

  _minX = 0.0;
  _maxX = ((double)_cellsX + 0.5) * _cellSize;
  _minY = 0.0;
  _maxY = ((double)_cellsY + 0.5) * _cellSize;

  unsigned int partSizeX = _cellsX/dimPartition;
  unsigned int partSizeY = _cellsY/dimPartition;
  for(unsigned int py=0; py<dimPartition; py++)
  {
    for(unsigned int px=0; px<dimPartition; px++)
    {
      TsdGridPartition* partition = new TsdGridPartition(px*partSizeX, py*partSizeY, partSizeX, partSizeY, cellSize);
      _partitions.push_back(partition);
    }
  }
}

TsdGrid::~TsdGrid(void)
{
  delete [] _grid;
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
  double* accuracy = sensor->getRealMeasurementAccuracy();

  double tr[2];
  sensor->getPosition(tr);

#pragma omp parallel
{
  Matrix cellCoordsHom(_cellsX, 3);
  double dx = 0.5 * _cellSize;
  for(int x=0; x<_cellsX; x++)
  {
    cellCoordsHom(x,0) = (dx += _cellSize);
    cellCoordsHom(x,2) = 1.0;
  }

#pragma omp for schedule(dynamic)
  for(int y=0; y<_cellsY; y++)
  {
    int idx[_cellsX];
    for(int x=0; x<_cellsX; x++)
      cellCoordsHom(x,1) = ((double)y + 0.5) * _cellSize;
    sensor->backProject(&cellCoordsHom, idx);

    for(int x=0; x<_cellsX; x++)
    {
      // Index of laser beam
      int index = idx[x];

      if(index>=0)
      {
        if(mask[index])
        {
          // calculate distance of current cell to sensor
          double crd[2];
          crd[0] = cellCoordsHom(x,0);
          crd[1] = cellCoordsHom(x,1);
          double distance = euklideanDistance<double>(tr, crd, 2);
          double sdf = data[index] - distance;
          double weight = 1.0;
          if(accuracy) weight = accuracy[index];
          addTsdfValue(x, y, sdf, weight);
        }
      }
    }
  }
}

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

void TsdGrid::pushPartitioned(SensorPolar2D* sensor)
{
  Timer t;
  double* data = sensor->getRealMeasurementData();
  bool*   mask = sensor->getRealMeasurementMask();
  double* accuracy = sensor->getRealMeasurementAccuracy();

  double tr[2];
  sensor->getPosition(tr);

#pragma omp parallel
  {
  unsigned int partSize = _partitions[0]->getSize();
  int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
  for(unsigned int i=0; i<_partitions.size(); i++)
  {
    TsdGridPartition* part = _partitions[i];
    Matrix* edgeCoordsHom = part->getEdgeCoordsHom();
    Matrix* cellCoordsHom = part->getCellCoordsHom();
    Matrix* gridCoords = part->getGridCoords();

    // Project back edge coordinates of partition
    int idxEdge[4];
    sensor->backProject(edgeCoordsHom, idxEdge);
    sort4(idxEdge);

    // Check whether non of the cells are in the field of view
    if(idxEdge[3]<0) continue;

    if(idxEdge[0]>0)
    {
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
      for(int j=idxEdge[0]; j<idxEdge[3]; j++)
      {
        double sdf = data[j] - minDist;
        isVisible = isVisible || (sdf >= -_maxTruncation);
      }
      if(!isVisible) continue;

      // Check if all cells are in empty space
      bool isEmpty = true;
      for(int j=idxEdge[0]; j<idxEdge[3]; j++)
      {
        double sdf = data[j] - maxDist;
        isEmpty = isEmpty && (sdf > _maxTruncation);
      }

      if(isEmpty)
      {
        for(unsigned int c=0; c<partSize; c++)
          addTsdfValueEmptyCell((*gridCoords)(c, 0), (*gridCoords)(c, 1));
        continue;
      }
    }

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
          double weight = 1.0;
          if(accuracy) weight = accuracy[index];
          addTsdfValue((*gridCoords)(c, 0), (*gridCoords)(c, 1), sdf, weight);
        }
      }
    }
  }
  delete [] idx;
  }

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

void TsdGrid::grid2GrayscaleImage(unsigned char* image)
{
  for(int y=0; y<_cellsY; y++)
  {
    int i = (_cellsY-1-y)*_cellsX;
    for(int x=0; x<_cellsX; x++, i++)
    {
      image[i] = (unsigned char)((_grid[y][x].tsdf * 127.0) + 128.0);
    }
  }
}

void TsdGrid::grid2ColorImage(unsigned char* image)
{
  const double MAX_DIST = 10.0;
  unsigned char rgb[3];
  for(int y=0; y<_cellsY; y++)
  {
    int i = (_cellsY-1-y)*_cellsX;
    for(int x=0; x<_cellsX; x++, i++)
    {

      double tsd = _grid[y][x].tsdf;
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
  int x;
  int y;
  double dx;
  double dy;

  if(!coord2Cell(coord, &x, &y, &dx, &dy)) return false;

  double tsdf_cell = _grid[y][x].tsdf;
  if(isnan(tsdf_cell)) return false;

  double wx = fabs((coord[0] - dx) / (_cellSize));
  double wy = fabs((coord[1] - dy) / (_cellSize));

  // Interpolate
  *tsdf =    tsdf_cell * (1. - wy) * (1. - wx)
                            + _grid[y - 1][x + 0].tsdf *       wy  * (1. - wx)
                            + _grid[y + 0][x + 1].tsdf * (1. - wy) *       wx
                            + _grid[y - 1][x + 1].tsdf *       wy  *       wx;

  return (!isnan(*tsdf));
}

void TsdGrid::addTsdfValue(const unsigned int x, const unsigned int y, const double sdf, const double weight)
{
  if(sdf >= -_maxTruncation)
  {
    TsdCell* cell = &_grid[y][x];

    double tsdf = sdf;
    tsdf /= _maxTruncation;
    tsdf = min(tsdf, 1.0);

    cell->weight += 1.0;

    if(isnan(cell->tsdf))
    {
      cell->tsdf = tsdf;
    }
    else
    {
      cell->weight = min(cell->weight, MAXWEIGHT);
      cell->tsdf   = (cell->tsdf * (cell->weight - 1.0) + tsdf) / cell->weight;
    }
  }
}

void TsdGrid::addTsdfValueEmptyCell(const unsigned int x, const unsigned int y)
{
  TsdCell* cell = &_grid[y][x];
  cell->weight += 1.0;

  if(isnan(cell->tsdf))
  {
    cell->tsdf = 1.0;
  }
  else
  {
    cell->weight = min(cell->weight, MAXWEIGHT);
    cell->tsdf   = (cell->tsdf * (cell->weight - 1.0) + 1.0) / cell->weight;
  }
}

bool TsdGrid::coord2Cell(double coord[2], int* x, int* y, double* dx, double* dy)
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
  if ((xIdx > (_cellsX - 2)) || (xIdx < 0) || (yIdx > (_cellsY - 1)) || (yIdx < 1))
    return false;

  *x = xIdx;
  *y = yIdx;

  return true;
}

void TsdGrid::serialize(const char* filename)
{
  ofstream f;
  f.open(filename);

  for(int y=0; y<_cellsY; y++)
  {
    for(int x=0; x<_cellsX; x++)
    {
      double tsdf = _grid[y][x].tsdf;
      if(!isnan(tsdf))
        f << y << " " << x << " " << tsdf << " " << _grid[y][x].weight << endl;

    }
  }
  f.close();
}

void TsdGrid::load(const char* filename)
{
  char line[256];
  double weight, tsdf;
  unsigned int x, y;
  ifstream f;

  f.open(filename, ios_base::in);

  if(!f)
  {
    std::cout << filename << " is no file!" << std::endl;
    abort();
  }

  do
  {
    f >> y >> x >> tsdf >> weight;
    TsdCell* cell = &_grid[y][x];
    cell->weight  = weight;
    cell->tsdf    = tsdf;

  }while(f.getline(line, 256).good());

  f.close();
}

TsdCell** TsdGrid::getData() const
{
  return(_grid);
}

}
