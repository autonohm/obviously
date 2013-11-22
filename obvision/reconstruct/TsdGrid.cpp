#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/Matrix.h"
#include "obcore/math/mathbase.h"
#include "TsdGrid.h"

#include <cstring>
#include <cmath>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 32.0

TsdGrid::TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize)
{
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

  _cellCoordsHom = new Matrix(_sizeOfGrid, 3);

  int i=0;
  for (int y = 0; y < _cellsY; y++)
  {
    for (int x = 0; x < _cellsX; x++, i++)
    {
      _grid[y][x].tsdf   = NAN;
      _grid[y][x].weight = 0.0;
      (*_cellCoordsHom)[i][0] = ((double)x + 0.5) * _cellSize;
      (*_cellCoordsHom)[i][1] = ((double)y + 0.5) * _cellSize;
      (*_cellCoordsHom)[i][2] = 1.0;
    }
  }

  _minX = 0.0;
  _maxX = ((double)_cellsX + 0.5) * _cellSize;
  _minY = 0.0;
  _maxY = ((double)_cellsY + 0.5) * _cellSize;
}

TsdGrid::~TsdGrid(void)
{
  delete [] _grid;
  delete _cellCoordsHom;
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

  int* indices = new int[_sizeOfGrid];
  sensor->backProject(_cellCoordsHom, indices);

  int i = 0;
  for(int y=0; y<_cellsY; y++)
  {
    for(int x=0; x<_cellsX; x++, i++)
    {
      // Index of laser beam
      int index = indices[i];

      if(index>=0)
      {
        if(mask[index])
        {
          // calculate distance of current cell to sensor
          double distance = euklideanDistance<double>(tr, (*_cellCoordsHom)[i], 2);
          double sdf = data[index] - distance;
          double weight = 1.0;
          if(accuracy) weight = accuracy[index];
          addTsdfValue(x, y, sdf, weight);
        }
      }
    }
  }

  delete [] indices;
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
  unsigned char rgb[3];
  for(int y=0; y<_cellsY; y++)
  {
    int i = (_cellsY-1-y)*_cellsX;
    for(int x=0; x<_cellsX; x++, i++)
    {

      double tsd = _grid[y][x].tsdf;
      if(tsd>0.0 && tsd<0.999)
      {
        rgb[0] = 127;
        rgb[1] = 127 + (unsigned char)(128.0*tsd);
        rgb[2] = 127;
      }
      else if(tsd<0.0 && tsd>-0.999)
      {
        rgb[0] = 127 + (unsigned char)(-128.0*tsd);
        rgb[1] = 127;
        rgb[2] = 127;
      }
      else
      {
        rgb[0] = 255;
        rgb[1] = 255;
        rgb[2] = 255;
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
  // Determine whether sdf/max_truncation = ]-1;1[
  if(sdf >= -_maxTruncation)
  {
    TsdCell* cell = &_grid[y][x];
    double tsdf = 0.0;
    if(sdf < 0.0)
    {
      tsdf = sdf / _maxTruncation;
      tsdf = min(tsdf, 1.0);
    }
    else
      tsdf = sdf;

    cell->weight += 1.0;

    if(isnan(cell->tsdf))
    {
      cell->tsdf = tsdf;
    }
    else
    {
      const double invWeight = 1.0 / cell->weight;
      cell->tsdf   = (cell->tsdf * (cell->weight - 1.0) + tsdf) * invWeight;
      cell->weight = min(cell->weight, MAXWEIGHT);
    }
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

void TsdGrid::Load(const char* filename)
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
obvious::TsdCell** TsdGrid::getData(void)const
{
  return(_grid);
}

}
