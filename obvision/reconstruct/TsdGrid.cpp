#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/Matrix.h"
#include "obcore/math/mathbase.h"
#include "TsdGrid.h"

#include <cstring>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 128.0

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

   int i=0;
   for (int y = 0; y < _cellsY; y++)
   {
      for (int x = 0; x < _cellsX; x++, i++)
      {
         _grid[y][x].tsdf   = 1.0;
         _grid[y][x].weight = 0.0;
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
   bool* mask = sensor->getRealMeasurementMask();
   double tr[2];
   sensor->getPosition(tr);

   for(int y=0; y<_cellsY; y++)
   {
      int i = y*_cellsX;
      for(int x=0; x<_cellsX; x++, i++)
      {
         double cellCoords[2];
         cellCoords[0] = ((double)x + 0.5) * _cellSize;
         cellCoords[1] = ((double)y + 0.5) * _cellSize;

         int index = sensor->backProject(cellCoords);

         if(index>0)
         {
            if(mask[index])
            {
               // calculate distance of current cell to sensor
               double distance = euklideanDistance<double>(tr, (double*)cellCoords, 2);
               double sdf = data[index] - distance;
               addTsdfValue(x, y, sdf);
            }
         }
      }
   }

   LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

void TsdGrid::grid2GrayscaleImage(unsigned char* image)
{
   for(int y=0; y<_cellsY; y++)
   {
      int i = y*_cellsX;
      for(int x=0; x<_cellsX; x++, i++)
      {
         image[i] = (unsigned char)((_grid[y][x].tsdf * 127.0) + 128.0);
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

   double wx = (coord[0] - dx) / _cellSize;
   double wy = (coord[1] - dy) / _cellSize;

   // Interpolate
   *tsdf =    _grid[y + 0][x + 0].tsdf * (1. - wx) * (1. - wy)
            + _grid[y - 1][x + 0].tsdf * (1. - wx) * wy
            + _grid[y + 0][x + 1].tsdf * wx * (1. - wy)
            + _grid[y - 1][x + 1].tsdf * wx * wy;

   return true;
}

void TsdGrid::addTsdfValue(const unsigned int x, const unsigned int y, const double sdf)
{
   // determine whether sdf/max_truncation = ]-1;1[
   if(sdf >= -_maxTruncation)
   {
      TsdCell* cell = &_grid[y][x];

      double tsdf = sdf / _maxTruncation;
      tsdf = min(tsdf, 1.0);

      cell->weight += 1.0;
      const double invWeight = 1.0 / cell->weight;
      cell->tsdf   = (cell->tsdf * (cell->weight - 1.0) + tsdf) * invWeight;
      cell->weight = min(cell->weight, MAXWEIGHT);
   }
}

inline bool TsdGrid::coord2Cell(double coord[2], int* x, int* y, double* dx, double* dy)
{
   // initialize
   // get cell indices
   int xIdx = (int) (coord[0] * _invCellSize);
   int yIdx = (int) (coord[1] * _invCellSize);

   // check edges / 0 is edge because of cell fine tuning
   if ((xIdx >= (_cellsX - 2)) || (xIdx <= 1) || (yIdx >= (_cellsY - 2)) || (yIdx <= 1))
      return false;

   // get center point of current cell
   *dx = (double(xIdx) + 0.5) * _cellSize;
   *dy = (double(yIdx) + 0.5) * _cellSize;

   // cell fine tuning -> shift to lower-left-front edge
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

   *x = xIdx;
   *y = yIdx;

   return true;
}

}
