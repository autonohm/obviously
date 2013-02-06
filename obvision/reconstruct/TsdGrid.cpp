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

TsdGrid::TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize, const unsigned int beams, double angularRes)
{
   _cellSize = cellSize;
   _invCellSize = 1.0 / _cellSize;

   _cellsX = ((double)dimX * _invCellSize + 0.5);
   _cellsY = ((double)dimY * _invCellSize + 0.5);
   _sizeOfGrid = _cellsY * _cellsX;

   _dimX = dimX;
   _dimY = dimY;
   _maxTruncation = 2*cellSize;

   _phiMin = -angularRes * (double)((beams-1)/2);
   _phiMax =  angularRes * (double)((beams-1)/2);
   _angularRes = angularRes;

   LOGMSG(DBG_DEBUG, "Grid dimensions are (x/y) (" << _cellsX << "/" << _cellsY << ")");

   System<TsdCell>::allocate(_cellsY, _cellsX, _grid);

   _cellCoords = new Matrix(_sizeOfGrid, 3);

   int i=0;
   for (int y = 0; y < _cellsY; y++)
   {
      for (int x = 0; x < _cellsX; x++, i++)
      {
         _grid[y][x].tsdf   = 1.0;
         _grid[y][x].weight = 0.0;
      }
   }

   _T = new Matrix(3, 3);
   _T->setIdentity();
   double Tinit[6];
   Tinit[0] = 1.0;  Tinit[1] = 0.0;  Tinit[2] = ((double)dimX) * 0.5;
   Tinit[3] = 0.0;  Tinit[4] = 1.0;  Tinit[5] = ((double)dimY) * 0.5;
   setTransformation(Tinit);
}

TsdGrid::~TsdGrid(void)
{
   delete _Tinv;
   delete _T;
   delete _cellCoords;
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

void TsdGrid::setTransformation(double TData[6])
{
   Matrix T(3, 3, TData);
   (*_T) *= T;

   (*_Tinv) = _T->getInverse();

   _tr[0] = (*_T)[0][2];
   _tr[1] = (*_T)[1][2];
}

double* TsdGrid::getTransformation()
{
   return _T->getBuffer()->data;
}

void TsdGrid::push(double* depthArray, bool* mask)
{
   Timer t;

   for(int y=0; y<_cellsY; y++)
   {
      int i = y*_cellsX;
      for(int x=0; x<_cellsX; x++, i++)
      {
         double cellCoords[2];
         cellCoords[0] = ((double)x + 0.5) * _cellSize;
         cellCoords[1] = ((double)y + 0.5) * _cellSize;

         double phi = atan2(cellCoords[1],cellCoords[0]);
         double r   = abs2D(cellCoords);

         if(phi>_phiMin && phi<_phiMax)
         {
            unsigned int index = phi2Index(phi);
            addTsdfValue(cellCoords, x, y, depthArray[index]);
         }
      }
   }

   LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
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

   double wx = dx - (double)x;
   double wy = dy - (double)y;

   // Interpolate
   *tsdf =    _grid[y + 0][x + 0].tsdf * (1. - wx) * (1. - wy)
            + _grid[y - 1][x + 0].tsdf * (1. - wx) * wy
            + _grid[y + 0][x + 1].tsdf * wx * (1. - wy)
            + _grid[y - 1][x + 1].tsdf * wx * wy;

   return true;
}

void TsdGrid::addTsdfValue(const double cellCoords[2], const unsigned int x, const unsigned int y, const double depth)
{
   // calculate distance of current cell to sensor
   double distance = euklideanDistance<double>(_tr, (double*)cellCoords, 2);
   double sdf = depth - distance;

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

inline unsigned int TsdGrid::phi2Index(double phi)
{
   return (phi - _phiMin)/_angularRes;
}

inline bool TsdGrid::coord2Cell(double coord[2], int* x, int* y, double* dx, double* dy)
{
   // initialize
   // get cell indices
   int xIdx = (int) (coord[0] * _invCellSize);
   int yIdx = (int) (coord[1] * _invCellSize);

   // check edges / 0 is edge because of cell fine tuning
   if ((xIdx >= (_cellsX - 2)) || (xIdx < 1) || (yIdx >= (_cellsY - 2)) || (yIdx < 1))
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
