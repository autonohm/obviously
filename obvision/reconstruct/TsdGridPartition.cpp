#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGridPartition.h"

#include <cstring>
#include <cmath>

#define MAXWEIGHT 32.0

namespace obvious
{

static Matrix* _partCoords;

TsdGridPartition::TsdGridPartition(const unsigned int x, const unsigned int y, const unsigned int cellsX, const unsigned int cellsY, const double cellSize)
{
  _x = x;
  _y = y;

  _grid = NULL;
  _cellCoordsHom = NULL;

  _cellSize = cellSize;

  _initWeight = 0.0;

  if(!_partCoords)
  {
    _partCoords = new Matrix(cellsX*cellsY, 2);
    unsigned int i=0;
    for(unsigned int iy=0; iy<cellsY; iy++)
    {
      for(unsigned int ix=0; ix<cellsX; ix++, i++)
      {
        (*_partCoords)(i, 0) = ix + 1;
        (*_partCoords)(i, 1) = iy + 1;
      }
    }
  }

  _edgeCoordsHom = new Matrix(4, 3);
  (*_edgeCoordsHom)(0, 0) = ((double)x + 0.5) * _cellSize;
  (*_edgeCoordsHom)(0, 1) = ((double)y + 0.5) * _cellSize;
  (*_edgeCoordsHom)(0, 2) = 1.0;

  (*_edgeCoordsHom)(1, 0) = ((double)(x+cellsX) + 0.5) * _cellSize;
  (*_edgeCoordsHom)(1, 1) = ((double)y + 0.5) * _cellSize;
  (*_edgeCoordsHom)(1, 2) = 1.0;

  (*_edgeCoordsHom)(2, 0) = ((double)x + 0.5) * _cellSize;
  (*_edgeCoordsHom)(2, 1) = ((double)(y+cellsY) + 0.5) * _cellSize;
  (*_edgeCoordsHom)(2, 2) = 1.0;

  (*_edgeCoordsHom)(3, 0) = ((double)(x+cellsX) + 0.5) * _cellSize;
  (*_edgeCoordsHom)(3, 1) = ((double)(y+cellsY) + 0.5) * _cellSize;
  (*_edgeCoordsHom)(3, 2) = 1.0;

  _centroid[0] = ((*_edgeCoordsHom)(0, 0) + (*_edgeCoordsHom)(1, 0) + (*_edgeCoordsHom)(2, 0) + (*_edgeCoordsHom)(3, 0)) / 4.0;
  _centroid[1] = ((*_edgeCoordsHom)(0, 1) + (*_edgeCoordsHom)(1, 1) + (*_edgeCoordsHom)(2, 1) + (*_edgeCoordsHom)(3, 1)) / 4.0;

  _cellsX = cellsX;
  _cellsY = cellsY;
}

TsdGridPartition::~TsdGridPartition()
{
  if(_grid) System<TsdCell>::deallocate(_grid);
  if(_cellCoordsHom) delete [] _cellCoordsHom;
  delete [] _edgeCoordsHom;
  if(_partCoords)
  {
    delete [] _partCoords;
    _partCoords = NULL;
  }
}

void TsdGridPartition::init()
{
  if(_grid) return;
  System<TsdCell>::allocate(_cellsY+2, _cellsX+2, _grid);
  for (unsigned int iy = 0; iy < _cellsY+2; iy++)
  {
    for (unsigned int ix = 0; ix < _cellsX+2; ix++)
    {
      _grid[iy][ix].tsd   = NAN;
      _grid[iy][ix].weight = _initWeight;
    }
  }

  _cellCoordsHom = new Matrix(_cellsX*_cellsY, 3);
  unsigned int i=0;
  for(unsigned int iy=_y; iy<_y+_cellsY; iy++)
  {
    for(unsigned int ix=_x; ix<_x+_cellsX; ix++, i++)
    {
      (*_cellCoordsHom)(i,0) = ((double)ix + 0.5) * _cellSize;
      (*_cellCoordsHom)(i,1) = ((double)iy + 0.5) * _cellSize;
      (*_cellCoordsHom)(i,2) = 1.0;
    }
  }
}

bool TsdGridPartition::isInitialized()
{
  return _grid!=NULL;
}

double& TsdGridPartition::operator () (unsigned int y, unsigned int x)
{
  return _grid[y+1][x+1].tsd;
}

unsigned int TsdGridPartition::getX()
{
  return _x;
}

unsigned int TsdGridPartition::getY()
{
  return _y;
}

double* TsdGridPartition::getCentroid()
{
  return _centroid;
}

Matrix* TsdGridPartition::getCellCoordsHom()
{
  return _cellCoordsHom;
}

Matrix* TsdGridPartition::getEdgeCoordsHom()
{
  return _edgeCoordsHom;
}

Matrix* TsdGridPartition::getPartitionCoords()
{
  return _partCoords;
}

unsigned int TsdGridPartition::getWidth()
{
  return _cellsX;
}

unsigned int TsdGridPartition::getHeight()
{
  return _cellsY;
}

unsigned int TsdGridPartition::getSize()
{
  return _cellsX*_cellsY;
}

void TsdGridPartition::addTsd(const unsigned int x, const unsigned int y, const double sdf, const double maxTruncation)
{
  if(sdf >= -maxTruncation)
  {
    TsdCell* cell = &_grid[y][x];

    double tsdf = sdf;
    tsdf /= maxTruncation;
    tsdf = min(tsdf, 1.0);

    cell->weight += 1.0;

    if(isnan(cell->tsd))
    {
      cell->tsd = tsdf;
    }
    else
    {
      cell->weight = min(cell->weight, MAXWEIGHT);
      cell->tsd   = (cell->tsd * (cell->weight - 1.0) + tsdf) / cell->weight;
    }
  }
}

void TsdGridPartition::increaseEmptiness()
{
  if(_grid)
  {
    for(unsigned int y=1; y<=_cellsY; y++)
    {
      for(unsigned int x=1; x<=_cellsX; x++)
      {
        TsdCell* cell = &_grid[y][x];
        cell->weight += 1.0;

        if(isnan(cell->tsd))
        {
          cell->tsd = 1.0;
        }
        else
        {
          cell->weight = min(cell->weight, MAXWEIGHT);
          cell->tsd    = (cell->tsd * (cell->weight - 1.0) + 1.0) / cell->weight;
        }
      }
    }
  }
  else
  {
    _initWeight += 1.0;
    _initWeight = min(_initWeight, MAXWEIGHT);
  }
}

double TsdGridPartition::interpolateBilinear(int x, int y, double dx, double dy)
{
  double tsdf_cell = _grid[y+1][x+1].tsd;

  // Interpolate
  return   tsdf_cell * (1. - dy) * (1. - dx)
         + _grid[y - 0][x + 1].tsd *       dy  * (1. - dx)
         + _grid[y + 1][x + 2].tsd * (1. - dy) *       dx
         + _grid[y - 0][x + 2].tsd *       dy  *       dx;
}

}
