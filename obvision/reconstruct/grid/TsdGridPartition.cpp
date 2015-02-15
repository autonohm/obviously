#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGridPartition.h"

#include <cstring>
#include <cmath>

namespace obvious
{

static Matrix* _partCoords;

TsdGridPartition::TsdGridPartition(const unsigned int x,
    const unsigned int y,
    const unsigned int cellsX,
    const unsigned int cellsY,
    const obfloat cellSize) : TsdGridComponent(true)
{
  _initialized = false;

  _x = x;
  _y = y;

  _grid = NULL;
  _cellCoordsHom = NULL;

  _cellSize = cellSize;
  _componentSize = cellSize * (obfloat)cellsX;

  _initWeight = 0.0;

  if(!_partCoords)
  {
    _partCoords = new Matrix(cellsX*cellsY, 2);
    unsigned int i=0;
    for(unsigned int iy=0; iy<cellsY; iy++)
    {
      for(unsigned int ix=0; ix<cellsX; ix++, i++)
      {
        (*_partCoords)(i, 0) = ix;
        (*_partCoords)(i, 1) = iy;
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

  obfloat dx = ((*_edgeCoordsHom)(3, 0)-(*_edgeCoordsHom)(0, 0));
  obfloat dy = ((*_edgeCoordsHom)(3, 1)-(*_edgeCoordsHom)(0, 1));
  _circumradius = sqrt(dx*dx + dy*dy) / 2.0;

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

obfloat& TsdGridPartition::operator () (unsigned int y, unsigned int x)
{
  return _grid[y][x].tsd;
}

void TsdGridPartition::init()
{
  if(_initialized) return;

  System<TsdCell>::allocate(_cellsY+1, _cellsX+1, _grid);
  if(_initWeight>0.0)
  {
    for (unsigned int iy = 0; iy <= _cellsY; iy++)
    {
      for (unsigned int ix = 0; ix <= _cellsX; ix++)
      {
        _grid[iy][ix].tsd    = 1.0;
        _grid[iy][ix].weight = _initWeight;
      }
    }
  }
  else
  {
    for (unsigned int iy = 0; iy <= _cellsY; iy++)
    {
      for (unsigned int ix = 0; ix <= _cellsX; ix++)
      {
        _grid[iy][ix].tsd    = NAN;
        _grid[iy][ix].weight = _initWeight;
      }
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

  _initialized = true;
}

bool TsdGridPartition::isInitialized()
{
  return _initialized;
}

bool TsdGridPartition::isEmpty()
{
  return (!_initialized && _initWeight > 0.0);
}

unsigned int TsdGridPartition::getX()
{
  return _x;
}

unsigned int TsdGridPartition::getY()
{
  return _y;
}

Matrix* TsdGridPartition::getCellCoordsHom()
{
  return _cellCoordsHom;
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

void TsdGridPartition::addTsd(const unsigned int x, const unsigned int y, const obfloat sd, const obfloat maxTruncation, const obfloat weight)
{
  // Factor avoids thin objects to be removed when seen from two sides
  if(sd >= -2.0*maxTruncation)
  {
    TsdCell* cell = &_grid[y][x];

    obfloat tsd = min(sd / maxTruncation, TSDINC);

    /** The following lines were proposed by
     *  E. Bylow, J. Sturm, C. Kerl, F. Kahl, and D. Cremers.
     *  Real-time camera tracking and 3d reconstruction using signed distance functions.
     *  In Robotics: Science and Systems Conference (RSS), June 2013.
     *
     *  SM: Improvements in tracking need to be verified, for the moment this is commented due to runtime improvements
     */
    /*
    obfloat w = 1.0;
    const obfloat eps = -_maxTruncation/4.0;
    if(sd <= eps)
    {
      const obfloat span = -_maxTruncation - eps;
      const obfloat sigma = 3.0/(span*span);
      w = exp(-sigma*(sd-eps)*(sd-eps));
    }
    weight *= w;*/

    if(isnan(cell->tsd))
    {
      cell->tsd = tsd;
      //cell->weight += TSDINC;
      cell->weight += weight;
    }
    else
    {
      //cell->weight = min(cell->weight+TSDINC, TSDGRIDMAXWEIGHT);
      //cell->tsd   = (cell->tsd * (cell->weight - TSDINC) + tsdf) / cell->weight;

      cell->tsd   = (cell->tsd * cell->weight + tsd * weight) / (cell->weight + weight);
      cell->weight = min(cell->weight+weight, TSDGRIDMAXWEIGHT);
    }
  }
}

void TsdGridPartition::increaseEmptiness()
{
  if(_initialized)
  {
    for(unsigned int y=0; y<=_cellsY; y++)
    {
      for(unsigned int x=0; x<=_cellsX; x++)
      {
        TsdCell* cell = &_grid[y][x];

        if(isnan(cell->tsd))
        {
          cell->weight += 1.0;
          cell->tsd = 1.0;
        }
        else
        {
          cell->weight = min(cell->weight+1, TSDGRIDMAXWEIGHT);
          cell->tsd    = (cell->tsd * (cell->weight - 1.0) + 1.0) / cell->weight;
        }
      }
    }
  }
  else
  {
    _initWeight += 1.0;
    _initWeight = min(_initWeight, TSDGRIDMAXWEIGHT);
  }
}

obfloat TsdGridPartition::interpolateBilinear(int x, int y, obfloat dx, obfloat dy)
{
  // Interpolate
  return _grid[y][x].tsd         * (1. - dy) * (1. - dx)
       + _grid[y + 1][x + 0].tsd *       dy  * (1. - dx)
       + _grid[y + 0][x + 1].tsd * (1. - dy) *       dx
       + _grid[y + 1][x + 1].tsd *       dy  *       dx;
}

}
