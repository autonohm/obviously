#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGridPartition.h"

#include <cstring>
#include <cmath>

namespace obvious
{

TsdGridPartition::TsdGridPartition(const unsigned int x, const unsigned int y, const unsigned int cellsX, const unsigned int cellsY, const double cellSize)
{
  _cellCoordsHom = new Matrix(cellsX*cellsY, 3);
  _edgeCoordsHom = new Matrix(4, 3);
  _gridCoords = new Matrix(cellsX*cellsY, 2);

  unsigned int i=0;
  for(unsigned int iy=y; iy<y+cellsY; iy++)
  {
    for(unsigned int ix=x; ix<x+cellsX; ix++, i++)
    {
      (*_cellCoordsHom)(i,0) = ((double)ix + 0.5) * cellSize;
      (*_cellCoordsHom)(i,1) = ((double)iy + 0.5) * cellSize;
      (*_cellCoordsHom)(i,2) = 1.0;
      (*_gridCoords)(i, 0) = ix;
      (*_gridCoords)(i, 1) = iy;
    }
  }

  for(i=0; i<=2; i++)
  {
    (*_edgeCoordsHom)(0, i) = (*_cellCoordsHom)(0, i);
    (*_edgeCoordsHom)(1, i) = (*_cellCoordsHom)(cellsX-1, i);
    (*_edgeCoordsHom)(2, i) = (*_cellCoordsHom)((cellsY-1)*cellsX, i);
    (*_edgeCoordsHom)(3, i) = (*_cellCoordsHom)(cellsY*cellsX-1, i);
  }

  _centroid[0] = ((*_edgeCoordsHom)(0, 0) + (*_edgeCoordsHom)(1, 0) + (*_edgeCoordsHom)(2, 0) + (*_edgeCoordsHom)(3, 0)) / 4.0;
  _centroid[1] = ((*_edgeCoordsHom)(0, 1) + (*_edgeCoordsHom)(1, 1) + (*_edgeCoordsHom)(2, 1) + (*_edgeCoordsHom)(3, 1)) / 4.0;

  _cellsX = cellsX;
  _cellsY = cellsY;
}

TsdGridPartition::~TsdGridPartition()
{
  delete [] _cellCoordsHom;
  delete [] _edgeCoordsHom;
  delete [] _gridCoords;
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

Matrix* TsdGridPartition::getGridCoords()
{
  return _gridCoords;
}

unsigned int TsdGridPartition::getSize()
{
  return _cellsX*_cellsY;
}


}
