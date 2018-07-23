#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdSpacePartition.h"

#include <cstring>
#include <cmath>

namespace obvious
{

Matrix* TsdSpacePartition::_partCoords = NULL;
Matrix* TsdSpacePartition::_cellCoordsHom = NULL;

static int _initializedPartitions = 0;

TsdSpacePartition::TsdSpacePartition(const unsigned int x,
    const unsigned int y,
    const unsigned int z,
    const unsigned int cellsX,
    const unsigned int cellsY,
    const unsigned int cellsZ,
    const obfloat cellSize) : TsdSpaceComponent(true)
{
  _x = x;
  _y = y;
  _z = z;

  _space = NULL;

  _cellSize = cellSize;
  _componentSize = cellSize * (obfloat)cellsX;

  _initWeight = 0.0;

  _edgeCoordsHom = new Matrix(8, 4);
  (*_edgeCoordsHom)(0, 0) = ((double)x) * _cellSize;
  (*_edgeCoordsHom)(0, 1) = ((double)y) * _cellSize;
  (*_edgeCoordsHom)(0, 2) = ((double)z) * _cellSize;
  (*_edgeCoordsHom)(0, 3) = 1.0;

  (*_edgeCoordsHom)(1, 0) = ((double)(x+cellsX)) * _cellSize;
  (*_edgeCoordsHom)(1, 1) = ((double)y) * _cellSize;
  (*_edgeCoordsHom)(1, 2) = ((double)z) * _cellSize;
  (*_edgeCoordsHom)(1, 3) = 1.0;

  (*_edgeCoordsHom)(2, 0) = ((double)x) * _cellSize;
  (*_edgeCoordsHom)(2, 1) = ((double)(y+cellsY)) * _cellSize;
  (*_edgeCoordsHom)(2, 2) = ((double)z) * _cellSize;
  (*_edgeCoordsHom)(2, 3) = 1.0;

  (*_edgeCoordsHom)(3, 0) = ((double)(x+cellsX)) * _cellSize;
  (*_edgeCoordsHom)(3, 1) = ((double)(y+cellsY)) * _cellSize;
  (*_edgeCoordsHom)(3, 2) = ((double)z) * _cellSize;
  (*_edgeCoordsHom)(3, 3) = 1.0;

  (*_edgeCoordsHom)(4, 0) = ((double)x) * _cellSize;
  (*_edgeCoordsHom)(4, 1) = ((double)y) * _cellSize;
  (*_edgeCoordsHom)(4, 2) = ((double)(z+cellsZ)) * _cellSize;
  (*_edgeCoordsHom)(4, 3) = 1.0;

  (*_edgeCoordsHom)(5, 0) = ((double)(x+cellsX)) * _cellSize;
  (*_edgeCoordsHom)(5, 1) = ((double)y) * _cellSize;
  (*_edgeCoordsHom)(5, 2) = ((double)(z+cellsZ)) * _cellSize;
  (*_edgeCoordsHom)(5, 3) = 1.0;

  (*_edgeCoordsHom)(6, 0) = ((double)x) * _cellSize;
  (*_edgeCoordsHom)(6, 1) = ((double)(y+cellsY)) * _cellSize;
  (*_edgeCoordsHom)(6, 2) = ((double)(z+cellsZ)) * _cellSize;
  (*_edgeCoordsHom)(6, 3) = 1.0;

  (*_edgeCoordsHom)(7, 0) = ((double)(x+cellsX)) * _cellSize;
  (*_edgeCoordsHom)(7, 1) = ((double)(y+cellsY)) * _cellSize;
  (*_edgeCoordsHom)(7, 2) = ((double)(z+cellsZ)) * _cellSize;
  (*_edgeCoordsHom)(7, 3) = 1.0;

  _centroid[0] = ((*_edgeCoordsHom)(7, 0)+(*_edgeCoordsHom)(0, 0)) * 0.5;
  _centroid[1] = ((*_edgeCoordsHom)(7, 1)+(*_edgeCoordsHom)(0, 1)) * 0.5;
  _centroid[2] = ((*_edgeCoordsHom)(7, 2)+(*_edgeCoordsHom)(0, 2)) * 0.5;

  obfloat dx = ((*_edgeCoordsHom)(7, 0)-(*_edgeCoordsHom)(0, 0));
  obfloat dy = ((*_edgeCoordsHom)(7, 1)-(*_edgeCoordsHom)(0, 1));
  obfloat dz = ((*_edgeCoordsHom)(7, 2)-(*_edgeCoordsHom)(0, 2));
  _circumradius = sqrt(dx*dx + dy*dy + dz*dz) * 0.5;

  _cellsX = cellsX;
  _cellsY = cellsY;
  _cellsZ = cellsZ;

  if(!_partCoords)
  {
    _partCoords = new Matrix(cellsX*cellsY*cellsZ, 3);
    unsigned int i=0;
    for(unsigned int iz=0; iz<cellsZ; iz++)
    {
      for(unsigned int iy=0; iy<cellsY; iy++)
      {
        for(unsigned int ix=0; ix<cellsX; ix++, i++)
        {
          (*_partCoords)(i, 0) = ix;
          (*_partCoords)(i, 1) = iy;
          (*_partCoords)(i, 2) = iz;
        }
      }
    }
  }

  _cellCoordsOffset[0] = ((obfloat)_x) * _cellSize;
  _cellCoordsOffset[1] = ((obfloat)_y) * _cellSize;
  _cellCoordsOffset[2] = ((obfloat)_z) * _cellSize;

  if(!_cellCoordsHom)
  {
    _cellCoordsHom = new Matrix(_cellsX*_cellsY*_cellsZ, 4);
    unsigned int i=0;
    for(unsigned int iz=0; iz<_cellsZ; iz++)
    {
      for(unsigned int iy=0; iy<_cellsY; iy++)
      {
        for(unsigned int ix=0; ix<_cellsX; ix++, i++)
        {
          (*_cellCoordsHom)(i,0) = ((double)ix + 0.5) * _cellSize;
          (*_cellCoordsHom)(i,1) = ((double)iy + 0.5) * _cellSize;
          (*_cellCoordsHom)(i,2) = ((double)iz + 0.5) * _cellSize;
          (*_cellCoordsHom)(i,3) = 1.0;
        }
      }
    }
  }
}

TsdSpacePartition::~TsdSpacePartition()
{
  reset();

  delete _edgeCoordsHom; _edgeCoordsHom = NULL;
}

int TsdSpacePartition::getInitializedPartitionSize()
{
  return _initializedPartitions;
}

void TsdSpacePartition::reset()
{
  if(_space)
  {
    System<TsdVoxel>::deallocate(_space); _space = NULL;
  }
}

void TsdSpacePartition::getRGB(unsigned int z, unsigned int y, unsigned int x, unsigned char rgb[3])
{
  rgb[0] = _space[z][y][x].rgb[0];
  rgb[1] = _space[z][y][x].rgb[1];
  rgb[2] = _space[z][y][x].rgb[2];
}

void TsdSpacePartition::init()
{
  if(_space) return;

  _initializedPartitions++;

  System<TsdVoxel>::allocate(_cellsZ+1, _cellsY+1, _cellsX+1, _space);
  for (unsigned int iz = 0; iz < _cellsZ+1; iz++)
  {
    for (unsigned int iy = 0; iy < _cellsY+1; iy++)
    {
      for (unsigned int ix = 0; ix < _cellsX+1; ix++)
      {
        _space[iz][iy][ix].tsd    = NAN;
        _space[iz][iy][ix].weight = _initWeight;
        _space[iz][iy][ix].rgb[0] = 255;
        _space[iz][iy][ix].rgb[1] = 255;
        _space[iz][iy][ix].rgb[2] = 255;
      }
    }
  }
}

bool TsdSpacePartition::isInitialized()
{
  return _space!=NULL;
}

bool TsdSpacePartition::isEmpty()
{
  return (_space==NULL && _initWeight > 0.0);
}

void TsdSpacePartition::getCellCoordsOffset(obfloat offset[3])
{
  offset[0] = _cellCoordsOffset[0];
  offset[1] = _cellCoordsOffset[1];
  offset[2] = _cellCoordsOffset[2];
}

void TsdSpacePartition::addTsd(const unsigned int x, const unsigned int y, const unsigned int z, const obfloat sd, const obfloat maxTruncation, const unsigned char rgb[3])
{
  // already checked int TsdSpace
  //if(sd >= -maxTruncation)
  {
    TsdVoxel* voxel = &_space[z][y][x];

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
    voxel->weight += w;*/

    voxel->weight += TSDINC;

    if(isnan(voxel->tsd))
    {
      voxel->tsd = tsd;
      if(rgb)
      {
        voxel->rgb[0] = rgb[0];
        voxel->rgb[1] = rgb[1];
        voxel->rgb[2] = rgb[2];
      }
    }
    else
    {
      voxel->weight = min(voxel->weight, TSDSPACEMAXWEIGHT);
      voxel->tsd   = (voxel->tsd * (voxel->weight - TSDINC) + tsd) / voxel->weight;
      if(rgb)
      {
        voxel->rgb[0] = (voxel->rgb[0] * (voxel->weight - TSDINC) + rgb[0]) / voxel->weight;
        voxel->rgb[1] = (voxel->rgb[1] * (voxel->weight - TSDINC) + rgb[1]) / voxel->weight;
        voxel->rgb[2] = (voxel->rgb[2] * (voxel->weight - TSDINC) + rgb[2]) / voxel->weight;
      }
    }
  }
}

void TsdSpacePartition::increaseEmptiness()
{
  if(_space)
  {
    for(unsigned int z=1; z<=_cellsZ; z++)
    {
      for(unsigned int y=1; y<=_cellsY; y++)
      {
        for(unsigned int x=1; x<=_cellsX; x++)
        {
          TsdVoxel* voxel = &_space[z][y][x];
          voxel->weight += 1.0;

          if(isnan(voxel->tsd))
          {
            //voxel->tsd = 1.0;
          }
          else
          {
            voxel->weight = min(voxel->weight, TSDSPACEMAXWEIGHT);
            voxel->tsd    = (voxel->tsd * (voxel->weight - 1.0) + 1.0) / voxel->weight;
          }
        }
      }
    }
  }
  else
  {
    _initWeight += 1.0;
    _initWeight = min(_initWeight, TSDSPACEMAXWEIGHT);
  }
}

obfloat TsdSpacePartition::interpolateTrilinear(int x, int y, int z, obfloat dx, obfloat dy, obfloat dz) const
{
  // Interpolate
  return _space[z][y][x].tsd * (1. - dx) * (1. - dy) * (1. - dz)
      +  _space[z + 1][y + 0][x + 0].tsd * (1. - dx) * (1. - dy) * dz
      +  _space[z + 0][y + 1][x + 0].tsd * (1. - dx) * dy * (1. - dz)
      +  _space[z + 1][y + 1][x + 0].tsd * (1. - dx) * dy * dz
      +  _space[z + 0][y + 0][x + 1].tsd * dx * (1. - dy) * (1. - dz)
      +  _space[z + 1][y + 0][x + 1].tsd * dx * (1. - dy) * dz
      +  _space[z + 0][y + 1][x + 1].tsd * dx * dy * (1. - dz)
      +  _space[z + 1][y + 1][x + 1].tsd * dx * dy * dz;
}

void TsdSpacePartition::serialize(ofstream* f)
{
  unsigned int initializedCells = 0;

  for(unsigned int z=0 ; z<_cellsZ+1; z++)
  {
    for(unsigned int y=0; y<_cellsY+1; y++)
    {
      for(unsigned int x=0; x<_cellsX+1; x++)
      {
        if(!isnan(_space[z][y][x].tsd))
          initializedCells++;
      }
    }
  }

  *f << initializedCells << endl;

  for(unsigned int z=0 ; z<_cellsZ+1; z++)
  {
    for(unsigned int y=0; y<_cellsY+1; y++)
    {
      for(unsigned int x=0; x<_cellsX+1; x++)
      {
        obfloat tsd = _space[z][y][x].tsd;
        if(!isnan(tsd))
        {
          *f << z << " " << y << " " << x << " " << tsd << " " << _space[z][y][x].weight << " " << (int)_space[z][y][x].rgb[0] << " " << (int)_space[z][y][x].rgb[1] << " " << (int)_space[z][y][x].rgb[2] << endl;
        }
      }
    }
  }
}

void TsdSpacePartition::load(ifstream* f)
{
  init();

  unsigned int initializedCells;
  unsigned int x, y, z;
  obfloat weight, tsd;
  unsigned int rgb0;
  unsigned int rgb1;
  unsigned int rgb2;

  *f >> initializedCells;

  for(unsigned int i = 0; i<initializedCells; i++)
  {
    *f >> z >> y >> x >> tsd >> weight >> rgb0 >> rgb1 >> rgb2;
    TsdVoxel* cell = &_space[z][y][x];
    cell->tsd      = tsd;
    cell->weight   = weight;
    cell->rgb[0]   = (unsigned char)rgb0;
    cell->rgb[1]   = (unsigned char)rgb1;
    cell->rgb[2]   = (unsigned char)rgb2;
  }

  _initializedPartitions++;
}

}
