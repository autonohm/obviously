#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdSpacePartition.h"

#include <cstring>
#include <cmath>

namespace obvious
{

static Matrix* _partCoords = NULL;
static Matrix* _cellCoordsHom = NULL;

static int _initializedPartitions = 0;

TsdSpacePartition::TsdSpacePartition(const unsigned int x,
    const unsigned int y,
    const unsigned int z,
    const unsigned int cellsX,
    const unsigned int cellsY,
    const unsigned int cellsZ,
    const double cellSize) : TsdSpaceComponent(true)
{
  _x = x;
  _y = y;
  _z = z;

  _space = NULL;

  _cellSize = cellSize;
  _componentSize = cellSize * (double)cellsX;

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

  _centroid[0] = ((*_edgeCoordsHom)(7, 0)+(*_edgeCoordsHom)(0, 0)) / 2.0;
  _centroid[1] = ((*_edgeCoordsHom)(7, 1)+(*_edgeCoordsHom)(0, 1)) / 2.0;
  _centroid[2] = ((*_edgeCoordsHom)(7, 2)+(*_edgeCoordsHom)(0, 2)) / 2.0;

  double dx = ((*_edgeCoordsHom)(7, 0)-(*_edgeCoordsHom)(0, 0));
  double dy = ((*_edgeCoordsHom)(7, 1)-(*_edgeCoordsHom)(0, 1));
  double dz = ((*_edgeCoordsHom)(7, 2)-(*_edgeCoordsHom)(0, 2));
  _circumradius = sqrt(dx*dx + dy*dy + dz*dz) / 2.0;

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

  _cellCoordsOffset[0] = ((double)_x) * _cellSize;
  _cellCoordsOffset[1] = ((double)_y) * _cellSize;
  _cellCoordsOffset[2] = ((double)_z) * _cellSize;

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

  delete [] _edgeCoordsHom; _edgeCoordsHom = NULL;
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

double& TsdSpacePartition::operator () (unsigned int z, unsigned int y, unsigned int x)
{
  return _space[z][y][x].tsd;
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

double TsdSpacePartition::getInitWeight()
{
  return _initWeight;
}

void TsdSpacePartition::setInitWeight(double weight)
{
  _initWeight = weight;
}

unsigned int TsdSpacePartition::getX()
{
  return _x;
}

unsigned int TsdSpacePartition::getY()
{
  return _y;
}

unsigned int TsdSpacePartition::getZ()
{
  return _z;
}

Matrix* TsdSpacePartition::getCellCoordsHom()
{
  return _cellCoordsHom;
}

void TsdSpacePartition::getCellCoordsOffset(double offset[3])
{
  offset[0] = _cellCoordsOffset[0];
  offset[1] = _cellCoordsOffset[1];
  offset[2] = _cellCoordsOffset[2];
}

Matrix* TsdSpacePartition::getPartitionCoords()
{
  return _partCoords;
}

unsigned int TsdSpacePartition::getWidth()
{
  return _cellsX;
}

unsigned int TsdSpacePartition::getHeight()
{
  return _cellsY;
}

unsigned int TsdSpacePartition::getDepth()
{
  return _cellsZ;
}

unsigned int TsdSpacePartition::getSize()
{
  return _cellsX*_cellsY*_cellsZ;
}

void TsdSpacePartition::addTsd(const unsigned int x, const unsigned int y, const unsigned int z, const double sd, const double maxTruncation)
{
  //if(sd >= -maxTruncation)
  {
    TsdVoxel* voxel = &_space[z][y][x];

    double tsd = sd;
    tsd /= maxTruncation;
    tsd = min(tsd, 1.0);

    /** The following lines were proposed by
     *  E. Bylow, J. Sturm, C. Kerl, F. Kahl, and D. Cremers.
     *  Real-time camera tracking and 3d reconstruction using signed distance functions.
     *  In Robotics: Science and Systems Conference (RSS), June 2013.
     *
     *  SM: Improvements in tracking need to be verified, for the moment this is commented due to runtime improvements
     */
    /*
    double w = 1.0;
    const double eps = -_maxTruncation/4.0;
    if(sd <= eps)
    {
      const double span = -_maxTruncation - eps;
      const double sigma = 3.0/(span*span);
      w = exp(-sigma*(sd-eps)*(sd-eps));
    }
    voxel->weight += w;*/

    voxel->weight += 1.0;

    if(isnan(voxel->tsd))
    {
      voxel->tsd = tsd;
    }
    else
    {
      voxel->weight = min(voxel->weight, TSDSPACEMAXWEIGHT);
      voxel->tsd   = (voxel->tsd * (voxel->weight - 1.0) + tsd) / voxel->weight;
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
            voxel->tsd = 1.0;
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

double TsdSpacePartition::interpolateTrilinear(int x, int y, int z, double dx, double dy, double dz)
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
        double tsd = _space[z][y][x].tsd;
        if(!isnan(tsd))
        {
          *f << z << " " << y << " " << x << " " << tsd << " " << _space[z][y][x].weight << endl;
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
  double weight, tsd;

  *f >> initializedCells;

  for(unsigned int i = 0; i<initializedCells; i++)
  {
    *f >> z >> y >> x >> tsd >> weight;
    TsdVoxel* cell = &_space[z][y][x];
    cell->tsd      = tsd;
    cell->weight   = weight;
  }

  _initializedPartitions++;
}

}
