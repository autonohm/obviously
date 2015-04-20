#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdSpace.h"
#include "TsdSpaceBranch.h"
#include "SensorProjective3D.h"

#include <cstring>
#include <cmath>
#include <omp.h>

namespace obvious
{

// Beware: this will slow down the application
#define PRINTSTATISTICS 0

#if PRINTSTATISTICS
static int _distancesPushed = 0;
#endif

#if _OBVIOUS_DOUBLE_PRECISION_
#define MAXWEIGHT 32.0
#else
#define MAXWEIGHT 32.f
#endif
#define RGB_MAX 255

TsdSpace::TsdSpace(const double voxelSize, const EnumTsdSpaceLayout layoutPartition, const EnumTsdSpaceLayout layoutSpace)
{
  _voxelSize = voxelSize;
  _invVoxelSize = 1.0 / _voxelSize;

  _layoutPartition = layoutPartition;
  _layoutSpace = layoutSpace;

  // determine number of voxels in each dimension
  _cellsX = 1u << layoutSpace;
  _cellsY = _cellsX;
  _cellsZ = _cellsX;

  unsigned int dimPartition = 1u << layoutPartition;

  if(dimPartition > _cellsX)
  {
    LOGMSG(DBG_ERROR, "Insufficient partition size : " << dimPartition << "x" << dimPartition << "x" << dimPartition << " in "
        << _cellsX << "x" << _cellsY << "x" << _cellsZ << " space");
    return;
  }

  _partitionsInX = _cellsX/dimPartition;
  _partitionsInY = _cellsY/dimPartition;
  _partitionsInZ = _cellsZ/dimPartition;

  _lutIndex2Partition = new int[_cellsX];
  _lutIndex2Cell = new int[_cellsX];
  for(unsigned int i=0; i<_cellsX; i++)
  {
    _lutIndex2Partition[i] = i / dimPartition;
    _lutIndex2Cell[i] = i % dimPartition;
  }

  _maxTruncation = 2.0*voxelSize;

  LOGMSG(DBG_DEBUG, "Dimensions are (x/y/z) (" << _cellsX << "/" << _cellsY << "/" << _cellsZ << ")");
  LOGMSG(DBG_DEBUG, "Creating TsdVoxel Space...");

  _minX = 0.0;
  _maxX = ((obfloat)_cellsX + 0.5) * _voxelSize;
  _minY = 0.0;
  _maxY = ((obfloat)_cellsY + 0.5) * _voxelSize;
  _minZ = 0.0;
  _maxZ = ((obfloat)_cellsZ + 0.5) * _voxelSize;

  LOGMSG(DBG_DEBUG, "Allocating " << _partitionsInX << "x" << _partitionsInY << "x" << _partitionsInZ << " partitions");
  LOGMSG(DBG_DEBUG, "Spanning area: " << _maxX << " " << _maxY << " " << _maxZ << endl;)
  System<TsdSpacePartition*>::allocate(_partitionsInZ, _partitionsInY, _partitionsInX, _partitions);

  for(int pz=0; pz<_partitionsInZ; pz++)
  {
    for(int py=0; py<_partitionsInY; py++)
    {
      for(int px=0; px<_partitionsInX; px++)
      {
        _partitions[pz][py][px] = new TsdSpacePartition(px*dimPartition, py*dimPartition, pz*dimPartition, dimPartition, dimPartition, dimPartition, voxelSize);
      }
    }
  }

  int depthTree = layoutSpace-layoutPartition;
  if(depthTree == 0)
  {
    _tree = _partitions[0][0][0];
  }
  else
  {
    TsdSpaceBranch* tree = new TsdSpaceBranch((TsdSpaceComponent****)_partitions, 0, 0, 0, depthTree);
    _tree = tree;
  }
}

TsdSpace::~TsdSpace(void)
{
  delete _tree;
  System<TsdSpacePartition*>::deallocate(_partitions);
  delete [] _lutIndex2Partition;
  delete [] _lutIndex2Cell;
}

void TsdSpace::reset()
{
  for(int pz=0; pz<_partitionsInZ; pz++)
  {
    for(int py=0; py<_partitionsInY; py++)
    {
      for(int px=0; px<_partitionsInX; px++)
      {
        _partitions[pz][py][px]->reset();
      }
    }
  }
}

unsigned int TsdSpace::getXDimension()
{
  return _cellsX;
}

unsigned int TsdSpace::getYDimension()
{
  return _cellsY;
}

unsigned int TsdSpace::getZDimension()
{
  return _cellsZ;
}

int TsdSpace::getPartitionsInX()
{
  return _partitionsInX;
}

int TsdSpace::getPartitionsInY()
{
  return _partitionsInX;
}

int TsdSpace::getPartitionsInZ()
{
  return _partitionsInX;
}

obfloat TsdSpace::getVoxelSize()
{
  return _voxelSize;
}

unsigned int TsdSpace::getPartitionSize()
{
  return _partitions[0][0][0]->getWidth();
}

obfloat TsdSpace::getMinX()
{
  return _minX;
}

obfloat TsdSpace::getMaxX()
{
  return _maxX;
}

obfloat TsdSpace::getMinY()
{
  return _minY;
}

obfloat TsdSpace::getMaxY()
{
  return _maxY;
}

obfloat TsdSpace::getMinZ()
{
  return _minZ;
}

obfloat TsdSpace::getMaxZ()
{
  return _maxZ;
}

void TsdSpace::getCentroid(obfloat centroid[3])
{
  centroid[0] = (_minX + _maxX) / 2.0;
  centroid[1] = (_minY + _maxY) / 2.0;
  centroid[2] = (_minZ + _maxZ) / 2.0;
}

void TsdSpace::setMaxTruncation(obfloat val)
{
  if(val < 2.0 * _voxelSize)
  {
    LOGMSG(DBG_WARN, "Truncation radius must be at 2 x voxel dimension. Setting minimum size.");
    val = 2.0 * _voxelSize;
  }

  _maxTruncation = val;
}

double TsdSpace::getMaxTruncation()
{
  return _maxTruncation;
}

TsdSpacePartition**** TsdSpace::getPartitions()
{
  return _partitions;
}

bool TsdSpace::isPartitionInitialized(obfloat coord[3])
{
  /*int x = (int)(coord[0] * _invVoxelSize);
  int y = (int)(coord[1] * _invVoxelSize);
  int z = (int)(coord[2] * _invVoxelSize);

  int px = _lutIndex2Partition[x];
  int py = _lutIndex2Partition[y];
  int pz = _lutIndex2Partition[z];

  return _partitions[pz][py][px]->isInitialized();*/

  // Get cell indices
  obfloat dxIdx = floor(coord[0] * _invVoxelSize);
  obfloat dyIdx = floor(coord[1] * _invVoxelSize);
  obfloat dzIdx = floor(coord[2] * _invVoxelSize);

  // Get center point of current cell
  obfloat dx = (dxIdx + 0.5) * _voxelSize;
  obfloat dy = (dyIdx + 0.5) * _voxelSize;
  obfloat dz = (dzIdx + 0.5) * _voxelSize;

  int x = (int)dxIdx;
  int y = (int)dyIdx;
  int z = (int)dzIdx;

  // Ensure that query point has 8 neighbors for trilinear interpolation
  if (coord[0] < dx)
    x--;

  if (coord[1] < dy)
    y--;

  if (coord[2] < dz)
    z--;

  int px = _lutIndex2Partition[x];
  int py = _lutIndex2Partition[y];
  int pz = _lutIndex2Partition[z];

  return _partitions[pz][py][px]->isInitialized();
}

bool TsdSpace::isInsideSpace(Sensor* sensor)
{
  obfloat coord[3];
  sensor->getPosition(coord);
  return (coord[0]>_minX && coord[0]<_maxX && coord[1]>_minY && coord[1]<_maxY && coord[2]>_minZ && coord[2]<_maxZ);
}

void TsdSpace::push(Sensor* sensor)
{
  Timer timer;
  timer.start();

  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();
  unsigned char* rgb = sensor->getRealMeasurementRGB();

  obfloat tr[3];
  sensor->getPosition(tr);

  Matrix* partCoords = TsdSpacePartition::getPartitionCoords();
  Matrix* cellCoordsHom = TsdSpacePartition::getCellCoordsHom();

#pragma omp parallel
  {
    unsigned int partSize = (_partitions[0][0][0])->getSize();
    int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
    for(int pz=0; pz<_partitionsInZ; pz++)
    {
      for(int py=0; py<_partitionsInY; py++)
      {
        for(int px=0; px<_partitionsInX; px++)
        {
          TsdSpacePartition* part = _partitions[pz][py][px];
          if(!part->isInRange(tr, sensor, _maxTruncation)) continue;

          obfloat t[3];
          part->getCellCoordsOffset(t);
          Matrix T = MatrixFactory::TranslationMatrix44(t[0], t[1], t[2]);
          sensor->backProject(cellCoordsHom, idx, &T);

          for(unsigned int c=0; c<partSize; c++)
          {
            // Measurement index
            int index = idx[c];

            if(index>=0)
            {
              if(mask[index])
              {
                // calculate distance of current cell to sensor
                obfloat crd[3];
                crd[0] = (*cellCoordsHom)(c,0) + t[0];
                crd[1] = (*cellCoordsHom)(c,1) + t[1];
                crd[2] = (*cellCoordsHom)(c,2) + t[2];
                obfloat distance = euklideanDistance<obfloat>(tr, crd, 3);
                obfloat sd = data[index] - distance;

                // Test with distance-related weighting
                /*double weight = 1.0 - (10.0 - distance);
              weight = max(weight, 0.1);*/

                unsigned char* color = NULL;
                if(rgb) color = &(rgb[3*index]);
                if(sd >= -_maxTruncation)
                {
                  part->init();
                  part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2), sd, _maxTruncation, color);

#if PRINTSTATISTICS
#pragma omp critical
                  {
                    _distancesPushed++;
                  }
#endif
                }
              }
            }
          }
        }
      }
    }
    delete [] idx;
  }

  propagateBorders();

#if PRINTSTATISTICS
  LOGMSG(DBG_DEBUG, "Distances pushed: " << _distancesPushed);
#endif

  LOGMSG(DBG_DEBUG, "Elapsed push: " << timer.elapsed() << "s, Initialized partitions: " << TsdSpacePartition::getInitializedPartitionSize());
}

void TsdSpace::pushTree(Sensor* sensor)
{
  Timer timer;
  timer.start();

  double* data = sensor->getRealMeasurementData();
  bool* mask = sensor->getRealMeasurementMask();
  unsigned char* rgb = sensor->getRealMeasurementRGB();

  obfloat tr[3];
  sensor->getPosition(tr);

  TsdSpaceComponent* comp = _tree;
  vector<TsdSpacePartition*> partitionsToCheck;
  pushRecursion(sensor, tr, comp, partitionsToCheck);

  LOGMSG(DBG_DEBUG, "Partitions to check: " << partitionsToCheck.size());

  Matrix* partCoords = TsdSpacePartition::getPartitionCoords();
  Matrix* cellCoordsHom = TsdSpacePartition::getCellCoordsHom();

#pragma omp parallel
  {
    unsigned int partSize = (_partitions[0][0][0])->getSize();
    int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
    for(unsigned int i=0; i<partitionsToCheck.size(); i++)
    {
      TsdSpacePartition* part = partitionsToCheck[i];

      obfloat t[3];
      part->getCellCoordsOffset(t);
      Matrix T = MatrixFactory::TranslationMatrix44(t[0], t[1], t[2]);
      sensor->backProject(cellCoordsHom, idx, &T);

      for(unsigned int c=0; c<partSize; c++)
      {
        // Measurement index
        int index = idx[c];

        if(index>=0)
        {
          if(mask[index])
          {
            // calculate distance of current cell to sensor
            obfloat crd[3];
            crd[0] = (*cellCoordsHom)(c,0) + t[0];
            crd[1] = (*cellCoordsHom)(c,1) + t[1];
            crd[2] = (*cellCoordsHom)(c,2) + t[2];
            obfloat distance = euklideanDistance<obfloat>(tr, crd, 3);
            obfloat sd = data[index] - distance;

            // Test with distance-related weighting
            /*double weight = 1.0 - (10.0 - distance);
          weight = max(weight, 0.1);*/

            unsigned char* color = NULL;
            if(rgb) color = &(rgb[3*index]);
            if(sd >= -_maxTruncation)
            {
              part->init();
              part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2), sd, _maxTruncation, color);

#if PRINTSTATISTICS
#pragma omp critical
              {
                _distancesPushed++;
              }
#endif

            }
          }
        }
      }
    }
    delete [] idx;
  }

  propagateBorders();

#if PRINTSTATISTICS
  LOGMSG(DBG_DEBUG, "Distances pushed: " << _distancesPushed);
#endif

  LOGMSG(DBG_DEBUG, "Elapsed push: " << timer.elapsed() << "s, Initialized partitions: " << TsdSpacePartition::getInitializedPartitionSize());
}

void TsdSpace::pushRecursion(Sensor* sensor, obfloat pos[3], TsdSpaceComponent* comp, vector<TsdSpacePartition*> &partitionsToCheck)
{
  if(comp->isInRange(pos, sensor, _maxTruncation))
  {
    if(comp->isLeaf())
      partitionsToCheck.push_back((TsdSpacePartition*)comp);
    else
    {
      vector<TsdSpaceComponent*> children = ((TsdSpaceBranch*)comp)->getChildren();
      for(unsigned int i=0; i<children.size(); i++)
        pushRecursion(sensor, pos, children[i], partitionsToCheck);
    }
  }
}

void TsdSpace::propagateBorders()
{
  unsigned int width  = _partitions[0][0][0]->getWidth();
  unsigned int height = _partitions[0][0][0]->getHeight();
  unsigned int depth = _partitions[0][0][0]->getDepth();

  // Copy valid tsd values of neighbors to borders of each partition.
  for(int pz=0; pz<_partitionsInZ; pz++)
  {
    for(int py=0; py<_partitionsInY; py++)
    {
      for(int px=0; px<_partitionsInX; px++)
      {
        TsdSpacePartition* partCur       = _partitions[pz][py][px];

        if(!partCur->isInitialized()) continue;

        if(px<_partitionsInX-1)
        {
          TsdSpacePartition* partRight      = _partitions[pz][py][px+1];
          if(partRight->isInitialized())
          {
            for(unsigned int d=0; d<depth; d++)
            {
              for(unsigned int h=0; h<height; h++)
              {
                partCur->_space[d][h][width].tsd = partRight->_space[d][h][0].tsd;
                partCur->_space[d][h][width].weight = partRight->_space[d][h][0].weight;
                partCur->_space[d][h][width].rgb[0] = partRight->_space[d][h][0].rgb[0];
                partCur->_space[d][h][width].rgb[1] = partRight->_space[d][h][0].rgb[1];
                partCur->_space[d][h][width].rgb[2] = partRight->_space[d][h][0].rgb[2];
              }
            }
          }
        }

        if(py<_partitionsInY-1)
        {
          TsdSpacePartition* partUp      = _partitions[pz][py+1][px];
          if(partUp->isInitialized())
          {
            for(unsigned int d=0; d<depth; d++)
            {
              for(unsigned int w=0; w<width; w++)
              {
                partCur->_space[d][height][w].tsd = partUp->_space[d][0][w].tsd;
                partCur->_space[d][height][w].weight = partUp->_space[d][0][w].weight;
                partCur->_space[d][height][w].rgb[0] = partUp->_space[d][0][w].rgb[0];
                partCur->_space[d][height][w].rgb[1] = partUp->_space[d][0][w].rgb[1];
                partCur->_space[d][height][w].rgb[2] = partUp->_space[d][0][w].rgb[2];
              }
            }
          }
        }

        if(pz<_partitionsInZ-1)
        {
          TsdSpacePartition* partBack      = _partitions[pz+1][py][px];
          if(partBack->isInitialized())
          {
            for(unsigned int h=0; h<height; h++)
            {
              for(unsigned int w=0; w<width; w++)
              {
                partCur->_space[depth][h][w].tsd = partBack->_space[0][h][w].tsd;
                partCur->_space[depth][h][w].weight = partBack->_space[0][h][w].weight;
                partCur->_space[depth][h][w].rgb[0] = partBack->_space[0][h][w].rgb[0];
                partCur->_space[depth][h][w].rgb[1] = partBack->_space[0][h][w].rgb[1];
                partCur->_space[depth][h][w].rgb[2] = partBack->_space[0][h][w].rgb[2];
              }
            }
          }
        }

        if(px<_partitionsInX-1 && pz<_partitionsInZ-1)
        {
          TsdSpacePartition* partRightBack      = _partitions[pz+1][py][px+1];
          if(partRightBack->isInitialized())
          {
            for(unsigned int h=0; h<height; h++)
            {
              partCur->_space[depth][h][width].tsd = partRightBack->_space[0][h][0].tsd;
              partCur->_space[depth][h][width].weight = partRightBack->_space[0][h][0].weight;
              partCur->_space[depth][h][width].rgb[0] = partRightBack->_space[0][h][0].rgb[0];
              partCur->_space[depth][h][width].rgb[1] = partRightBack->_space[0][h][0].rgb[1];
              partCur->_space[depth][h][width].rgb[2] = partRightBack->_space[0][h][0].rgb[2];
            }
          }
        }

        if(px<_partitionsInX-1 && py<_partitionsInY-1)
        {
          TsdSpacePartition* partRightUp      = _partitions[pz][py+1][px+1];
          if(partRightUp->isInitialized())
          {
            for(unsigned int d=0; d<depth; d++)
            {
              partCur->_space[d][height][width].tsd = partRightUp->_space[d][0][0].tsd;
              partCur->_space[d][height][width].weight = partRightUp->_space[d][0][0].weight;
              partCur->_space[d][height][width].rgb[0] = partRightUp->_space[d][0][0].rgb[0];
              partCur->_space[d][height][width].rgb[1] = partRightUp->_space[d][0][0].rgb[1];
              partCur->_space[d][height][width].rgb[2] = partRightUp->_space[d][0][0].rgb[2];
            }
          }
        }

        if(py<_partitionsInY-1 && pz<_partitionsInZ-1)
        {
          TsdSpacePartition* partBackUp      = _partitions[pz+1][py+1][px];
          if(partBackUp->isInitialized())
          {
            for(unsigned int w=0; w<width; w++)
            {
              partCur->_space[depth][height][w].tsd = partBackUp->_space[0][0][w].tsd;
              partCur->_space[depth][height][w].weight = partBackUp->_space[0][0][w].weight;
              partCur->_space[depth][height][w].rgb[0] = partBackUp->_space[0][0][w].rgb[0];
              partCur->_space[depth][height][w].rgb[1] = partBackUp->_space[0][0][w].rgb[1];
              partCur->_space[depth][height][w].rgb[2] = partBackUp->_space[0][0][w].rgb[2];
            }
          }
        }

        if(px<_partitionsInX-1 && py<_partitionsInY-1 && pz<_partitionsInZ-1 )
        {
          TsdSpacePartition* partBackRightUp      = _partitions[pz+1][py+1][px+1];
          if(partBackRightUp->isInitialized())
          {
            partCur->_space[depth][height][width].tsd = partBackRightUp->_space[0][0][0].tsd;
            partCur->_space[depth][height][width].weight = partBackRightUp->_space[0][0][0].weight;
            partCur->_space[depth][height][width].rgb[0] = partBackRightUp->_space[0][0][0].rgb[0];
            partCur->_space[depth][height][width].rgb[1] = partBackRightUp->_space[0][0][0].rgb[1];
            partCur->_space[depth][height][width].rgb[2] = partBackRightUp->_space[0][0][0].rgb[2];
          }
        }
      }
    }
  }
}

bool TsdSpace::interpolateNormal(const obfloat* coord, obfloat* normal)
{
  obfloat neighbor[3];
  obfloat depthVarInc = 0;
  obfloat depthVarDec = 0;

  //interpolate around Voxel in x+1 direction
  neighbor[0] = coord[0] + _voxelSize;
  neighbor[1] = coord[1];
  neighbor[2] = coord[2];
  if(interpolateTrilinear(neighbor, &depthVarInc)!=INTERPOLATE_SUCCESS)
    return false;

  //interpolate around Voxel in x-1 direction
  neighbor[0] = coord[0] - _voxelSize;
  //neighbor[1] = coord[1];
  //neighbor[2] = coord[2];
  if(interpolateTrilinear(neighbor, &depthVarDec)!=INTERPOLATE_SUCCESS)
    return false;

  //x-coordinate of normal vector
  normal[0] = depthVarInc - depthVarDec;

  //interpolate around Voxel in y+1 direction
  neighbor[0] = coord[0];
  neighbor[1] = coord[1] + _voxelSize;
  //neighbor[2] = coord[2];
  if(interpolateTrilinear(neighbor, &depthVarInc)!=INTERPOLATE_SUCCESS)
    return false;

  //interpolate around Voxel in y-1 direction
  //neighbor[0] = coord[0];
  neighbor[1] = coord[1] - _voxelSize;
  //neighbor[2] = coord[2];
  if(interpolateTrilinear(neighbor, &depthVarDec)!=INTERPOLATE_SUCCESS)
    return false;

  //y-coordinate of normal vector
  normal[1] = depthVarInc - depthVarDec;

  //interpolate around Voxel in z+1 direction
  //neighbor[0] = coord[0];
  neighbor[1] = coord[1];
  neighbor[2] = coord[2] + _voxelSize;
  if(interpolateTrilinear(neighbor, &depthVarInc)!=INTERPOLATE_SUCCESS)
    return false;

  //interpolate around Voxel in z-1 direction
  //neighbor[0] = coord[0];
  //neighbor[1] = coord[1];
  neighbor[2] = coord[2] - _voxelSize;
  if(interpolateTrilinear(neighbor, &depthVarDec)!=INTERPOLATE_SUCCESS)
    return false;

  //z-coordinate of normal vector
  normal[2] = depthVarInc - depthVarDec;

  norm3<obfloat>(normal);

  return true;
}

bool TsdSpace::coord2Index(obfloat coord[3], int* x, int* y, int* z, obfloat* dx, obfloat* dy, obfloat* dz)
{
  // Get cell indices
  obfloat dxIdx = floor(coord[0] * _invVoxelSize);
  obfloat dyIdx = floor(coord[1] * _invVoxelSize);
  obfloat dzIdx = floor(coord[2] * _invVoxelSize);

  // Get center point of current cell
  *dx = (dxIdx + 0.5) * _voxelSize;
  *dy = (dyIdx + 0.5) * _voxelSize;
  *dz = (dzIdx + 0.5) * _voxelSize;

  *x = (int)dxIdx;
  *y = (int)dyIdx;
  *z = (int)dzIdx;

  // Ensure that query point has 8 neighbors for trilinear interpolation
  if (coord[0] < *dx)
  {
    (*x)--;
    (*dx) -= _voxelSize;
  }
  if (coord[1] < *dy)
  {
    (*y)--;
    (*dy) -= _voxelSize;
  }
  if (coord[2] < *dz)
  {
    (*z)--;
    (*dz) -= _voxelSize;
  }

  // Check boundaries -> should never happen
  /*if ((*x >= (int)_cellsX) || (*x < 0) || (*y >= (int)_cellsY) || (*y < 0) || (*z >= (int)_cellsZ) || *z < 0)
  {
#pragma omp critical
{
    cout << "coord not in space: " << coord[0] << " " << coord[1] << " " << coord[2] << " assigned: " << *x << " " << *y << " " << *z << endl;
    //abort();
}
    return false;
  }*/

  return true;
}

EnumTsdSpaceInterpolate TsdSpace::interpolateTrilinear(obfloat coord[3], obfloat* tsd)
{
  obfloat dx;
  obfloat dy;
  obfloat dz;

  int xIdx;
  int yIdx;
  int zIdx;
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz)) return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2Partition[xIdx];
  int py = _lutIndex2Partition[yIdx];
  int pz = _lutIndex2Partition[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized()) return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2Cell[xIdx];
  int y = _lutIndex2Cell[yIdx];
  int z = _lutIndex2Cell[zIdx];

  obfloat wx = fabs((coord[0] - dx) * _invVoxelSize);
  obfloat wy = fabs((coord[1] - dy) * _invVoxelSize);
  obfloat wz = fabs((coord[2] - dz) * _invVoxelSize);

  *tsd = part->interpolateTrilinear(x, y, z, wx, wy, wz);

  if(isnan(*tsd)) return INTERPOLATE_ISNAN;

  return INTERPOLATE_SUCCESS;
}

EnumTsdSpaceInterpolate TsdSpace::getTsd(obfloat coord[3], obfloat* tsd)
{
  obfloat dx;
  obfloat dy;
  obfloat dz;

  int xIdx;
  int yIdx;
  int zIdx;
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz)) return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2Partition[xIdx];
  int py = _lutIndex2Partition[yIdx];
  int pz = _lutIndex2Partition[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized()) return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2Cell[xIdx];
  int y = _lutIndex2Cell[yIdx];
  int z = _lutIndex2Cell[zIdx];

  *tsd = (*part)(z, y, x);

  if(isnan(*tsd)) return INTERPOLATE_ISNAN;

  return INTERPOLATE_SUCCESS;
}

EnumTsdSpaceInterpolate TsdSpace::interpolateTrilinearRGB(obfloat coord[3], unsigned char rgb[3])
{
  obfloat dx;
  obfloat dy;
  obfloat dz;

  int xIdx;
  int yIdx;
  int zIdx;
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz)) return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2Partition[xIdx];
  int py = _lutIndex2Partition[yIdx];
  int pz = _lutIndex2Partition[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized()) return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2Cell[xIdx];
  int y = _lutIndex2Cell[yIdx];
  int z = _lutIndex2Cell[zIdx];

  double wx = fabs((coord[0] - dx) * _invVoxelSize);
  double wy = fabs((coord[1] - dy) * _invVoxelSize);
  double wz = fabs((coord[2] - dz) * _invVoxelSize);

  unsigned char pRGB[8][3];

  part->getRGB(z+0, y+0, x+0, pRGB[0]);
  part->getRGB(z+1, y+0, x+0, pRGB[1]);
  part->getRGB(z+0, y+1, x+0, pRGB[2]);
  part->getRGB(z+1, y+1, x+0, pRGB[3]);
  part->getRGB(z+0, y+0, x+1, pRGB[4]);
  part->getRGB(z+1, y+0, x+1, pRGB[5]);
  part->getRGB(z+0, y+1, x+1, pRGB[6]);
  part->getRGB(z+1, y+1, x+1, pRGB[7]);

  double pw[8];
  pw[0] = (1. - wx) * (1. - wy) * (1. - wz);
  pw[1] = (1. - wx) * (1. - wy) * wz;
  pw[2] = (1. - wx) * wy * (1. - wz);
  pw[3] = (1. - wx) * wy * wz;
  pw[4] = wx * (1. - wy) * (1. - wz);
  pw[5] = wx * (1. - wy) * wz;
  pw[6] = wx * wy * (1. - wz);
  pw[7] = wx * wy * wz;

  memset(rgb,0,3);
  for(unsigned int i=0; i<8; i++)
  {
    rgb[0] += pRGB[i][0] * pw[i];
    rgb[1] += pRGB[i][1] * pw[i];
    rgb[2] += pRGB[i][2] * pw[i];
  }

  return INTERPOLATE_SUCCESS;
}

/*bool TsdSpace::buildSliceImage(const unsigned int depthIndex, unsigned char* image)
{
  unsigned char R[_cellsX * _cellsY];
  unsigned char G[_cellsX * _cellsY];
  unsigned char B[_cellsX * _cellsY];
  unsigned int ctr = 0;
  unsigned im_ctr = 0;
  double cTsd;

  // initialize arrays for RGB
  for (unsigned int i = 0; i < _cellsX * _cellsY; i++)
  {
    R[i] = 0;
    G[i] = 0;
    B[i] = 0;
  }

  // iterate over given slice, generate 2D-Picture
  for (unsigned int row = 0; row < _cellsY; row++)
  {
    for (unsigned int col = 0; col < _cellsX; col++)
    {
      // Get current tsd
      cTsd = _space[depthIndex][row][col].tsd;


      if (isnan(cTsd))
          G[im_ctr++] = (unsigned char) 150; //row*_cellsX+col
      // Blue for depth behind Voxel
      else if (cTsd > 0)
          B[im_ctr++] = (unsigned char) (cTsd * RGB_MAX + 0.5); //row*_cellsX+col


      // Red for depth in front of Voxel
      else if (cTsd < 0)
          R[im_ctr++] = (unsigned char) ((cTsd * -1.0) * RGB_MAX + 0.5); //row*_cellsX+col
    }
  }

  //put components together to complete picture
  for (unsigned int i = 0; i < _cellsX * _cellsY * 3; i++)
  {
    image[i]   = R[ctr];
    image[++i] = G[ctr];
    image[++i] = B[ctr];
    ctr++;
  }

  return (true);
}*/

void TsdSpace::serialize(const char* filename)
{
  ofstream f;
  f.open(filename);

  f << _voxelSize << " " << (int)_layoutPartition << " " << (int)_layoutSpace << " " << _maxTruncation << endl;

  for(int pz=0; pz<_partitionsInZ; pz++)
  {
    for(int py=0; py<_partitionsInY; py++)
    {
      for(int px=0; px<_partitionsInX; px++)
      {
        f << _partitions[pz][py][px]->getInitWeight() << endl;
        bool initialized = _partitions[pz][py][px]->isInitialized();
        f << initialized << endl;
        if(initialized) _partitions[pz][py][px]->serialize(&f);
      }
    }
  }

  LOGMSG(DBG_WARN, "Saved file: " << filename);
  f.close();
}

TsdSpace* TsdSpace::load(const char* filename)
{
  ifstream f;
  f.open(filename, ios_base::in);

  if(!f)
  {
    std::cout << filename << " is no file!" << std::endl;
    abort();
  }

  double voxelSize;
  EnumTsdSpaceLayout layoutPartition;
  EnumTsdSpaceLayout layoutSpace;
  int lp, ls;
  double maxTruncation;

  f >> voxelSize >> lp >> ls >> maxTruncation;
  layoutPartition = (EnumTsdSpaceLayout)lp;
  layoutSpace = (EnumTsdSpaceLayout)ls;

  TsdSpace* space = new TsdSpace(voxelSize, layoutPartition, layoutSpace);
  space->setMaxTruncation(maxTruncation);
  TsdSpacePartition**** partitions = space->getPartitions();

  for(int pz=0; pz<space->getPartitionsInZ(); pz++)
  {
    for(int py=0; py<space->getPartitionsInY(); py++)
    {
      for(int px=0; px<space->getPartitionsInX(); px++)
      {
        double initWeight;
        f >> initWeight;
        partitions[pz][py][px]->setInitWeight(initWeight);
        bool initialized;
        f >> initialized;
        if(initialized) partitions[pz][py][px]->load(&f);
      }
    }
  }

  f.close();

  return space;
}

}
