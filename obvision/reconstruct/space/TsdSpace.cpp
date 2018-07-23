#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/tools.h"
#include "TsdSpace.h"
#include "TsdSpaceBranch.h"
#include "SensorProjective3D.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>

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

  _lutIndex2PartitionX = new int[_cellsX];
  _lutIndex2CellX = new int[_cellsX];
  _lutIndex2PartitionY = _lutIndex2PartitionX;
  _lutIndex2CellY = _lutIndex2CellX;
  _lutIndex2PartitionZ = _lutIndex2PartitionX;
  _lutIndex2CellZ = _lutIndex2CellX;

  _lutIndex2Partition = new int[_cellsX];
  _lutIndex2Cell = new int[_cellsX];
  for(unsigned int i=0; i<_cellsX; i++)
  {
    _lutIndex2Partition[i] = i / dimPartition;
    _lutIndex2Cell[i] = i % dimPartition;
    _lutIndex2PartitionX[i] = i / dimPartition;
    _lutIndex2CellX[i] = i % dimPartition;
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

TsdSpace::TsdSpace(const double voxelSize, const EnumTsdSpaceLayout layoutPartition, const unsigned int cellsX, const unsigned int cellsY, const unsigned int cellsZ)
{
  _voxelSize = voxelSize;
  _invVoxelSize = 1.0 / _voxelSize;

  _layoutPartition = layoutPartition;

  // determine number of voxels in each dimension
  _cellsX = cellsX;
  _cellsY = cellsY;
  _cellsZ = cellsZ;

  unsigned int dimPartition = 1u << layoutPartition;

  if(dimPartition > _cellsX)
  {
    LOGMSG(DBG_ERROR, "Insufficient partition size : " << dimPartition << "x" << dimPartition << "x" << dimPartition << " in " << _cellsX << "x" << _cellsY << "x" << _cellsZ << " space");
    return;
  }


  if (((cellsX % dimPartition) + (cellsY % dimPartition) + (cellsZ % dimPartition)) > 0){
    LOGMSG(DBG_ERROR, "Cells must be a multiple of partition size: partition size=" << dimPartition << " cellsX= " << cellsX << " cellsY= " << cellsY << " cellsZ= " << cellsZ);
    return;
  }


  _partitionsInX = _cellsX / dimPartition;
  _partitionsInY = _cellsY / dimPartition;
  _partitionsInZ = _cellsZ / dimPartition;

  _lutIndex2PartitionX = new int[_cellsX];
  _lutIndex2CellX = new int[_cellsX];
  _lutIndex2PartitionY = new int[_cellsY];
  _lutIndex2CellY = new int[_cellsY];
  _lutIndex2PartitionZ = new int[_cellsZ];
  _lutIndex2CellZ = new int[_cellsZ];

  for(unsigned int i = 0; i < _cellsX; i++)
  {
    _lutIndex2PartitionX[i] = i / dimPartition;
    _lutIndex2CellX[i] = i % dimPartition;
  }

  for(unsigned int i = 0; i < _cellsY; i++)
  {
    _lutIndex2PartitionY[i] = i / dimPartition;
    _lutIndex2CellY[i] = i % dimPartition;
  }

  for(unsigned int i = 0; i < _cellsZ; i++)
  {
    _lutIndex2PartitionZ[i] = i / dimPartition;
    _lutIndex2CellZ[i] = i % dimPartition;
  }

  _maxTruncation = 2.0 * voxelSize;

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

  for(int pz = 0; pz < _partitionsInZ; pz++)
  {
    for(int py = 0; py < _partitionsInY; py++)
    {
      for(int px = 0; px < _partitionsInX; px++)
      {
        _partitions[pz][py][px] = new TsdSpacePartition(px * dimPartition, py * dimPartition, pz * dimPartition, dimPartition, dimPartition, dimPartition, voxelSize);
      }
    }
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

unsigned int TsdSpace::getPartitionSize()
{
  return _partitions[0][0][0]->getWidth();
}

void TsdSpace::getCentroid(obfloat centroid[3])
{
  centroid[0] = (_minX + _maxX) * 0.5;
  centroid[1] = (_minY + _maxY) * 0.5;
  centroid[2] = (_minZ + _maxZ) * 0.5;
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

  //std::cout << __PRETTY_FUNCTION__ << " x y z " << x << " " << y << "  " << z  << std::endl;
  int px = _lutIndex2PartitionX[x];
  int py = _lutIndex2PartitionY[y];
  int pz = _lutIndex2PartitionZ[z];
  //std::cout << __PRETTY_FUNCTION__ << " px py zz " << px << " " << py << "  " << pz  << std::endl;

  return _partitions[pz][py][px]->isInitialized();
}

bool TsdSpace::isInsideSpace(Sensor* sensor)
{
  obfloat coord[3];
  sensor->getPosition(coord);
  return (coord[0]>_minX && coord[0]<_maxX && coord[1]>_minY && coord[1]<_maxY && coord[2]>_minZ && coord[2]<_maxZ);
}

bool TsdSpace::isInsideSpace(const Eigen::Vector3f& pos)
{
  // std::cout << __PRETTY_FUNCTION__ << " minX " << _minX << " " << pos(0) << " " << _maxX <<  std::endl;
  return (pos(0)>_minX && pos(0)<_maxX && pos(1)>_minY && pos(1)<_maxY && pos(2)>_minZ && pos(2)<_maxZ);
}


//{
//
//}

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
                if((sd > -_maxTruncation) && (sd < _maxTruncation))
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

void TsdSpace::push(stdVecEig3f& points)
{

  std::vector<std::vector<PointWithAngle>* > slicesZ(this->getZDimension(), NULL);
  std::cout << __PRETTY_FUNCTION__ << "sort points z-wise" << std::endl;
  std::sort(points.begin(), points.end(), compareZ);
  obfloat nVxlSize = this->getVoxelSize();
  std::vector<PointWithAngle>* subCloud;
  const unsigned int nBinAngleRes = 1000;
  const double binRes = (2.0 * M_PI) / static_cast<double>(nBinAngleRes);
  //static_cast<unsigned int>(binRes);
  std::vector<stdVecEig3f> binAngles(this->getZDimension());
  std::cout << __PRETTY_FUNCTION__ << " create sliced point clouds" << std::endl;
  unsigned int i = 0;
  unsigned int n = 1; //todo: why 1...should be 0. 1 times vxlsize for first depth is ok but using it as idx will go wrong
  obfloat centerSpace[3] = {0.0};
  this->getCentroid(centerSpace);

  while(1)
  {
    subCloud = new std::vector<PointWithAngle>;
    nVxlSize = static_cast<obfloat>(n) * this->getVoxelSize();
    binAngles[n - 1].resize(nBinAngleRes, Eigen::Vector3f(NAN, NAN, NAN));
    unsigned int naaas = 0;
    while(1)
    {
      if(points[i](2) > nVxlSize)
      {
        //nVxlSize += this->getVoxelSize();
        n++;
        break;
      }
      const double angleCurrent = std::atan2(points[i](1) - centerSpace[1], points[i](0) - centerSpace[0]);
      const unsigned int idxBin = static_cast<unsigned int>(std::floor((M_PI + angleCurrent) / binRes));
      std::cout << __PRETTY_FUNCTION__ << " idxbin  = " << M_PI + angleCurrent << " / " << binRes << " = " << idxBin << std::endl;
      if(idxBin < nBinAngleRes)
      {
        if(!std::isnan(binAngles[n - 1][idxBin](0)))
        {
          //std::cout << __PRETTY_FUNCTION__ << " Error! Bin idx " << idxBin << " already full" << binAngles[n - 1][idxBin](0) << std::endl;
          //std::cout << M_PI + std::atan2(binAngles[n - 1][idxBin](1) - centerSpace[1], binAngles[n - 1][idxBin](0) - centerSpace[0]);// << std::endl;
          //std::cout << M_PI + std::atan2(binAngles[n - 1][idxBin](1) - centerSpace[1], binAngles[n - 1][idxBin](0) - centerSpace[0]) / binRes << std::endl;
          // std::cout << " na aaaah " << std::endl;
          naaas++;
          //abort();
        }
        binAngles[n - 1][idxBin] = points[i];
      }
      if(naaas)
        std::cout << __PRETTY_FUNCTION__ << " naas " << naaas << " of " << subCloud->size() << std::endl;
      subCloud->push_back(PointWithAngle(points[i], std::atan2(points[i](1) - centerSpace[1], points[i](0) - centerSpace[0])));
      //   std::cout << __PRETTY_FUNCTION__ << "i " << i << " of " << points.size() << " n " << n <<std::endl;
      if(++i >= points.size())
        break;
    }

    if(subCloud->size())
    {
      //     std::cout << __PRETTY_FUNCTION__ << " Adding slice with " << subCloud->size() << " points at pos " << n << std::endl;
      slicesZ[n - 1] = subCloud;
    }
    if(i >= points.size())
      break;
  }

  //  for(auto& iter : slicesZ)
  //  {
  //    if(iter)
  //      std::cout << __PRETTY_FUNCTION__ << " points in slice " << iter->size() << std::endl;
  //  }

  std::cout << __PRETTY_FUNCTION__ << " sort angle" << std::endl;
  for(auto& iter : slicesZ)
  {
    if(iter)
      std::sort(iter->begin(), iter->end(), compareAngle);
  }

  Matrix* partCoords = TsdSpacePartition::getPartitionCoords();
  Matrix* cellCoordsHom = TsdSpacePartition::getCellCoordsHom();

  unsigned int partSize = (_partitions[0][0][0])->getSize();
  //      int* idx = new int[partSize];
  unsigned int ctr = 0;
  std::cout << __PRETTY_FUNCTION__ << " start pushing" << std::endl;


  //  float arsch = centerSpace[0];
  //  float arscharsch = centerSpace[1];

  Timer timer;
  timer.start();

#pragma omp parallel
  {
#pragma omp for schedule(dynamic)

    for(int pz=0; pz<this->getPartitionsInZ(); pz++)
    {
      for(int py=0; py<_partitionsInY; py++)
      {
        for(int px=0; px<_partitionsInX; px++)
        {
          TsdSpacePartition* part = _partitions[pz][py][px];
          //std::cout << __PRETTY_FUNCTION__ << " px py pz" << px << " " << py << " " << pz << std::endl;
          obfloat t[3];
          part->getCellCoordsOffset(t);
          const unsigned int zSlicesIdcs = t[2]/this->getVoxelSize();
          bool found = false;
          for(unsigned int i = zSlicesIdcs; i < zSlicesIdcs + static_cast<obfloat>(part->getDepth()) / this->getVoxelSize(); i++)
          {
            if(slicesZ[i])
            {
              found = true;
              break;
            }
          }
          if(!found)
            continue;
          for(unsigned int c = 0; c < partSize; c++)
          {

            Eigen::Vector3f coordCell;
            for(unsigned int i = 0; i < 3; i++)
              coordCell(i) = ((*cellCoordsHom)(c, i) + t[i]);
            Eigen::Vector3f slice0(centerSpace[0], centerSpace[1], coordCell(2));
            const double angleCell = M_PI + std::atan2(coordCell(1) - centerSpace[1], coordCell(0) - centerSpace[0]);
            const unsigned int idxBeam = static_cast<unsigned int>(std::floor(angleCell / binRes));
            if(idxBeam >= nBinAngleRes)
            {
              std::cout << __PRETTY_FUNCTION__ << " idx " << idxBeam << " out of range " << nBinAngleRes << " (should not happen)" << std::endl;
              continue;
            }

            const double hyst = 0.05; //half a degree)
            obfloat beam = NAN;
            const unsigned int zIdx = coordCell(2) / this->getVoxelSize();
            if(!slicesZ[zIdx])  //todo: this continue should come earlier to save time
            {
              //    std::cout << __PRETTY_FUNCTION__ << " empty slice " << std::endl;
              continue;
            }
            //            for(unsigned int i = 0; i < slicesZ[zIdx]->size(); i++)
            //            {
            //              if(std::abs(angleCell - (*slicesZ[zIdx])[i].angle) < hyst)
            //              {
            //                Eigen::Vector3f centerVec(centerSpace[0], centerSpace[1], centerSpace[2]);
            //                Eigen::Vector3f beamVec = (*slicesZ[zIdx])[i].point - slice0;
            //                beam = beamVec.norm();
            //                break;
            //              }
            //            }
            //            if(std::isnan(beam))
            //            {
            //              continue;
            //            }
            Eigen::Vector3f beamVec;
            if(std::isnan(binAngles[zIdx][idxBeam](0))) //interpolate between neighbours
            {
              unsigned int iterPos = zIdx;
              unsigned int iterNeg = zIdx;
              Eigen::Vector3f posNeighbour(NAN, NAN, NAN);
              Eigen::Vector3f negNeighbour(NAN, NAN, NAN);
              while(1)
              {
                //       if(std::isnan(posNeighbour(0)))
                iterPos++;
                //     if(std::isnan(negNeighbour(0)))
                iterNeg--;
                if((iterPos >= nBinAngleRes) || (iterNeg >= nBinAngleRes))
                {
                  break;
                }
                if(((iterPos - zIdx) > 20) || ((zIdx - iterNeg) > 20))
                  break;
                if(!std::isnan(binAngles[zIdx][iterNeg](0)))
                  negNeighbour = binAngles[zIdx][iterNeg];
                if(!std::isnan(binAngles[zIdx][iterPos](0)))
                  posNeighbour = binAngles[zIdx][iterPos];
              }
              if(std::isnan(binAngles[zIdx][iterNeg](0)) && !std::isnan(binAngles[zIdx][iterPos](0)))
                beamVec = binAngles[zIdx][iterPos];
              else if(!std::isnan(binAngles[zIdx][iterNeg](0)) && std::isnan(binAngles[zIdx][iterPos](0)))
                beamVec = binAngles[zIdx][iterNeg];
              else if(!std::isnan(binAngles[zIdx][iterNeg](0)) && !std::isnan(binAngles[zIdx][iterPos](0)))
              {
                if((iterPos - zIdx) > (zIdx - iterNeg))
                  beamVec = binAngles[zIdx][iterNeg];
                else if((iterPos - zIdx) < (zIdx - iterNeg))
                  beamVec = binAngles[zIdx][iterPos];
                else
                  beamVec = (binAngles[zIdx][iterPos] + binAngles[zIdx][iterPos]) / 2.0;
              }
              else
                continue;
              beamVec -= slice0;
            }
            else
              beamVec = binAngles[zIdx][idxBeam] - slice0;
            beam = beamVec.norm();
            obfloat sd = beam - (coordCell - slice0).norm();
            if((sd >= -_maxTruncation))// && (sd <= _maxTruncation))
            {
              part->init();
              part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2), sd, _maxTruncation, NULL);
#pragma omp critical
              {
                ctr++;
              }

            }
          }
        }
      }
    }
  }
  std::cout << __PRETTY_FUNCTION__ << "distances pushed " << ctr << " in " << timer.elapsed() << " s " << std::endl;
  propagateBorders();
}

void TsdSpace::push(const std::vector<stdVecEig3f>& data, const unsigned int width, const unsigned int height, const Eigen::Vector3f& t, const double resDepth,
    const double resHor)
{
  omp_lock_t writelock;
  omp_init_lock(&writelock);
  Timer timer;
  timer.start();

  Matrix* partCoords = TsdSpacePartition::getPartitionCoords();
  Matrix* cellCoordsHom = TsdSpacePartition::getCellCoordsHom();

  unsigned int partSize = (_partitions[0][0][0])->getSize();
  unsigned int ctr = 0;

#pragma omp parallel
  {
#pragma omp for schedule(dynamic)

    for(int pz=0; pz<this->getPartitionsInZ(); pz++)
    {
      for(int py=0; py<_partitionsInY; py++)
      {
        for(int px=0; px<_partitionsInX; px++)
        {
          TsdSpacePartition* part = _partitions[pz][py][px];
          //std::cout << __PRETTY_FUNCTION__ << " px py pz" << px << " " << py << " " << pz << std::endl;
          obfloat partOffset[3];
          part->getCellCoordsOffset(partOffset);

          for(unsigned int c = 0; c < partSize; c++)
          {
            Eigen::Vector3f coordCell;
            for(unsigned int i = 0; i < 3; i++)
              coordCell(i) = ((*cellCoordsHom)(c, i) + partOffset[i]);
            Eigen::Vector3f sliceCenter(t(0), t(1), coordCell(2));
            const double angleCell = M_PI + std::atan2(coordCell(1) - t(1), coordCell(0) - t(0));
            const unsigned int idxBeam = static_cast<unsigned int>(std::floor(angleCell / resHor));

            if(idxBeam >= width)
            {
              std::cout << __PRETTY_FUNCTION__ << " idx(h) " << idxBeam << " out of range " << width << " (should not happen)" << std::endl;
              continue;
            }
            const unsigned int zIdx = static_cast<unsigned int>(std::floor(coordCell(2) / resDepth));

            if(zIdx >= height)
            {
              //       std::cout << __PRETTY_FUNCTION__ << " idx(v) " << zIdx << " out of range " << height << std::endl;
              continue;
            }

            stdVecEig3f beams = data[zIdx * width + idxBeam]; //todo: inefficient. User ptr or ref instead
            Eigen::Vector3f beam(0.0, 0.0, 0.0);
            if(!beams.size())
            {
              Eigen::Vector3f mean(0.0, 0.0, 0.0);
              //stdVecEig3f subCluster;
              unsigned int meanCtr = 0;
              unsigned int fieldCtr = 0;
              for(unsigned int i = zIdx - 5; i <= zIdx + 5; i++)
              {
                for(unsigned int j = idxBeam - 5; j <= idxBeam + 5; j++)
                {
                  if((i >= height) || (j >= width))
                    continue;
                  if(!data[i * width + j].size())
                    continue;
                  if(data[i * width + j].size() >= 1)
                  {
                    for(auto& iter : data[i * width + j])
                    {
                      if((coordCell - iter).norm() > 0.3)
                        continue;
                      mean += iter;
                      // subCluster.push_back(iter);
                      meanCtr++;
                    }
                    fieldCtr++;
                  }
                  else if(data[i * width + j].size() == 1)
                  {
                    mean += *data[i * width + j].begin();
                    // subCluster.push_back(*data[i * width + j].begin());
                    meanCtr++;
                    fieldCtr++;
                  }

                }
              }

              if(fieldCtr < 1)
                continue;
              else
              {
                //                pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
                //                subcloud->resize(subCluster.size());
                //                for(unsigned int i = 0; i < subCluster.size(); i++)
                //                  subcloud->points[i] = pcl::PointXYZ(subCluster[i](0), subCluster[i](1), subCluster[i](2));
                //
                //                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                //                  tree->setInputCloud(subcloud);
                //
                //                  std::vector<pcl::PointIndices> cluster_indices;
                //                  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                //                  ec.setClusterTolerance(5.0);
                //                  ec.setMinClusterSize  (3);
                //                  ec.setMaxClusterSize  (200);
                //                  ec.setSearchMethod(tree);
                //                  ec.setInputCloud(subcloud);
                //                  ec.extract(cluster_indices);
                ////                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                ////                  pcl::PointCloud<pcl::PointXYZ>::Ptr input = subcloud.makeShared();
                ////                  sor.setInputCloud(input);
                ////                  sor.setMeanK(5);
                ////                  sor.setStddevMulThresh(5.0);
                ////                  sor.filter(subcloud);
                //                  pcl::CentroidPoint<pcl::PointXYZ> centroid;
                //                  for(auto& iter : subcloud->points)
                //                  {
                //                    centroid.add(iter);
                //                  }
                //                  pcl::PointXYZ pclMean;
                //                  centroid.get(pclMean);
                //
                //                  beam = Eigen::Vector3f(pclMean.x, pclMean.y, pclMean.z);
                beam = mean / static_cast<float>(meanCtr);
                // std::cout << " fieldctr = " << fieldCtr << std::endl;
              }
            }
            else if(beams.size() > 1)
            {
              for(auto& iter : beams)
                beam += iter;
              beam /= static_cast<float>(beams.size());
            }
            else
              beam = *beams.begin();
            obfloat sd = (beam - sliceCenter).norm() - (coordCell - sliceCenter).norm();
            if((sd >  -2.0 * _maxTruncation))// && (sd <  2.0 * _maxTruncation))
            {
              part->init();
              part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2), sd, _maxTruncation, NULL);
              //#pragma omp critical
              omp_set_lock(&writelock);
              //            {
              ctr++;
              //                if(beams.size() > 1)
              //                {
              //                  Eigen::Vector3f mean;
              //                  for(auto& iter : beams)
              //                  {
              //                    mean += iter;
              //                    std::cout << iter(0) << " " << iter(1) << " " << iter(2) << " ";
              //                  }
              //                  mean /= static_cast<float>(beams.size());
              //                  std::cout << mean(0) << " " << mean(1) << " " << mean(2);
              //                  std::cout << std::endl;
              //                  std::cout << std::endl;
              //                }
              //          }
              omp_unset_lock(&writelock);
            }
          }
        }
      }
    }
  }
  omp_destroy_lock(&writelock);
  std::cout << __PRETTY_FUNCTION__ << "distances pushed " << ctr << " in " << timer.elapsed() << " s " << std::endl;
  propagateBorders();
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

void TsdSpace::pushForward(Sensor* const sensor)
{
  Timer timer;
  timer.start();

  double* data = sensor->getRealMeasurementData();
  unsigned char* rgbs = sensor->getRealMeasurementRGB();
  bool* mask = sensor->getRealMeasurementMask();
  int dim = 3;
  //unsigned char* rgb = sensor->getRealMeasurementRGB();

  obfloat tr[dim];
  sensor->getPosition(tr);

  //Matrix* partCoords = TsdSpacePartition::getPartitionCoords();
  //Matrix* cellCoordsHom = TsdSpacePartition::getCellCoordsHom();

  int rayCount = sensor->getRealMeasurementSize();

  Matrix* rays = sensor->getNormalizedRayMap(1);

  TsdSpacePartition* part = _partitions[0][0][0];
  unsigned int partSize[3] = {part->getWidth(),part->getHeight(),part->getDepth()};


  for(int zTest = 0; zTest < 1; ++zTest)
  {
    // 1) for each measurement in sensor, take distance and normal
    for(int beam = 0; beam < rayCount; ++beam)
    {
      if(isinf(data[beam]) || !mask[beam])
      {
        continue;
      }

      unsigned char rgb[3] = {rgbs[beam * 3], rgbs[beam * 3 + 1], rgbs[beam * 3 + 2]};

      // 2) addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2), sd, _maxTruncation, color);
      //    to cells along ray [-truncation radius; truncation radius]:

      obfloat crd[dim];
      obfloat beamCrd[dim];
      for(int i = 0; i < dim; ++i)
      {
        crd[i] = (*rays)(i, beam) * data[beam] + tr[i];
        //go back -_maxTruncation on beam:
        beamCrd[i] = (crd[i] + (*rays)(i, beam) * (-_maxTruncation));
      }

      //Stepsize for moving on beam
      double stepSize = _voxelSize / 2;
      int steps = _maxTruncation / stepSize;

      for(int i = 0; i < steps * 2; ++i)
      {
        // => project measurement to cell index
        int vIdx[dim];
        int pIdx[dim];
        int cIdx[dim];

        for(int j = 0; j < dim; ++j)
        {
          vIdx[j] = (int)(beamCrd[j] / _voxelSize + 0.5);
          pIdx[j] = vIdx[j] / partSize[j];
          cIdx[j] = vIdx[j] % partSize[j];
        }


        part = _partitions[pIdx[2]][pIdx[1]][pIdx[0]];

        obfloat sd = data[beam] - euklideanDistance<obfloat>(beamCrd, tr, 3);

        part->init();
        part->addTsd(cIdx[0], cIdx[1], cIdx[2], sd, _maxTruncation, rgb);
#if PRINTSTATISTICS
#pragma omp critical
        {
          _distancesPushed++;
        }
#endif

        for(int j = 0; j < dim; ++j)
        {
          beamCrd[j] += (*rays)(j, beam) * stepSize;
        }
      }
    }
    tr[2] += _voxelSize;
  }

  propagateBorders();

#if PRINTSTATISTICS
  LOGMSG(DBG_DEBUG, "Distances pushed: " << _distancesPushed);
#endif

  LOGMSG(DBG_DEBUG, "Elapsed push: " << timer.elapsed() << "s, Initialized partitions: " << TsdSpacePartition::getInitializedPartitionSize());
}

void TsdSpace::pushForward(const stdVecEig3f& points)
{
  std::cout << __PRETTY_FUNCTION__ << "hello" << std::endl;
  TsdSpacePartition* part = _partitions[0][0][0];
  unsigned int partSize[3] = {part->getWidth(),part->getHeight(),part->getDepth()};
  std::cout << __PRETTY_FUNCTION__ << " salllutt" << std::endl;
  unsigned int ctr = 0;
  for(stdVecEig3f::const_iterator iter = points.begin(); iter < points.end(); iter++)
  {
    Eigen::Vector3f pos(0.0, 0.0, iter->z());   //(*iter)(2)
    const Eigen::Vector3f org = pos;
    Eigen::Vector3f dir(iter->x(), iter->y(), 0.0);
    dir = (dir / dir.norm()) * this->getVoxelSize() / 2.0;
    while(1)
    {
      std::cout << __PRETTY_FUNCTION__ << " hello again" << std::endl;
      unsigned int vIdx[3];
      unsigned int pIdx[3];
      unsigned int cIdx[3];
      for(int j = 0; j < 3; ++j)
      {
        vIdx[j] = static_cast<unsigned int>(std::floor(pos(j) / _voxelSize));
        pIdx[j] = vIdx[j] / partSize[j];
        cIdx[j] = vIdx[j] % partSize[j];
        std::cout << __PRETTY_FUNCTION__ << "pos v p c " << pos(j) << " " << vIdx[j] << " " << pIdx[j] << " " << cIdx[j] << std::endl;
      }
      pos += dir;

      part = _partitions[pIdx[2]][pIdx[1]][pIdx[0]];
      obfloat sd = iter->norm() - pos.norm();
      if((sd > 2.0 * _maxTruncation) || (sd < -2.0 * _maxTruncation))
        continue;
      std::cout << __PRETTY_FUNCTION__ << " sd " << sd << std::endl;
      part->init();
      part->addTsd(cIdx[0], cIdx[1], cIdx[2], sd, _maxTruncation, NULL);
      ctr++;
      if(sd < 2.0 * -_maxTruncation)
        break;
    }

    //    Point vec;  //start vector in zaxis
    //    vec.x = 0.0;//iter->x;
    //    vec.y = 0.0;//iter->y;
    //    vec.z = iter->z;
    //    Point dirVec = iter;
    //    dirVec.z = 0.0;
    //    dirVec =
  }
  std::cout << " pushed " << ctr << " points" << std::endl;
  propagateBorders();
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
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz))
    return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2PartitionX[xIdx];
  int py = _lutIndex2PartitionY[yIdx];
  int pz = _lutIndex2PartitionZ[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized())
    return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2CellX[xIdx];
  int y = _lutIndex2CellY[yIdx];
  int z = _lutIndex2CellZ[zIdx];

  obfloat wx = fabs((coord[0] - dx) * _invVoxelSize);
  obfloat wy = fabs((coord[1] - dy) * _invVoxelSize);
  obfloat wz = fabs((coord[2] - dz) * _invVoxelSize);

  *tsd = part->interpolateTrilinear(x, y, z, wx, wy, wz);

  if(isnan(*tsd))
    return INTERPOLATE_ISNAN;

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
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz))
    return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2PartitionX[xIdx];
  int py = _lutIndex2PartitionY[yIdx];
  int pz = _lutIndex2PartitionZ[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized())
    return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2CellX[xIdx];
  int y = _lutIndex2CellY[yIdx];
  int z = _lutIndex2CellZ[zIdx];

  *tsd = (*part)(z, y, x);

  if(isnan(*tsd))
    return INTERPOLATE_ISNAN;

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
  if(!coord2Index(coord, &xIdx, &yIdx, &zIdx, &dx, &dy, &dz))
    return INTERPOLATE_INVALIDINDEX;

  int px = _lutIndex2PartitionX[xIdx];
  int py = _lutIndex2PartitionY[yIdx];
  int pz = _lutIndex2PartitionZ[zIdx];

  TsdSpacePartition* part = _partitions[pz][py][px];
  if(!part->isInitialized())
    return INTERPOLATE_EMPTYPARTITION;

  int x = _lutIndex2CellX[xIdx];
  int y = _lutIndex2CellY[yIdx];
  int z = _lutIndex2CellZ[zIdx];

  double wx = fabs((coord[0] - dx) * _invVoxelSize);
  double wy = fabs((coord[1] - dy) * _invVoxelSize);
  double wz = fabs((coord[2] - dz) * _invVoxelSize);

  unsigned char pRGB[8][3];

  part->getRGB(z + 0, y + 0, x + 0, pRGB[0]);
  part->getRGB(z + 1, y + 0, x + 0, pRGB[1]);
  part->getRGB(z + 0, y + 1, x + 0, pRGB[2]);
  part->getRGB(z + 1, y + 1, x + 0, pRGB[3]);
  part->getRGB(z + 0, y + 0, x + 1, pRGB[4]);
  part->getRGB(z + 1, y + 0, x + 1, pRGB[5]);
  part->getRGB(z + 0, y + 1, x + 1, pRGB[6]);
  part->getRGB(z + 1, y + 1, x + 1, pRGB[7]);

  double pw[8];
  pw[0] = (1. - wx) * (1. - wy) * (1. - wz);
  pw[1] = (1. - wx) * (1. - wy) * wz;
  pw[2] = (1. - wx) * wy * (1. - wz);
  pw[3] = (1. - wx) * wy * wz;
  pw[4] = wx * (1. - wy) * (1. - wz);
  pw[5] = wx * (1. - wy) * wz;
  pw[6] = wx * wy * (1. - wz);
  pw[7] = wx * wy * wz;

  memset(rgb, 0, 3);
  for(unsigned int i = 0; i < 8; i++)
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

  //f << _voxelSize << " " << (int)_layoutPartition << " " << (int)_layoutSpace << " " << _maxTruncation << endl;
  f << _voxelSize << " " << (int)_layoutPartition << " " << _cellsX << " " << " " << _cellsY << " " << _cellsZ << " " <<_maxTruncation << endl;

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
  f << "EOF" << std::endl;
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
  unsigned int cellsX = 0;
  unsigned int cellsY = 0;
  unsigned int cellsZ = 0;
  double maxTruncation;

  //f >> voxelSize >> lp >> ls >> maxTruncation;
  f >> voxelSize >> lp >> cellsX >> cellsY >> cellsZ >> maxTruncation;
  layoutPartition = (EnumTsdSpaceLayout)lp;
  layoutSpace = (EnumTsdSpaceLayout)ls;

  // TsdSpace* space = new TsdSpace(voxelSize, layoutPartition, layoutSpace);
  TsdSpace* space = new TsdSpace(voxelSize, layoutPartition, cellsX, cellsY, cellsZ);
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
  _minX = 0.0;
  _minY = 0.0;
  _minZ = 0.0;
  _maxX = static_cast<obfloat>(_cellsX) * _voxelSize;
  _maxY = static_cast<obfloat>(_cellsY) * _voxelSize;
  _maxZ = static_cast<obfloat>(_cellsZ) * _voxelSize;
  f.close();

  return space;
}

bool TsdSpace::sliceImage(const unsigned int idx, const EnumSpaceAxis& axis, std::vector<unsigned char>* const rgb)
{
  rgb->clear();
  const unsigned int sizePartition = this->getPartitionSize();
  bool something = false;
  if(axis == Z)
  {
    const unsigned int partIdxZ  = idx / sizePartition;
    const unsigned int voxelIdxZ = idx - partIdxZ * sizePartition;
    rgb->resize(_cellsX * _cellsY * 3, 255);
    for(unsigned int i = 0; i < _cellsY; i++)
    {
      for(unsigned int j = 0; j < _cellsX; j++)
      {
        const unsigned int partIdxX = j / sizePartition;
        const unsigned int partIdxY = i / sizePartition;

        TsdSpacePartition* partCur  = _partitions[partIdxZ][partIdxY][partIdxX];

        if(!partCur->isInitialized())
          continue;
        if(partCur->isEmpty())
          continue;
        //something = true;
        else
          if(!something)
            something = true;

        const unsigned int  voxelIdxX = j - partIdxX * sizePartition;
        const unsigned int  voxelIdxY = i - partIdxY * sizePartition;
        const double        tsdCur = partCur->_space[voxelIdxZ][voxelIdxY][voxelIdxX].tsd;
        const unsigned char color = static_cast<unsigned char>(255.0 * std::abs(tsdCur));
        if(tsdCur < 0.0)   //behind voxel RED
        {
          (*rgb)[(i * _cellsX + j) * 3    ] = color;
          (*rgb)[(i * _cellsX + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 2] = 0;
        }
        else if(tsdCur > 0.0)  //in front of voxel BLUE
        {
          (*rgb)[(i * _cellsX + j) * 3    ] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 2] = color;
        }
        else
          continue;
      }
    }
  }
  else if(axis == Y)
  {
    const unsigned int partIdxY  = idx / sizePartition;
    const unsigned int voxelIdxY = idx - partIdxY * sizePartition;
    rgb->resize(_cellsX * _cellsZ * 3, 255);
    for(unsigned int i = 0; i < _cellsZ; i++)
    {
      for(unsigned int j = 0; j < _cellsX; j++)
      {
        const unsigned int partIdxX = j / sizePartition;
        const unsigned int partIdxZ = i / sizePartition;

        TsdSpacePartition* partCur  = _partitions[partIdxZ][partIdxY][partIdxX];

        if(!partCur->isInitialized())
          continue;
        if(partCur->isEmpty())
          continue;
        else
          if(!something)
            something = true;

        const unsigned int  voxelIdxX = j - partIdxX * sizePartition;
        const unsigned int  voxelIdxZ = i - partIdxZ * sizePartition;
        const double        tsdCur = partCur->_space[voxelIdxZ][voxelIdxY][voxelIdxX].tsd;
        const unsigned char color = static_cast<unsigned char>(255.0 * std::abs(tsdCur));
        if(tsdCur < 0.0)   //behind voxel RED
        {
          (*rgb)[(i * _cellsX + j) * 3    ] = color;
          (*rgb)[(i * _cellsX + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 2] = 0;
        }
        else if(tsdCur > 0.0)   //in front of voxel BLUE
        {
          (*rgb)[(i * _cellsX + j) * 3    ] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsX + j) * 3 + 2] = color;
        }
        else
          continue;
      }
    }
  }
  else if(axis == X)
  {
    const unsigned int partIdxX  = idx / sizePartition;
    const unsigned int voxelIdxX = idx - partIdxX * sizePartition;
    rgb->resize(_cellsY * _cellsZ * 3, 255);
    for(unsigned int i = 0; i < _cellsZ; i++)
    {
      for(unsigned int j = 0; j < _cellsY; j++)
      {
        const unsigned int partIdxY = j / sizePartition;
        const unsigned int partIdxZ = i / sizePartition;

        TsdSpacePartition* partCur  = _partitions[partIdxZ][partIdxY][partIdxX];

        if(!partCur->isInitialized())
          continue;
        if(partCur->isEmpty())
          continue;
        else
          if(!something)
            something = true;

        const unsigned int  voxelIdxY = j - partIdxY * sizePartition;
        const unsigned int  voxelIdxZ = i - partIdxZ * sizePartition;
        const double        tsdCur = partCur->_space[voxelIdxZ][voxelIdxY][voxelIdxX].tsd;
        const unsigned char color = static_cast<unsigned char>(255.0 * std::abs(tsdCur));
        if(tsdCur < 0.0)   //behind voxel RED
        {
          (*rgb)[(i * _cellsY + j) * 3    ] = color;
          (*rgb)[(i * _cellsY + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsY + j) * 3 + 2] = 0;
        }
        else if(tsdCur > 0.0)   //in front of voxel BLUE
        {
          (*rgb)[(i * _cellsY + j) * 3    ] = 0;
          (*rgb)[(i * _cellsY + j) * 3 + 1] = 0;
          (*rgb)[(i * _cellsY + j) * 3 + 2] = color;
        }
        else
          continue;
      }
    }
  }
  return something;
}

void TsdSpace::serializeSliceImages(const EnumSpaceAxis& axis, const std::string& path)
{
  std::string storePath;
  if(path.size())
    storePath = path;
  else
    storePath = "/tmp";
  std::vector<unsigned char> imageBuf;
  if(axis == Z)
  {
    for(unsigned int i = 0; i < _cellsZ; i++)
    {
      if(!this->sliceImage(i, axis, &imageBuf))
        continue;
      std::stringstream ss;
      ss << storePath << "/z_axis_" << i << ".ppm";
      serializePPM(ss.str().c_str(), imageBuf.data(), _cellsX, _cellsY);
    }
  }
  else if(axis == Y)
  {
    for(unsigned int i = 0; i < _cellsY; i++)
    {
      this->sliceImage(i, axis, &imageBuf);
      std::stringstream ss;
      ss << storePath << "/y_axis_" << i << ".ppm";
      serializePPM(ss.str().c_str(), imageBuf.data(), _cellsX, _cellsZ);
    }
  }
  else if(axis == X)
  {
    for(unsigned int i = 0; i < _cellsX; i++)
    {
      this->sliceImage(i, axis, &imageBuf);
      std::stringstream ss;
      ss << storePath << "/x_axis_" << i << ".ppm";
      serializePPM(ss.str().c_str(), imageBuf.data(), _cellsY, _cellsZ);
    }
  }
  else
    return;
}

bool compareZ(const Eigen::Vector3f& var1, const Eigen::Vector3f& var2)
{
  return var1(2) < var2(2);
}

bool compareAngle(const PointWithAngle& var1, const PointWithAngle& var2)
{
  return var1.angle < var2.angle;
}



}
