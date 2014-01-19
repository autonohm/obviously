#ifndef TSDSPACEPARTITION_H
#define TSDSPACEPARTITION_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/space/TsdSpaceComponent.h"

namespace obvious
{

/**
* Struct for Class Tsd_space
* Contains data that is saved in a voxel
* @param tsdf value of the truncated signed distance function
* @param weight used to calculate mean of all points in a voxel
*/
struct TsdVoxel
{
  double tsd;
  double weight;
  unsigned char rgb[3];
};

/**
 * @class TsdSpacePartition
 * @brief Acceleration structure for TsdSpace approach
 * @author Stefan May
 */
class TsdSpacePartition : public TsdSpaceComponent
{
  friend class TsdSpace;
public:

  /**
   * Standard constructor
   * Allocates and initializes space and matrices
   * @param[in] x start index in x-dimension
   * @param[in] y start index in y-dimension
   * @param[in] z start index in z-dimension
   * @param[in] dimX Number of cells in x-dimension
   * @param[in] dimY Number of cells in y-dimension
   * @param[in] dimZ Number of cells in z-dimension
   * @param[in] cellSize Size of cell in meters
   */
  TsdSpacePartition(const unsigned int x, const unsigned int y, const unsigned int z, const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ, const double cellSize);

  ~TsdSpacePartition();

  static int getInitializedPartitionSize();

  static int getDistancesPushed();

  void reset();

  double& operator () (unsigned int z, unsigned int y, unsigned int x);

  void init();

  bool isInitialized();

  bool isEmpty();

  unsigned int getX();

  unsigned int getY();

  unsigned int getZ();

  static Matrix* getCellCoordsHom();

  void getCellCoordsOffset(double offset[3]);

  static Matrix* getPartitionCoords();

  unsigned int getWidth();

  unsigned int getHeight();

  unsigned int getDepth();

  unsigned int getSize();

  void addTsd(const unsigned int x, const unsigned int y, const unsigned int z, const double sd, const double maxTruncation);

  virtual void increaseEmptiness();

  double interpolateTrilinear(int x, int y, int z, double dx, double dy, double dz);

private:

  TsdVoxel*** _space;

  double _cellSize;

  double _cellCoordsOffset[3];

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _cellsZ;

  unsigned int _x;

  unsigned int _y;

  unsigned int _z;

  double _initWeight;
};

}

#endif
