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
  obfloat tsd;
  obfloat weight;
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
  TsdSpacePartition(const unsigned int x, const unsigned int y, const unsigned int z, const unsigned int dimX, const unsigned int dimY, const unsigned int dimZ, const obfloat cellSize);

  ~TsdSpacePartition();

  static int getInitializedPartitionSize();

  static int getDistancesPushed();

  void reset();

  obfloat& operator () (unsigned int z, unsigned int y, unsigned int x);

  void getRGB(unsigned int z, unsigned int y, unsigned int x, unsigned char rgb[3]);

  void init();

  bool isInitialized();

  bool isEmpty();

  obfloat getInitWeight();

  void setInitWeight(obfloat weight);

  unsigned int getX();

  unsigned int getY();

  unsigned int getZ();

  static Matrix* getCellCoordsHom();

  void getCellCoordsOffset(obfloat offset[3]);

  static Matrix* getPartitionCoords();

  unsigned int getWidth();

  unsigned int getHeight();

  unsigned int getDepth();

  unsigned int getSize();

  void addTsd(const unsigned int x, const unsigned int y, const unsigned int z, const obfloat sd, const obfloat maxTruncation, const unsigned char rgb[3]);

  virtual void increaseEmptiness();

  obfloat interpolateTrilinear(int x, int y, int z, obfloat dx, obfloat dy, obfloat dz);

  void serialize(ofstream* f);

  void load(ifstream* f);

private:

  TsdVoxel*** _space;

  obfloat _cellSize;

  obfloat _cellCoordsOffset[3];

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _cellsZ;

  unsigned int _x;

  unsigned int _y;

  unsigned int _z;

  obfloat _initWeight;
};

}

#endif
