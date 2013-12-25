#ifndef TSDGRIDPARTITION_H
#define TSDGRIDPARTITION_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/SensorPolar2D.h"

namespace obvious
{

/**
 * Cell structure for TsdGrid
 * @param tsd value of the truncated signed distance function
 * @param weight used to calculate mean of all points in a voxel
 */
struct TsdCell
{
  double tsd;
  double weight;
};

/**
 * @class TsdGridPartition
 * @brief Acceleration structur for TsdGrid approach
 * @author Stefan May
 */
class TsdGridPartition
{
  friend class TsdGrid;
public:

  /**
   * Standard constructor
   * Allocates and initializes space and matrices
   * @param[in] x start index in x-dimension
   * @param[in] y start index in y-dimension
   * @param[in] dimY Number of cells in y-dimension
   * @param[in] dimX Number of cells in x-dimension
   * @param[in] dimY Number of cells in y-dimension
   * @param[in] cellSize Size of cell in meters
   */
  TsdGridPartition(const unsigned int x, const unsigned int y, const unsigned int dimX, const unsigned int dimY, const double cellSize);

  ~TsdGridPartition();

  void init();

  bool isInitialized();

  double& operator () (unsigned int y, unsigned int x);

  bool isVisible(Sensor* sensor);

  unsigned int getX();

  unsigned int getY();

  double* getCentroid();

  Matrix* getCellCoordsHom();

  Matrix* getEdgeCoordsHom();

  Matrix* getPartitionCoords();

  unsigned int getWidth();

  unsigned int getHeight();

  unsigned int getSize();

  void addTsd(const unsigned int x, const unsigned int y, const double sdf, const double maxTruncation);

  void increaseEmptiness();

  double interpolateBilinear(int x, int y, double dx, double dy);

private:

  TsdCell** _grid;

  Matrix* _cellCoordsHom;

  Matrix* _edgeCoordsHom;

  double _cellSize;

  double _initWeight;

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _x;

  unsigned int _y;

  double _centroid[2];
};

}

#endif
