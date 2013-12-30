#ifndef TSDGRIDPARTITION_H
#define TSDGRIDPARTITION_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGridComponent.h"

namespace obvious
{

/**
 * Cell structure for TsdGrid
 * @param tsd value of the truncated signed distance function
 * @param weight used to calculate contribution of pushed samples
 */
struct TsdCell
{
  double tsd;
  double weight;
};

/**
 * @class TsdGridPartition
 * @brief Acceleration structure for TsdGrid approach
 * @author Stefan May
 */
class TsdGridPartition : public TsdGridComponent
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

  double& operator () (unsigned int y, unsigned int x);

  void init();

  bool isInitialized();

  bool isEmpty();

  unsigned int getX();

  unsigned int getY();

  Matrix* getCellCoordsHom();

  Matrix* getPartitionCoords();

  unsigned int getWidth();

  unsigned int getHeight();

  unsigned int getSize();

  void addTsd(const unsigned int x, const unsigned int y, const double sdf, const double maxTruncation);

  virtual void increaseEmptiness();

  double interpolateBilinear(int x, int y, double dx, double dy);

private:

  TsdCell** _grid;

  double _cellSize;

  Matrix* _cellCoordsHom;

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _x;

  unsigned int _y;

  double _initWeight;
};

}

#endif
