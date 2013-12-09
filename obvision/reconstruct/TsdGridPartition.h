#ifndef TSDGRIDPARTITION_H
#define TSDGRIDPARTITION_H

#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class TsdGridPartition
 * @brief Acceleration structur for TsdGrid approach
 * @author Stefan May
 */
class TsdGridPartition
{
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

  double* getCentroid();

  Matrix* getCellCoordsHom();

  Matrix* getEdgeCoordsHom();

  Matrix* getGridCoords();

  unsigned int getSize();

private:

  Matrix* _gridCoords;

  Matrix* _cellCoordsHom;

  Matrix* _edgeCoordsHom;

  unsigned int _cellsX;

  unsigned int _cellsY;

  double _centroid[2];

};

}

#endif
