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
  obfloat tsd;
  obfloat weight;
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
  TsdGridPartition(const unsigned int x, const unsigned int y, const unsigned int dimX, const unsigned int dimY, const obfloat cellSize);

  /**
   * Destructor
   */
  ~TsdGridPartition();

  /**
   * TSD access operator
   * @param y y-index
   * @param x x-index
   * @return TSD
   */
  obfloat& operator () (unsigned int y, unsigned int x);

  /**
   * Initialize partitions
   */
  void init();

  /**
   * Initialization indication
   * @return initialization flag
   */
  bool isInitialized();

  /**
   * Emptiness indication
   * @return emptiness flag
   */
  bool isEmpty();

  /**
   * Get x-index
   * @return x-index
   */
  unsigned int getX();

  /**
   * Get y-index
   * @return y-index
   */
  unsigned int getY();

  /**
   * Get homogeneous cell coordinates
   * @return cell coordinate matrix of partition
   */
  Matrix* getCellCoordsHom();

  /**
   * Get partition coordinates
   * @return partition coordinates matrix
   */
  Matrix* getPartitionCoords();

  /**
   * Get width, i.e., number of cells in x-dimension
   * @return width
   */
  unsigned int getWidth();

  /**
   * Get height, i.e., number of cells in y-dimension
   * @return height
   */
  unsigned int getHeight();

  /**
   * Get number of cells in partition, i.e., width x height
   * @return number of cells
   */
  unsigned int getSize();

  /**
   * Add TSD value at certain cell
   * @param x cell x-index
   * @param y cell y-index
   * @param sdf SDF
   * @param maxTruncation maximum truncation radius
   */
  void addTsd(const unsigned int x, const unsigned int y, const obfloat sdf, const obfloat maxTruncation);

  /**
   * Increase emptiness of whole partition, i.e., every measurement ray passes through partition
   */
  virtual void increaseEmptiness();

  /**
   * Interpolate bilinear within cell
   * @param x x-index
   * @param y y-index
   * @param dx x-coordinate within cell
   * @param dy y-coordinate within cell
   * @return interpolated TSD
   */
  obfloat interpolateBilinear(int x, int y, obfloat dx, obfloat dy);

private:

  TsdCell** _grid;

  obfloat _cellSize;

  Matrix* _cellCoordsHom;

  unsigned int _cellsX;

  unsigned int _cellsY;

  unsigned int _x;

  unsigned int _y;

  obfloat _initWeight;

  bool _initialized;
};

}

#endif
