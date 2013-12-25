#ifndef TSDGRID_H
#define TSDGRID_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/SensorPolar2D.h"
#include "TsdGridPartition.h"

namespace obvious
{

/**
 * @class TsdGrid
 * @brief Grid on the basis of true signed distance functions
 * @author Stefan May, Philipp Koch
 */
class TsdGrid
{
public:

  /**
   * Standard constructor
   * Allocates and initializes space and matrices
   * @param[in] dimX Number of cells in x-dimension
   * @param[in] dimY Number of cells in y-dimension
   * @param[in] cellSize Size of cell in meters
   * @param[in] number of partitioned per dimension, i.e. in x- and y-direction
   */
  TsdGrid(const unsigned int dimX, const unsigned int dimY, const double cellSize, const unsigned int partitionSize);

  /**
   * Destructor
   */
  virtual ~TsdGrid();

  /**
   * Access truncated signed distance at specific cell. This method does not check validity of indices.
   * The specific cell might not be instantiated.
   * @param y y coordinate
   * @param x x coordinate
   * @return truncated signed distance
   */
  double& operator () (unsigned int y, unsigned int x);

  /**
   * Get number of cells in x-dimension
   * @return number of cells
   */
  unsigned int getCellsX();

  /**
   * Get number of cells in y-dimension
   * @return number of cells
   */
  unsigned int getCellsY();

  /**
   * Get size of cell in meters
   * @return size
   */
  double getCellSize();

  /**
   * Get minimum for x-coordinate
   * @return x-coordinate
   */
  double getMinX();

  /**
   * Get maximum for x-coordinate
   * @return x-coordinate
   */
  double getMaxX();

  /**
   * Get minimum for y-coordinate
   * @return y-coordinate
   */
  double getMinY();

  /**
   * Get maximum for y-coordinate
   * @return y-coordinate
   */
  double getMaxY();

  /**
   * Set maximum truncation radius
   * @param[in] val truncation radius
   */
  void setMaxTruncation(const double val);

  /**
   * Get maximum truncation radius
   * @return truncation radius
   */
  double getMaxTruncation();

  /**
   * Push current measurement from sensor
   * @param[in] virtual 2D measurement unit
   */
  void push(SensorPolar2D* sensor);

  /**
   * Create color image from tsdf grid
   * @param[out] color image (3-channel)
   */
  void grid2ColorImage(unsigned char* image, unsigned int width, unsigned int height);

  /**
   * Calculates normal of plain element hit by a ray caster
   * @param[out] coordinates
   * @param[out] normal vector
   */
  bool interpolateNormal(const double coord[2], double normal[2]);

  /**
   * interpolate bilinear
   * @param coordinates pointer to coordinates of intersection
   * @param[out] tsdf interpolated TSD value
   */
  bool interpolateBilinear(double coord[2], double* tsdf);

  /**
   * Convert arbitrary coordinate to grid coordinates
   * @param[in] coord 2D query coordinates
   * @param[out] x x-index
   * @param[out] y y-index
   * @param[out] dx x-coordinate of cell-center in metric space
   * @param[out] dy y-coordinate of cell-center in metric space
   */
  bool coord2Cell(double coord[2], int* p, int* x, int* y, double* dx, double* dy);

private:

  void propagateBorders();

  int _cellsX;

  int _cellsY;

  int _sizeOfGrid;

  int _dimX;

  int _dimY;

  double _cellSize;

  double _invCellSize;

  double _maxTruncation;

  double _minX;

  double _maxX;

  double _minY;

  double _maxY;

  TsdGridPartition*** _partitions;

  int _dimPartition;

  int _partitionsInX;

  int _partitionsInY;
};

}

#endif
