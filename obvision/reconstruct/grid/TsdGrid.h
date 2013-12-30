#ifndef TSDGRID_H
#define TSDGRID_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "TsdGridPartition.h"

namespace obvious
{

enum EnumTsdGridLayout { LAYOUT_1x1=0,
                         LAYOUT_2x2=1,
                         LAYOUT_4x4=2,
                         LAYOUT_8x8=3,
                         LAYOUT_16x16=4,
                         LAYOUT_32x32=5,
                         LAYOUT_64x64=6,
                         LAYOUT_128x128=7,
                         LAYOUT_256x256=8,
                         LAYOUT_512x512=9,
                         LAYOUT_1024x1024=10,
                         LAYOUT_2048x2048=11,
                         LAYOUT_4096x4096=12,
                         LAYOUT_9192x9192=13,
                         LAYOUT_18384x18384=14,
                         LAYOUT_36768x36768=15};

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
   * @param[in] cellSize Size of cell in meters
   * @param[in] layoutPartition Partition layout, i.e., cells in partition
   * @param[in] layoutGrid Grid layout, i.e., partitions in grid
   */
  TsdGrid(const double cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid);

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
   * Get centroid of grid
   * @param[out] centroid centroid coordinates
   */
  void getCentroid(double centroid[2]);

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
  void pushTree(SensorPolar2D* sensor);

  void pushRecursion(SensorPolar2D* sensor, double pos[2], TsdGridComponent* comp, vector<TsdGridPartition*> &partitionsToCheck);

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

  TsdGridComponent* _tree;

  int _cellsX;

  int _cellsY;

  int _sizeOfGrid;

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
