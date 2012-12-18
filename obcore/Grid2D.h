/**
* @file   2DGrid.h
* @author Christian Pfitzner
* @date   11.12.2012
*
*
*/

#ifndef GRID2D_H_
#define GRID2D_H_

#include <iostream>

#include <math.h>
#include "obcore/Point.h"
#include "obcore/Point3D.h"
#include "obcore/math/MatD.h"

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @enum SUCCESFUL
 */
enum SUCCESFUL
{
  ERROR, //!< ERROR
  ALRIGHT//!< ALRIGHT
};

const unsigned char OBSTACLE_COLOR = 255;
const unsigned char FREE_COLOR     = 0;
const double         INIT_DOUBLE    = 0.0;
/**
 * @class Grid2D
 */
class Grid2D
{
public:
  /**
   * Default constructor
   */
  Grid2D(double resolution = 0.05, double length = 2.005, double width = 2.005);
  /**
   * Destructor
   */
  ~Grid2D(void);
  /**
   * Function to fit cloud in grid
   * @param     cloud   cloud with points in double array
   * @param     size    number of points in cloud
   * @param     data    data to save to grid (height, gradient, ..)
   * @return    TRUE if no error occures @see SUCCESFUL
   */
  SUCCESFUL cloud2Grid(double* cloud, unsigned int size, double* data);
  /**
   * Function to fit cloud in grid
   * @param     cloud   cloud with points in double array
   * @param     size    number of points in cloud
   * @param     normals normals to save to grid (height, gradient, ..)
   * @return    TRUE if no error occures @see SUCCESFUL
   */
  SUCCESFUL normals2Grid(double* cloud, unsigned int size, double* normals);
  /**
   * Function to return obstacles (x0,y0, x1,y1, ..)
   * @return    obstacle array with 2d coords
   */
  double* getObstacles(void);
  /**
   * Function to return number of rows
   * @return    rows
   */
  unsigned int getRows(void);
  /**
   * Function to return number columns
   * @return    columns
   */
  unsigned int getCols(void);
  /**
   * Function to return resolution of grid in meter
   * @return    resolution in meter
   */
  double getResolution(void);
  /**
   * Function to return all fitted points/obstacles in grid
   * @return    number of points
   */
  unsigned int getPointsInGrid(void);
  /**
   * Function to return unsigned char image for visualization
   * @param     img    allocated unsigned char array
   */
  void getImageOfGrid(unsigned char* img);
private:
  /**
   * Function to estimate index for x in grid
   * @param     xValue  x value of point in cloud
   * @return    x index
   */
  int getIndexX(double xValue);
  /**
   * Function to estimate index for y in grid
   * @param     yValue  y value of point in cloud
   * @return    y index
   */
  int getIndexY(double yValue);

  double              _resolution;          //!< resolution of grid
  double              _width;               //!< width of grid in meters
  double              _length;              //!< length of grid in meters
  unsigned int       _rows;                 //!< number of rows
  unsigned int       _cols;                 //!< number of columns
  unsigned int       _nrChannels;           //!< number of channels in grid
  unsigned int       _obstaclesInGrid;      //!< number of obstacles in grid
  bool                _pointsEstimated;      //!< true if function getPointsInGrid was called
  MatD* _grid;                               //!< pointer on grid
};

} // namespace

#endif /* GRID2D_H_ */
