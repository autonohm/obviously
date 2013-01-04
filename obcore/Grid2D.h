/**
* @file   2DGrid.h
* @author Christian Pfitzner
* @date   11.12.2012
*
*
*/

#ifndef GRID2D_H_
#define GRID2D_H_

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

/**
 * @class Grid2D
 */
class Grid2D
{
public:
  /**
   * Default constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  Grid2D(double resolution = 0.05, double length = 2.005, double width = 2.005);
  /**
   * Destructor
   */
  ~Grid2D(void);
  /**
   * Function to fit cloud in grid
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @param[in]     data    data to save to grid (height, gradient, ..)
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL cloud2Grid(double* cloud, unsigned int size, double* data);
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
   * @param[in]     img    allocated unsigned char array
   */
  void getImageOfGrid(unsigned char* img);
protected:
  /**
   * Function to estimate index for x in grid
   * @param[in]     xValue  x value of point in cloud
   * @return        x index
   */
  int getIndexX(double xValue);
  /**
   * Function to estimate index for x in grid
   * @param[in]     yValue  y value of point in cloud
   * @return        y index
   */
  int getIndexY(double yValue);

  //!< @param   INIT_DOUBLE       0.0
  static const double         INIT_DOUBLE    = 0.0;
  //!< @param   FREE_COLOR        0
  static const unsigned char FREE_COLOR     = 0;
  //!< @param   SET_COLOR        0
  static const unsigned char SET_COLOR      = 255;

  double              _resolution;          //!< resolution of grid
  double              _width;               //!< width of grid in meters
  double              _length;              //!< length of grid in meters
  unsigned int       _rows;                 //!< number of rows
  unsigned int       _cols;                 //!< number of columns
  unsigned int       _nrChannels;           //!< number of channels in grid
  bool                _pointsEstimated;      //!< true if function getPointsInGrid was called
  MatD* _grid;                               //!< pointer on grid
};

} // namespace

#endif /* GRID2D_H_ */
