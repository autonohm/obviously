/**
* @file   ObstacleGrid.h
* @author Christian Pfitzner
* @date   04.01.2013
*
*
*/

#ifndef OBSTACLEGRID_H_
#define OBSTACLEGRID_H_

#include "obcore/Grid2D.h"
/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class ObstacleGrid
 * @brief Grid to detect obstacles
 *
 * The reduction of point clouds to a grid makes following algorithms easier
 * to detect obstacles. This is why the class is set up. The class is derived
 * from Grid2D
 * @see Grid2D
 */
class ObstacleGrid : public Grid2D
{
public:
  /**
   * Default constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  ObstacleGrid(double resolution = 0.05, double length = 2.005, double width = 2.005);
  /**
   * Default destructor
   */
  virtual ~ObstacleGrid(void);
  /**
   * Function to get height map
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL height2Grid(double* cloud, unsigned int size);
  /**
   * Function to fit cloud in grid
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @param[in]     normals normals to save to grid (height, gradient, ..)
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL normals2Grid(double* cloud, unsigned int size, double* normals);
  /**
   * Function to return obstacles (x0,y0, x1,y1, ..)
   * @return    obstacle array with 2d coords
   */
  double* getObstacles(void) const;

  virtual void getImageOfGrid(unsigned char* img);
private:
  /**
   * @enum CHANNEL
   */
  enum CHANNEL
  {
    HEIGHT,    //!< HEIGHT
    GRADIENT_X,//!< GRADIENT_X
    GRADIENT_Y //!< GRADIENT_Y
  };

  //!< @param   OBSTACLE_COLOR    255
  static const unsigned char OBSTACLE_COLOR = 255;

  unsigned int       _obstaclesInGrid;      //!< number of obstacles in grid


};
}




#endif /* OBSTACLEGRID_H_ */
