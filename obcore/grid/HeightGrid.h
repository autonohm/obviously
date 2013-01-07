/**
* @file   HeightGrid.h
* @author Christian Pfitzner
* @date   06.01.2013
*
*
*/

#ifndef HEIGHTGRID_H_
#define HEIGHTGRID_H_

#include "obcore/grid/Grid2D.h"
/**
 * @namespace obvious
 */
namespace obvious {
/**
 * @class HeightGrid
 * @brief Class for height maps
 *
 * This class builds height maps out of a given point cloud and is derived
 * from Grid2D
 * @see Grid2D
 */
class HeightGrid : public Grid2D
{
public:
  /**
   * Standard constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  HeightGrid(const double& resolution, const double& length, const double& width)
    : Grid2D(resolution, length, width, 1) {   }
  /**
   * Default destructor
   */
  virtual ~HeightGrid(void) {  }
  /**
   * Function to get height map
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL height2Grid(double* cloud, unsigned int size);
  /**
   * Function to return unsigned char image for visualization
   * @param[in]     img    allocated unsigned char array
   */
  void getImageOfGrid(unsigned char* img);
private:
  /**
   * Function to return map as image with heights
   * @param         img
   */
  void getHeightMap(unsigned char* img);

};

}



#endif /* HEIGHTGRID_H_ */
