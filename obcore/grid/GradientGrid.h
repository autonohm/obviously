/**
* @file   GradientGrid.h
* @author Christian Pfitzner
* @date   06.01.2013
*
*
*/

#ifndef GRADIENTGRID_H_
#define GRADIENTGRID_H_

#include "obcore/grid/Grid2D.h"
#include "obcore/math/MatD.h"

/**
 * @namespace obvious
 */
namespace obvious {
/**
 * @class GradientGrid
 * @brief Grid to estimate gradiend in x and y direction of a given height map
 */
class GradientGrid : public Grid2D
{
public:
  /**
   * Standard constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  GradientGrid(const double& resolution, const double& length, const double& width)
    : Grid2D(resolution, length, width, 2) {   }
  /**
   * Default destructor
   */
  virtual ~GradientGrid(void) {  }
  /**
   * Function to get height map
   * @param[in]     heightGrid   height grid of a cloud
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL gradient2Grid(const MatD& heigthMat);
  /**
   * Function to return unsigned char image for visualization
   * @param         img    allocated unsigned char array
   */
  unsigned char* getImageOfGrid(void);
private:
  /**
   * Function to return map as image with gradients
   * @param         img
   */
  unsigned char* getGradientMap(void);
  enum EnumChannel{GRADIENT_X, GRADIENT_Y};

};

} // namespace

#endif /* GRADIENTGRID_H_ */
