/**
* @file Obvious2DMap.h
* @autor christian
* @date  12.01.2013
*
*
*/

#ifndef OBVIOUS2DMAP_H_
#define OBVIOUS2DMAP_H_

#include "obgraphic/Obvious2D.h"

/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class Obvious2DMap
 * @brief Derived class from Obvious2D viewer to view generated maps
 * @see Obvious2D
 */
class Obvious2DMap : public Obvious2D
{
public:
  /**
   * Standard constructor
   * @param[in]     width       width of viewer
   * @param[in]     height      height of viewer
   * @param[in]     title       title of viewer
   * @param[in]     lengthMap   length of map in meters
   * @param[in]     widthMap    width of map in meters
   */
  Obvious2DMap(unsigned int width, unsigned int height, const char* title, double lengthMap, double widthMap)
    : Obvious2D(width, height, title), _lengthMap(lengthMap), _heightMap(lengthMap),
      _gridActive(false), _circleActive(false), _anglesActive(false) {  }
  /**
   * Function to activate grid
   * @param[in] active    (default=true)
   */
  void showGrid(bool active = true)   { _gridActive = active; }
  /**
   * Function to activate circles in map
   * @param[in] active    (default=true)
   */
  void showCircle(bool active = true) { _circleActive = active; }
  /**
   * Function to activate angles in map
   * @param[in] active    (default=true)
   */
  void showAngles(bool active = true) { _anglesActive = active; }
  /**
   * Function to update drawing
   * @param     image     image to view
   * @param     width     width of image
   * @param     height    height of image
   * @param     channels  number of channels in image (1 or 3 allowed)
   */
  void draw(unsigned char* image, unsigned int width, unsigned int height, unsigned int channels);
private:
  /**
   * Function to draw a grid in the map
   */
  void drawGrid(void) const;
  /**
   * Function to draw circles in the map
   */
  void drawCircle(void) const;
  /**
   * Function to draw angles in map
   */
  void drawAngles(const float angleStep=30.0) const;
  /**
   * Function to represent center of map with a triangle
   */
  void drawCenter(void) const;

  bool   _gridActive;     //!< true if grid active
  bool   _circleActive;   //!< true if circle map active
  bool   _anglesActive;   //!< true if angles in map active

  double _lengthMap;      //!< length of map in meters
  double _heightMap;      //!< height of map in meters
};

} // namespace



#endif /* OBVIOUS2DMAP_H_ */
