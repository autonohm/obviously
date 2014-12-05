/*
 * Obstacle.h
 *
 *  Created on: 03.11.2014
 *      Author: Stefan May
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <vector>
#include "obcore/base/types.h"

namespace obvious
{

struct ObstacleBounds
{
  obfloat xmin;
  obfloat xmax;
  obfloat ymin;
  obfloat ymax;
};

class Obstacle
{
public:

  /**
   * Constructor
   * @param xcoords x-coordinates of obstacle-related points
   * @param ycoords y-coordinates of obstacle-related points
   */
  Obstacle(std::vector<obfloat> xcoords, std::vector<obfloat> ycoords);

  /**
   * Constructor
   * @param bounds bounding box of obstacle
   */
  Obstacle(ObstacleBounds bounds);

  /**
   * Copy constructor
   * @param o obstacle to be copied
   */
  Obstacle(Obstacle* o);

  /**
   * Get unique obstacle ID
   * @return ID
   */
  unsigned int getID();

  /**
   * Get bounds of obstacle
   * @return bounds
   */
  ObstacleBounds* getBounds();

  /**
   * Inflate obstacle
   * @param radius used for obstacle inflation
   */
  void inflate(obfloat radius);

  /**
   * Check if two obstacles intersect with each other
   * @param o intersection candidate
   * @return intersection flag
   */
  bool intersects(Obstacle* o);

  /**
   * Fuse obstacles, i.e., merging related regions
   * @param xcoords x-coordinates of obstacle
   * @param ycoords y-coordinates of obstacle
   */
  void merge(std::vector<obfloat> xcoords, std::vector<obfloat> ycoords);

private:

  ObstacleBounds _bounds;

  unsigned int _id;
};

} /* namespace obvious */
#endif /* OBSTACLE_H_ */
