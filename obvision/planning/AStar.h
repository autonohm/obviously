/*
 * AStar.h
 *
 *  Created on: 03.11.2014
 *      Author: Jon Martin and Stefan May
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <string>
#include <vector>

#include "obvision/planning/Obstacle.h"
#include "obvision/planning/AStarMap.h"

namespace obvious
{

class AStar
{

public:

  /**
   * Plan path giving start and target indices
   * @param map map
   * @param start pixel coordinates of starting point
   * @param target pixel coordinates of target
   * @param penalty use directional change penalty
   * @return path
   */
  static std::vector<unsigned int> pathFind(AStarMap* map, const Pixel start, const Pixel target, const bool penalty);

  /**
   * Plan path giving start and target coordinates
   * @param map map
   * @param coordStart coordinates of starting point
   * @param coordTarget coordinates of target
   * @param penalty use directional change penalty
   * @return path
   */
  static std::vector<unsigned int> pathFind(AStarMap* map, const Point2D coordStart, const Point2D coordTarget, const bool penalty, const Point2D* const offset = NULL);

  /**
   * Convert row/col indices to cell index
   * @param x column
   * @param y row
   * @param width width of grid, i.e., number of columns
   */
  static inline unsigned int toId(unsigned int x, unsigned int y, unsigned int width)
  {
     return y*width+x;
  }

private:

};

} /* namespace obvious */
#endif /* ASTAR_H_ */
