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
   * @param idxStart indices of starting point
   * @param idxTarget indices of target
   * @return path
   */
  static std::vector<unsigned int> pathFind(AStarMap* map, const unsigned int idxStart[2], const unsigned int idxTarget[2]);

  /**
   * Plan path giving start and target coordinates
   * @param map map
   * @param coordStart coordinates of starting point
   * @param coordTarget coordinates of target
   * @return path
   */
  static std::vector<unsigned int> pathFind(AStarMap* map, const AStarCoord coordStart, const AStarCoord coordTarget);

private:

};

} /* namespace obvious */
#endif /* ASTAR_H_ */
