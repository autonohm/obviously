/*
 * AStar.h
 *
 *  Created on: 03.11.2014
 *      Author: mayst
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

  static std::vector<unsigned int> pathFind(AStarMap* map, const unsigned int & xStart, const unsigned int & yStart, const unsigned int & xFinish, const unsigned int & yFinish);

  static std::vector<unsigned int> pathFind(AStarMap* map, const double & xStart, const double & yStart, const double & xFinish, const double & yFinish);

private:

};

} /* namespace obvious */
#endif /* ASTAR_H_ */
