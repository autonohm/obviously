/*
 * Obstacle.h
 *
 *  Created on: 03.11.2014
 *      Author: mayst
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <vector>

namespace obvious
{

class Obstacle
{
public:

  Obstacle(std::vector<double> xcoords, std::vector<double> ycoords);

  void getCoords(std::vector<double> &xcoords, std::vector<double> &ycoords);

private:
  std::vector<double> _xcoords;
  std::vector<double> _ycoords;
};

} /* namespace obvious */
#endif /* OBSTACLE_H_ */
