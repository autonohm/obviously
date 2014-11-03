/*
 * Obstacle.cpp
 *
 *  Created on: 03.11.2014
 *      Author: mayst
 */

#include "Obstacle.h"

namespace obvious
{

Obstacle::Obstacle(std::vector<double> xcoords, std::vector<double> ycoords)
{
  _xcoords = xcoords;
  _ycoords = ycoords;
}

void Obstacle::getCoords(std::vector<double> &xcoords, std::vector<double> &ycoords)
{
  xcoords = _xcoords;
  ycoords = _ycoords;
}

} /* namespace obvious */
