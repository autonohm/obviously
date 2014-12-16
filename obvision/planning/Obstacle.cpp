/*
 * Obstacle.cpp
 *
 *  Created on: 03.11.2014
 *      Author: mayst
 */

#include "Obstacle.h"
#include <iostream>

using namespace std;

namespace obvious
{

static unsigned int _obstacleId = 0;

Obstacle::Obstacle(std::vector<obfloat> xcoords, std::vector<obfloat> ycoords)
{
  if(xcoords.size()!=ycoords.size())
  {
    cout << __PRETTY_FUNCTION__ << " -- ERROR: coordinate vectors of different size." << endl;
    return;
  }

  _bounds.xmin =  1e12;
  _bounds.xmax = -1e12;
  _bounds.ymin =  1e12;
  _bounds.ymax = -1e12;

  vector<obfloat>::iterator itx=xcoords.begin();
  for(vector<obfloat>::iterator ity=ycoords.begin(); ity!=ycoords.end(); ++ity, ++itx)
  {
    if(*itx<_bounds.xmin) _bounds.xmin = *itx;
    if(*itx>_bounds.xmax) _bounds.xmax = *itx;
    if(*ity<_bounds.ymin) _bounds.ymin = *ity;
    if(*ity>_bounds.ymax) _bounds.ymax = *ity;
  }

  _id = _obstacleId;
  _obstacleId++;
}

Obstacle::Obstacle(ObstacleBounds bounds)
{
  _id = _obstacleId;
  _obstacleId++;
  _bounds.xmin           = bounds.xmin;
  _bounds.xmax           = bounds.xmax;
  _bounds.ymin           = bounds.ymin;
  _bounds.ymax           = bounds.ymax;
  cout << __PRETTY_FUNCTION__ << "obstacle created" << endl;

}

Obstacle::Obstacle(Obstacle* o)
{
  _id                    = o->getID();
  ObstacleBounds* bounds = o->getBounds();
  _bounds.xmin           = bounds->xmin;
  _bounds.xmax           = bounds->xmax;
  _bounds.ymin           = bounds->ymin;
  _bounds.ymax           = bounds->ymax;
}

unsigned int Obstacle::getID()
{
  return _id;
}

ObstacleBounds* Obstacle::getBounds()
{
  return &_bounds;
}

bool Obstacle::intersects(Obstacle* o)
{
  ObstacleBounds* bounds = o->getBounds();
  return (_bounds.xmin < bounds->xmax && _bounds.xmax > bounds->xmin && _bounds.ymin < bounds->ymax && _bounds.ymax > bounds->xmin);
}

void Obstacle::inflate(obfloat radius)
{
  _bounds.xmin -= radius;
  _bounds.xmax += radius;
  _bounds.ymin -= radius;
  _bounds.ymax += radius;
}

void Obstacle::merge(std::vector<obfloat> xcoords, std::vector<obfloat> ycoords)
{
  vector<obfloat>::iterator itx=xcoords.begin();
  for(vector<obfloat>::iterator ity=ycoords.begin(); ity!=ycoords.end(); ++ity, ++itx)
  {
    if(*itx<_bounds.xmin) _bounds.xmin = *itx;
    if(*itx>_bounds.xmax) _bounds.xmax = *itx;
    if(*ity<_bounds.ymin) _bounds.ymin = *ity;
    if(*ity>_bounds.ymax) _bounds.ymax = *ity;
  }
}

} /* namespace obvious */
