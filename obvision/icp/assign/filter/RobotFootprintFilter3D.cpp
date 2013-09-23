/*
 * RobotFootprintFilter3D.cpp
 *
 *  Created on: 20.09.2013
 *      Author: chris
 */


#include "RobotFootprintFilter3D.h"

namespace obvious
{

RobotFootprintFilter3D::RobotFootprintFilter3D(double minRadius, double maxRadius)
  : _minRadius(minRadius), _maxRadius(maxRadius)
{
  _offset = NULL;
  _x_lim  = NULL;
  _y_lim  = NULL;
  _z_lim  = NULL;
};

RobotFootprintFilter3D::RobotFootprintFilter3D(const double* x_lim, const double* y_lim, const double* z_lim)
 : _minRadius(0.0), _maxRadius(NAN)
{
  _offset = NULL;
  _x_lim  = NULL;
  _y_lim  = NULL;
  _z_lim  = NULL;
}

RobotFootprintFilter3D::~RobotFootprintFilter3D()
{

}

void RobotFootprintFilter3D::filter(double** scene, unsigned int size, bool* mask)
{
  for (unsigned int i=0 ; i<size ; i++)
  {
    double p[3] = {scene[i][0], scene[i][1], scene[i][2]};
    if (abs3D<double>(p) < _maxRadius)
      mask[i] = false;
  }
}

}
