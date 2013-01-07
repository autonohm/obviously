/**
* @file   ObstacleGrid.cpp
* @author Christian Pfitzner
* @date   04.01.2013
*
*
*/

#include "obcore/grid/ObstacleGrid.h"

#include <math.h>
#include "obcore/Point.h"
#include "obcore/Point3D.h"

using namespace obvious;

ObstacleGrid::ObstacleGrid(double resolution, double length, double width)
: Grid2D(resolution, length, width, 4),
  _obstaclesInGrid(0)
{
  _heightTH = 1.0;
  _hGrid = new HeightGrid(resolution, length, width);
  _gGrid = new GradientGrid(resolution, length, width);
}

ObstacleGrid::~ObstacleGrid()
{
  delete _hGrid;
  delete _gGrid;
}

//SUCCESFUL ObstacleGrid::normals2Grid(double* cloud, unsigned int size, double* normals)
//{
//  for(unsigned int i=0; i<size ; i+=3)
//  {
//    // gets indices of grid for point
//    unsigned int x = getIndexX(cloud[i+X]);
//    unsigned int y = getIndexY(cloud[i+Y]);
//
//    // check if points are in frontiers of grid
//    if (x<_cols && y<_rows) {
//      // check if new height value is bigger than saved
//      if (_grid->at(x,y,GRADIENT_X) < fabs(normals[i+GRADIENT_X]))
//        _grid->at(x,y,GRADIENT_X) = normals[i+GRADIENT_X];
//
//      if (_grid->at(x,y,GRADIENT_Y) < fabs(normals[i+GRADIENT_Y]))
//        _grid->at(x,y,GRADIENT_Y) = normals[i+GRADIENT_Y];
//    }
//  }
//  _pointsEstimated = false;
//  return(ALRIGHT);
//}

SUCCESFUL ObstacleGrid::height2Grid(double* cloud, unsigned int size)
{
  if(!_hGrid->height2Grid(cloud, size))
    return(ERROR);
  return(_gGrid->gradient2Grid(dynamic_cast<MatD&>(_hGrid->getMat())));
}

double* ObstacleGrid::getObstacles() const
{
//  if (!_pointsEstimated)
//    getPointsInGrid();
  double* obstacles = new double[_obstaclesInGrid];
  return(obstacles);
}

void ObstacleGrid::getImageOfGrid(unsigned char* img)
{
//  _hGrid->getImageOfGrid(img);
  _gGrid->getImageOfGrid(img);
}





