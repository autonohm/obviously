/**
* @file   ObstacleGrid.cpp
* @author Christian Pfitzner
* @date   04.01.2013
*
*
*/

#include "obcore/ObstacleGrid.h"

#include <math.h>
#include "obcore/Point.h"
#include "obcore/Point3D.h"

using namespace obvious;

ObstacleGrid::ObstacleGrid(double resolution, double length, double width)
: Grid2D(resolution, length, width),
  _obstaclesInGrid(0)
{

}

ObstacleGrid::~ObstacleGrid()
{

}

SUCCESFUL ObstacleGrid::normals2Grid(double* cloud, unsigned int size, double* normals)
{
  return(ALRIGHT);
}

SUCCESFUL ObstacleGrid::height2Grid(double* cloud, unsigned int size)
{
  unsigned int j=0;
  for(unsigned int i=0; i<size ; i++)
  {
    // gets indices of grid for point
    int x = getIndexX(cloud[i+X]);
    int y = getIndexY(cloud[i+Y]);

    // check if points are in frontiers of grid
    if (x<=_cols && y<=_rows) {
      // check if new height value is bigger than saved
      if (_grid->at(x,y,HEIGHT) < cloud[i+Z])
        _grid->at(x,y,HEIGHT) = cloud[i+Z];
    }

    j++;
  }
  _pointsEstimated = false;
  return(ALRIGHT);
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
  //init image
  for(unsigned int i=0 ; i<_cols*_rows ; i++)
  {
    img[i] = OBSTACLE_COLOR;
  }
  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(_grid->at(x,y,0) != INIT_DOUBLE)
        img[x*_rows + y] = OBSTACLE_COLOR;
      else
        img[x*_rows + y] = FREE_COLOR;
    }
  }
}





