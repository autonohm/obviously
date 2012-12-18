/**
* @file Grid2D.cpp
* @autor christian
* @date  17.12.2012
*
*
*/

#include "obcore/Grid2D.h"

using namespace obvious;

/*
 * default constructor with initialization
 */
Grid2D::Grid2D(double resolution, double length, double width)
{
  _width           = width;
  _length          = length;
  _resolution      = resolution;
  _rows            = floor((_width  / _resolution) + _resolution);
  _cols            = floor((_length / _resolution) + _resolution);
  _grid            = new MatD(_rows, _cols);
  _nrChannels      = 0;
  _obstaclesInGrid = 0;
  _pointsEstimated = false;

  //init _grid
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      _grid->at(x,y,0) = INIT_DOUBLE;
    }
  }

}

/*
 * default destructor
 */
Grid2D::~Grid2D()
{
  delete _grid;
}

SUCCESFUL Grid2D::cloud2Grid(double* cloud, unsigned int size, double* data)
{
  unsigned int j=0;
  for(unsigned int i=0; i<size ; i++)
  {
    // gets indices of grid for point
    int x = getIndexX(cloud[i+X]);
    int y = getIndexY(cloud[i+Y]);

    // check if points are in frontiers of grid
    if (x<=_cols && y<=_rows)
      _grid->at(x,y,0) = data[j];

    j++;
  }
  _pointsEstimated = false;

  return(ALRIGHT);
}

SUCCESFUL Grid2D::normals2Grid(double* cloud, unsigned int size, double* normals)
{
  return(ALRIGHT);
}

double* Grid2D::getObstacles()
{
  if (!_pointsEstimated)
    this->getPointsInGrid();
  double* obstacles = new double[_obstaclesInGrid];
  return(obstacles);
}

unsigned int Grid2D::getCols()
{
  return _cols;
}

unsigned int Grid2D::getRows()
{
  return _rows;
}

double Grid2D::getResolution()
{
  return _resolution;
}

unsigned int Grid2D::getPointsInGrid(void)
{
  unsigned int points = 0;
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(_grid->at(x,y,0) != INIT_DOUBLE)
        points++;
    }
  }
  _pointsEstimated = true;
  return _obstaclesInGrid = points;
}

void Grid2D::getImageOfGrid(unsigned char* img)
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

//~~~~~~~~~~~~ Private ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int Grid2D::getIndexX(double xValue)
{
  if (xValue > 0)
    return(floor((xValue + _resolution) / _resolution) + floor((double)_cols/2));
  else
    return(ceil((xValue  + _resolution) / _resolution) + ceil((double)_cols/2));
}

int Grid2D::getIndexY(double yValue)
{
  if (yValue > 0)
    return(floor((yValue + _resolution) / _resolution) + floor((double)_rows/2));
  else
    return(ceil((yValue  + _resolution) / _resolution) + ceil((double)_rows/2));
}




