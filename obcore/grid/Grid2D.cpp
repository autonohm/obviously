/**
* @file Grid2D.cpp
* @autor christian
* @date  17.12.2012
*
*
*/

#include "obcore/grid/Grid2D.h"
#include <math.h>
#include "obcore/Point.h"
#include "obcore/Point3D.h"

using namespace obvious;

/*
 * default constructor with initialization
 */
Grid2D::Grid2D(const double resolution, const double length,
                 const double width, const unsigned int channels)
{
  _width           = width;
  _length          = length;
  _resolution      = resolution;
  _rows            = floor((_width  / _resolution) + _resolution);
  _cols            = floor((_length / _resolution) + _resolution);
  _grid            = new MatD(_rows, _cols, channels);
  _pointsEstimated = false;
  _img             = new unsigned char[_cols*_rows];
}

/*
 * default destructor
 */
Grid2D::~Grid2D()
{
  delete _grid;
  delete [] _img;
}

SUCCESFUL Grid2D::cloud2Grid(const double* cloud, unsigned int size)
{
  initGrid();
  for(unsigned int i=0; i<size ; i+=3)
  {
    // gets indices of grid for point
    unsigned int x = getIndexX(cloud[i]);
    unsigned int y = getIndexY(cloud[i+1]);//getIndexY(cloud[i+Y]);

    // check if estimated index is valid
    if (x<_cols && y<_rows)
      _grid->at(x,y) = 1.0;
  }
  _pointsEstimated = false;

  return(ALRIGHT);
}


unsigned int Grid2D::getCols() const
{
  return _cols;
}

unsigned int Grid2D::getRows() const
{
  return _rows;
}

double Grid2D::getResolution() const
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
  return points;
}

unsigned char* Grid2D::getImageOfGrid(void)
{
  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(_grid->at(x,y,0) != INIT_DOUBLE)
        _img[x*_rows + y] = SET_COLOR;
      else
        _img[x*_rows + y] = FREE_COLOR;
    }
  }
  return(_img);
}

MatD& Grid2D::getMat(void)
{
  return(*_grid);
}

//~~~~~~~~~~~~ Private ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Grid2D::initGrid(void)
{
  // init channel
  for(unsigned int i=0 ; i<_grid->channels() ; i++) {
    // init columns
    for(unsigned int x=0 ; x<_cols ; x++) {
      //init rows
      for(unsigned int y=0 ; y<_rows ; y++)
        _grid->at(x,y,i) = INIT_DOUBLE;
    }
  }
}

void Grid2D::initChannel(unsigned int channel)
{

}

int Grid2D::getIndexX(const double& xValue) const
{
  if (xValue > 0) {
    double tmp = xValue + _resolution;
    return(-floor(tmp  / _resolution) + floor((double)_rows/2));
  }
  else {
    double tmp = xValue - _resolution;
    return(-ceil(tmp   / _resolution) -1 + floor((double)_rows/2));
  }
}

int Grid2D::getIndexY(const double& yValue) const
{
  if (yValue > 0) {
    double tmp = yValue + _resolution;
    return(-floor(tmp  / _resolution) + floor((double)_cols/2));
  }
  else {
    double tmp = yValue - _resolution;
    return(-ceil(tmp   / _resolution) -1 + floor((double)_cols/2));
  }
}





