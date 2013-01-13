/**
* @file HeightGrid.cpp
* @autor christian
* @date  06.01.2013
*
*
*/

#include "obcore/grid/HeightGrid.h"
#include "obcore/Point3D.h"

using namespace obvious;

SUCCESFUL HeightGrid::height2Grid(double* cloud, unsigned int size)
{
  initGrid();
  for(unsigned int i=0; i<size ; i+=3)
  {
    // gets indices of grid for point
    unsigned int x = getIndexX(cloud[i+Z]);
    unsigned int y = getIndexY(cloud[i+X]);

    // check if points are in frontiers of grid
    if (x<_cols && y<_rows) {
      // check if new height value is bigger than saved
      if (_grid->at(x,y,0) < cloud[i+Y])
        _grid->at(x,y,0) = cloud[i+Y];
    }
  }
  _pointsEstimated = false;
  return(ALRIGHT);
}

void HeightGrid::getImageOfGrid(unsigned char *img)
{
  getHeightMap(img);
}

//~~~~~~~~~~~~ Private ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void HeightGrid::getHeightMap(unsigned char *img)
{
  unsigned char minValue = 255;
  unsigned char maxValue = 0;

  // estimate min max value für maximum color range
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(_grid->at(x,y) > maxValue)
        maxValue = _grid->at(x,y);
      if(_grid->at(x,y) < minValue)
        minValue = _grid->at(x,y);
    }
  }

  double range = maxValue - minValue;

  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++)
    for(unsigned int y=0 ; y<_rows ; y++)
      _img[x*_rows + y] = (unsigned char)(_grid->at(x,y) - minValue)/range*255;
}
