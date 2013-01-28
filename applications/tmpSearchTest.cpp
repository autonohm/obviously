/**
* @file tmpSearchTest.cpp
* @autor christian
* @date  27.01.2013
*
*
*/




#include "obcore/grid/ObstacleGrid.h"
#include "iostream"

using namespace obvious;

int main (void)
{
  ObstacleGrid* _G = new ObstacleGrid(0.1, 1.0, 1.0);

  unsigned int  size = 3;
  double       _coord[3] = {1.0, 0.5, 0.5};

  _G->cloud2Grid(_coord, size);
  std::cout << _G->getNearestObstacle() << std::endl;

  return(0);
}
