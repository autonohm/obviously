/**
* @file obstacleView.cpp
* @autor christian
* @date  11.01.2013
*
*
*/



#include "obdevice/Xtion.h"
#include "obgraphic/Obvious3D.h"
#include "obgraphic/Obvious2D.h"
#include "obcore/grid/ObstacleGrid.h"

using namespace std;
using namespace obvious;


int main(int argc, char* argv[])
{
  Xtion *_xtion           = new Xtion(argv[1]);
  Obvious2D* viewer       = new Obvious2D(1200, 1200, "Obstacle streaming");
  ObstacleGrid* _G        = new ObstacleGrid(0.015625, 6.0, 6.0);
  unsigned int size     = _xtion->getRows()*_xtion->getRows();
  unsigned int width    = _G->getRows();
  unsigned int height   = _G->getCols();
  unsigned char* img    = new unsigned char[width*height];

  while(1)
  {
    if(_xtion->grab())
    {
      _G->height2Grid(_xtion->getCoords(), size*3);
      _G->getImageOfGrid(img);
    }
    viewer->draw(img, width, height, 1);
  }

  delete _xtion;
  delete _G;
  delete viewer;
  return 0;

}








