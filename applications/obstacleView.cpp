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
#include "obgraphic/Obvious2DMap.h"
#include "obcore/grid/ObstacleGrid.h"
#include "obcore/base/Image.h"

using namespace std;
using namespace obvious;


int main(int argc, char* argv[])
{
  Xtion         *_xtion    = new Xtion(argv[1]);
  ObstacleGrid  *_G        = new ObstacleGrid(/*0.015625*/0.02, 8.0, 8.0);
  Obvious2DMap  *_viewer   = new Obvious2DMap(800, 800, "Obstacle streaming", 8.0, 8.0);
  Image         *_img      = new Image(_G->getCols(), _G->getRows(), Image::GREY);
  unsigned int size      = _xtion->getRows()*_xtion->getRows();

//  _viewer->showGrid();
  _viewer->showCircle();
  _viewer->showAngles();
  while(1)
  {
    if(_xtion->grab())
    {
      _G->height2Grid(_xtion->getCoords(), size*3);
      _img->setImg(_G->getImageOfGrid());
    }
    _viewer->draw(_img->getImg(), _img->getWidth(), _img->getHeight(), _img->getType());
  }

  delete _xtion;
  delete _G;
  delete _img;
  delete _viewer;
  return 0;

}








