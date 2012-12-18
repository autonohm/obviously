/**
* @file   Cloud2Grid.cpp
* @author Christian Pfitzner
* @date   13.12.2012
*
*
*/

#include "obgraphic/Obvious3D.h"
#include "obgraphic/Obvious2D.h"

#include "obcore/Grid2D.h"
//#include "obcore/filter/Filter.h"
//#include "obcore/filter/CartesianFilter.h"

using namespace obvious;
using namespace std;

int main(void)
{
  unsigned int size  = 5;
  VtkCloud* cloud     = new VtkCloud;
  Obvious2D viewer(500, 500, "UVC streaming example");
  double* coords    = new double[size*3];
  double* coordsOut = new double[size*3];

  Grid2D _G(0.03125, 2.0, 2.0);
  unsigned width = _G.getRows();
  unsigned height= _G.getCols();
  std::cout << "Cols: " << _G.getCols() << std::endl;
  std::cout << "Rows: " << _G.getRows() << std::endl;

  cloud = VtkCloud::createRandom(size, 0.5);
  cloud->copyCoords(coords);

//  CartesianFilter* filterCartesian = new CartesianFilter;
//  filterCartesian->setAxis(IFilter::x);   // set axis to filter
//  filterCartesian->setThreshold(1.0);             // specifiy threshold
//  filterCartesian->setInput(coords, size);        // set input and size of data
//  filterCartesian->setOutput(coordsOut);          // get output address
//  filterCartesian->setFilterDirection(FILTER_BIGGER);
//  filterCartesian->applyFilter();                 // start filtering
//  size = filterCartesian->getValidSize();

  double* ones = new double[size];
  for(unsigned int i=0 ; i<size ; i++)
    ones[i] = 1;

  _G.cloud2Grid(coords, size, ones);
  std::cout << "Numbers of obstacles: " << _G.getPointsInGrid() << std::endl; ;
  unsigned char* img = new unsigned char[width*height];

  _G.getImageOfGrid(img);

  while(1)
  {
    viewer.draw(img, width, height, 1);
  }
  return 0;
}

