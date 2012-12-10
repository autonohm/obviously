/**
* @file   filter_test.cpp
* @author Christian Pfitzner
* @date   03.12.2012
*
*
*/

#include <iostream>
#include "obcore/filter/Filter.h"
#include "obcore/filter/CartesianFilter.h"
#include "obcore/filter/EuclideanFilterD.h"
#include "obcore/filter/BoundingBoxFilter.h"

#include "obcore/Point3D.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  const unsigned int size = 12;
  CartesianFilter*    filterCartesian = new CartesianFilter;
  EuclideanFilterD*   filterEuclidean = new EuclideanFilterD;
  BoundingBoxFilter* _pBBFilter       = new BoundingBoxFilter;

  double coords[size]    = { 1.2,  1.7,  1.6,
                             2.1,  0.1,  0.9,
                             3.4, -2.1,  0.1,
                            -1.2, -3.1,  2.3};
  double coordsOut[size];

  std::cout << "************************************" << std::endl <<
               "* Input"                              << std::endl <<
               "**" << std::endl;
  for (unsigned int i = 0 ; i < size ; )
  {
    std::cout << "Point: \t" << coords[i]   << " \t"
                             << coords[i+1] << " \t"
                             << coords[i+2] << std::endl;
    i += 3;
  }
  std::cout << std::endl;

  // Interface for filters of obcore
  IFilter* pFilter = filterCartesian;

  filterCartesian->setAxis(IFilter::x);   // set axis to filter
  pFilter->setThreshold(2.0);             // specifiy threshold
  pFilter->setInput(coords, size);        // set input and size of data
  pFilter->setOutput(coordsOut);          // get output address
  pFilter->setFilterDirection(FILTER_BIGGER);
  pFilter->applyFilter();                 // start filtering
  std::cout << "************************************" << std::endl <<
               "* Cartesian Filter"                   << std::endl <<
               "**" << std::endl <<
               "* Threshold \t2.0"                    << std::endl <<
               "* Axis \t\tx"                         << std::endl <<
               "**" << std::endl;
  for (unsigned int i = 0 ; i < pFilter->getValidSize() ; i += 3)
  {
    std::cout << "Point: \t" << coordsOut[i]   << " \t"
                             << coordsOut[i+1] << " \t"
                             << coordsOut[i+2] << std::endl;
  }
  std::cout << std::endl;

  filterEuclidean->setThreshold(3.2);            // set threshold to filter
  filterEuclidean->setInput(coords, size);       // set input and size of data
  filterEuclidean->setOutput(coordsOut);         // get output address
  filterEuclidean->applyFilter();                // start filtering
  std::cout << "************************************" << std::endl <<
               "* Euclidean Filter"                   << std::endl <<
               "**" << std::endl <<
               "* Threshold \t3.2"                    << std::endl <<
               "**" << std::endl;
  for (unsigned int i = 0 ; i < filterEuclidean->getValidSize() ; i += 3)
  {
    std::cout << "Point: \t" << coordsOut[i]   << " \t"
                             << coordsOut[i+1] << " \t"
                             << coordsOut[i+2] << std::endl;
  }
  std::cout << std::endl;


  _pBBFilter->setCentroid(Point3D(0.0, 0.0, 0.0));
  _pBBFilter->setDimension(3.0f);
  _pBBFilter->setInput(coords, size);        // set input and size of data
  _pBBFilter->setOutput(coordsOut);         // get output address
  _pBBFilter->applyFilter();

  std::cout << "************************************" << std::endl <<
               "* Bounding Box Filter"                   << std::endl <<
               "**" << std::endl;
  for (unsigned int i = 0 ; i < _pBBFilter->getValidSize() ; i += 3 )
  {
    std::cout << "Point: \t" << coordsOut[i]   << " \t"
                             << coordsOut[i+1] << " \t"
                             << coordsOut[i+2] << std::endl;
  }
  std::cout << std::endl;

  delete filterCartesian;
  delete filterEuclidean;
  delete _pBBFilter;
}






