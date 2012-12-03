/**
* @file filter_test.cpp
* @autor christian
* @date  03.12.2012
*
*
*/

#include <iostream>
#include "obcore/filter/Filter.h"
#include "obcore/filter/CartesianFilter.h"
#include "obcore/filter/EuclideanFilterD.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  const unsigned int size = 12;
  CartesianFilter  filterCartesian;
  EuclideanFilterD filterEuclidean;
  double coords[size]    = { 1.2,  1.7,  1.6,
                             2.1,  0.1,  0.9,
                             3.4,  -2.1, 0,
                             -1.2, -3.1, 2.3};
  double coordsOut[size];

  std::cout << "************************************" << std::endl <<
               "* Input"                              << std::endl;
  for (unsigned int i = 0 ; i < size ; )
  {
    std::cout << "Point: " << coords[i]   << " \t"
                           << coords[i+1] << " \t"
                           << coords[i+2] << std::endl;
    i += 3;
  }
  std::cout << std::endl;

  filterCartesian.setAxis(CartesianFilter::x);   // set axis to filter
  filterCartesian.setThreshold(2.0);             // specifiy threshold
  filterCartesian.setInput(coords, size);        // set input and size of data
  filterCartesian.setOutput(coordsOut);          // get output address
  filterCartesian.applyFilter();                 // start filtering
  std::cout << "************************************" << std::endl <<
               "* Cartesian Filter"                   << std::endl <<
               "************************************" << std::endl <<
               "* Threshold \t2.0"                    << std::endl <<
               "* Axis \t\tx"                         << std::endl <<
               "************************************" << std::endl;
  for (unsigned int i = 0 ; i < filterCartesian.getValidSize() ; )
  {
    std::cout << "Point: \t" << coordsOut[i]   << " \t"
                             << coordsOut[i+1] << " \t"
                             << coordsOut[i+2] << std::endl;
    i += 3;
  }
  std::cout << std::endl;

  filterEuclidean.setThreshold(3.2);            // set threshold to filter
  filterEuclidean.setInput(coords, size);       // set input and size of data
  filterEuclidean.setOutput(coordsOut);         // get output address
  filterEuclidean.applyFilter();                // start filtering
  std::cout << "************************************" << std::endl <<
               "* Euclidean Filter"                   << std::endl <<
               "************************************" << std::endl <<
               "* Threshold \t3.2"                    << std::endl <<
               "************************************" << std::endl;
  for (unsigned int i = 0 ; i < filterEuclidean.getValidSize() ; )
  {
    std::cout << "Point: \t" << coordsOut[i]   << " \t"
                             << coordsOut[i+1] << " \t"
                             << coordsOut[i+2] << std::endl;
    i += 3;
  }
  std::cout << std::endl;

}




