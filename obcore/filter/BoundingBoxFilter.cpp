/**
* @file   FilterBoundingBox.cpp
* @author Christian Pfitzner
* @date   10.12.2012
*/

#include "obcore/filter/BoundingBoxFilter.h"
using namespace obvious;


BoundingBoxFilter::~BoundingBoxFilter(void)
{
  delete _pFilter;
}

FILRETVAL BoundingBoxFilter::applyFilter(void)
{
  if((!_input)||(!_output))
  {
    LOGMSG(DBG_ERROR, "Pointer to input or output invalid");
    return(FILTER_ERROR);
  }

  // init output array
  for(unsigned int i=0 ; i<_size ; i++)
    _output[i]=0.0;

  double tempCoords1[_size];
  double tempCoords2[_size];

  _pFilter->setCentroid(_centroid);

  ///@todo get algorithm in loop
  // set filter for positiver x direction ++++++++++++++++++++++++++++++++++++++
  _pFilter->setInput(_input, _size);
  _pFilter->setOutput(tempCoords1);
  _pFilter->setThreshold(_dim.x);
  _pFilter->setAxis(x);
  _pFilter->setFilterDirection(FILTER_BIGGER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _size = _pFilter->getValidSize();
   //set filter for negative x direction
  _pFilter->setInput(tempCoords1, _size);
  _pFilter->setOutput(tempCoords2);
  _pFilter->setThreshold(-_dim.x);
  _pFilter->setFilterDirection(FILTER_SMALLER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _size = _pFilter->getValidSize();

  // set filter for negative y direction ++++++++++++++++++++++++++++++++++++++
  _pFilter->setInput(tempCoords2, _size);
  _pFilter->setOutput(tempCoords1);
  _pFilter->setThreshold(_dim.y);
  _pFilter->setAxis(y);
  _pFilter->setFilterDirection(FILTER_BIGGER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _size = _pFilter->getValidSize();
  // set filter for negative y direction
  _pFilter->setInput(tempCoords1, _size);
  _pFilter->setOutput(tempCoords2);
  _pFilter->setThreshold(-_dim.y);
  _pFilter->setFilterDirection(FILTER_SMALLER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _size = _pFilter->getValidSize();

  // set filter for negative z direction ++++++++++++++++++++++++++++++++++++++
  _pFilter->setInput(tempCoords2, _size);
  _pFilter->setOutput(tempCoords1);
  _pFilter->setThreshold(_dim.z);
  _pFilter->setAxis(z);
  _pFilter->setFilterDirection(FILTER_BIGGER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _size = _pFilter->getValidSize();
  // set filter for negative z direction
  _pFilter->setInput(tempCoords1, _size);
  _pFilter->setOutput(_output);
  _pFilter->setThreshold(-_dim.z);
  _pFilter->setFilterDirection(FILTER_SMALLER);
  if (_pFilter->applyFilter() == FILTER_ERROR)
    return(FILTER_ERROR);
  _validSize = _pFilter->getValidSize();

  return(FILTER_OK);
}

void BoundingBoxFilter::setCentroid(const Point3D& center)
{
  setCentroidDefault(center);
}

void BoundingBoxFilter::setDimension(double x, double y, double z)
{
  _dim.x = x;
  _dim.y = y;
  _dim.z = z;
}

void BoundingBoxFilter::setDimension(double val)
{
  _dim.x = _dim.y = _dim.z = val;
}






