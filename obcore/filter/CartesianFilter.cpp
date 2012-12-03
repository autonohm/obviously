/**
* @file CartesianFilter.cpp
* @autor christian
* @date  03.12.2012
*
*
*/

#include "obcore/filter/CartesianFilter.h"
#include <iostream>

using namespace obvious;

/*
 * Function to set input of filter
 */
FILRETVAL CartesianFilter::setInput(double *addr,const unsigned int inputSize)
{
  _input = addr;
  _size  = inputSize;
  return(FILTER_OK);
}

/*
 * Funtion to apply filter
 */
FILRETVAL CartesianFilter::applyFilter(void)
{
  if((!_input)||(!_output))
  {
    LOGMSG(DBG_ERROR, "Pointer to input or output invalid");
    return(FILTER_ERROR);
  }
  _validSize      = 0;
  double distance = 0;
  double *dPtr    = _input;

  // init output array
  for(unsigned int i=0;i<_size;i++)
    _output[i]=0.0;

  for(unsigned int i=_axis ; i<_size/3 ; i++)
  {
    if(_input[i] > _threshold)
    {
      dPtr += 3;
      _validSize += 3;
    }
    else
    {
      for(unsigned int j=0 ; j<3 ; j++)
        *_output++=*dPtr++;
    }
  }
  return(FILTER_OK);
}






