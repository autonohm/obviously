/**
* @file CartesianFilter.cpp
* @autor christian
* @date  03.12.2012
*
*
*/

#include "obcore/filter/CartesianFilter.h"

using namespace obvious;

/*
 * Function to start filtering
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
  for(unsigned int i=0 ; i<_size ; i++)
    _output[i]=0.0;

  if (_direction == FILTER_BIGGER)
  {
    for(unsigned int i=_axis ; i<_size ; i+=3) {
      if(_input[i] > _threshold) {
        dPtr += 3;
      }
      else {
        for(unsigned int j=0 ; j<3 ; j++)
          *_output++=*dPtr++;
        _validSize += 3;
      }
    }
  }
  else // FILTER_SMALLER
  {
    for(unsigned int i=_axis ; i<_size ; i+=3) {
      if(_input[i] <= _threshold) {
        dPtr += 3;
      }
      else {
        for(unsigned int j=0 ; j<3 ; j++)
          *_output++=*dPtr++;
        _validSize += 3;
      }
    }
  }
  return(FILTER_OK);
}






