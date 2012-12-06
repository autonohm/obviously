/*
 * EuclideanFilterD.cpp
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#include "obcore/filter/EuclideanFilterD.h"
#include "obcore/math/mathbase.h"

using std::cout;

using namespace obvious;

FILRETVAL EuclideanFilterD::applyFilter(void)
{
	if((!_input)||(!_output))
	{
	  LOGMSG(DBG_ERROR, "Pointer to input or output invalid");
		return(FILTER_ERROR);
	}
	double depthVar = 0;
	double *dPtr    = _input;
	_validSize      = 0;

	for(unsigned int i=0 ; i<_size ; i++)
		_output[i]=0.0;

	if (_direction == FILTER_BIGGER)
	{
    for(unsigned int i=0 ; i<_size ; i+=3) {
      depthVar =  euklideanDistance<double>((double*)dPtr, NULL, 3);
      if(depthVar > _threshold) {
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
    for(unsigned int i=0 ; i<_size ; i+=3) {
      depthVar =  euklideanDistance<double>((double*)dPtr, NULL, 3);
      if(depthVar <= _threshold) {
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

