/*
 * EuclideanFilterD.cpp
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#include "obcore/filter/EuclideanFilterD.h"
#include "obcore/math/mathbase.h"
#include <iostream>

using std::cout;

namespace obvious
{

FILRETVAL EuclideanFilterD::applyFilter(void)
{
	if((!_input)||(!_output))
	{
		cout<<"\nEUCLF: Pointer to input or output invalid!\n";
		return(FILTER_ERROR);
	}
	double depthVar = 0;
	double *dPtr    = _input;
	_validSize      = 0;

	for(unsigned int i=0 ; i<_size ; i++)
		_output[i]=0.0;

	for(unsigned int i=0 ; i<_size/3 ; i++)
	{
		depthVar =	euklideanDistance<double>((double*)dPtr, NULL, 3);
		if(depthVar > _threshold)
		{
		  _validSize += 3;
		  dPtr       += 3;
		}

		else
		{
			for(unsigned int j=0 ; j<3 ; j++)
				*_output++ = *dPtr++;
		}
	}
	return(FILTER_OK);
}

/***************************************************************************************************/

}
