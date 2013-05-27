/*
 * EuclideanFilter.cpp
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#include "obcore/filter/EuclideanFilter.h"
#include "obcore/math/mathbase.h"

using namespace obvious;

FILRETVAL EuclideanFilter::applyFilter(void)
{
  // check if input and output are set properly
	if((!_input)||(!_output))
	{
	  LOGMSG(DBG_ERROR, "Pointer to input or output invalid");
		return(FILTER_ERROR);
	}
	double depthVar = 0;
	double *dPtr    = _input;
	_validSize      = 0;

	// init output with zeros
	for(unsigned int i=0 ; i<_size ; i++)
		_output[i]=0.0;

	if (_direction == FILTER_BIGGER)
	{
    for(unsigned int i=0 ; i<_size ; i+=Point3D::sizeP) {
      // new coords array for filtering with centroid
      double coord[3] = { dPtr[i]   + _centroid[X],
                          dPtr[i+1] + _centroid[Y],
                          dPtr[i+2] + _centroid[Z]
      };
      depthVar =  l2Norm<double>(coord, Point3D::sizeP);
      if(depthVar > _threshold) {
        dPtr += Point3D::sizeP;
      }
      else {
        for(unsigned int j=0 ; j<Point3D::sizeP ; j++)
          *_output++=*dPtr++;
        _validSize += Point3D::sizeP;
      }
    }
	}
	else // FILTER_SMALLER
	{
    for(unsigned int i=0 ; i<_size ; i+=Point3D::sizeP) {
      depthVar =  l2Norm<double>((double*)dPtr, Point3D::sizeP);
      if(depthVar <= _threshold) {
        dPtr += Point3D::sizeP;
      }
      else {
        for(unsigned int j=0 ; j<Point3D::sizeP ; j++)
          *_output++=*dPtr++;
        _validSize += Point3D::sizeP;
      }
    }
	}
	return(FILTER_OK);
}

/*
 * Function to set centroid
 */
void EuclideanFilter::setCentroid(const Point3D& center)
{
  setCentroidDefault(center);
}

