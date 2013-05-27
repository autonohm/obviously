/*
 * EuclideanFilter.h
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#ifndef EUCLIDEANFilter_H_
#define EUCLIDEANFilter_H_

#include "obcore/filter/FilterDistance.h"

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class  EuclideanFilter
 * @brief  Filter to push values from the input into the output buffer.
 *         Values which amount is bigger than a threshold, won't be copied.
 */
class EuclideanFilter : public FilterDistance
{
public:
  /**
   * Default constructor
   */
	EuclideanFilter(void)
    : FilterDistance()    { }
	/**
	 * virtual destructor
	 */
	virtual ~EuclideanFilter(void) { }
	/**
	 * Function to start the Filter
	 */
	FILRETVAL applyFilter(void);
	/**
   * Function to set centroid for filtering
   * @param   center   center point
   */
	void setCentroid(const Point3D& center);
};

}; // namespace

#endif /* EUCLIDEANFilter_H_ */
