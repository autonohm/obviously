/*
 * EuclideanFilterD.h
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#ifndef EUCLIDEANFILTERD_H_
#define EUCLIDEANFILTERD_H_

#include "obcore/filter/FilterDistance.h"

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class  EuclideanFilterD
 * @brief  Filter to push values from the input into the output buffer.
 *         Values which amount is bigger than a threshold, won't be copied.
 */
class EuclideanFilterD : public FilterDistance
{
public:
  /**
   * Default constructor
   */
	EuclideanFilterD(void)
    : FilterDistance()    { }
	/**
	 * virtual destructor
	 */
	virtual ~EuclideanFilterD(void) { }
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

#endif /* EUCLIDEANFILTERD_H_ */
