/*
 * EuclideanFilterD.h
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#ifndef EUCLIDEANFILTERD_H_
#define EUCLIDEANFILTERD_H_

#include "obcore/filter/Filter.h"
#include "obcore/filter/FilterBase.h"

namespace obvious
{

/**
 * @class EuclideanFilterD
 * Filter to push values from the input into the output buffer.
 * Values which amount is bigger than a threshold, won't be copied.
 */
class EuclideanFilterD : public FilterBase
{
public:
	EuclideanFilterD(void)
    : FilterBase() {}

	virtual ~EuclideanFilterD(void) { }

	/**
	 * Function to start the Filter
	 */
	FILRETVAL applyFilter(void);
};

} // namespace

#endif /* EUCLIDEANFILTERD_H_ */
