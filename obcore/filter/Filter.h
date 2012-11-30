/*
 * Filter.h
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#ifndef FILTER_H_
#define FILTER_H_

namespace obvious
{

/**
 * Enum datatype for the return values of the filter classes
 */
enum FILRETVAL{
	FILTER_OK,
	FILTER_ERROR
};

/**
 * Interface for the filters
 */
class Filter
{
public:
	virtual ~Filter(){}
	/**
	 * Function to start the filter
	 * @return Returns error in an error case
	 */
	virtual FILRETVAL applyFilter(void)=0;
private:
};

}

#endif /* FILTER_H_ */
