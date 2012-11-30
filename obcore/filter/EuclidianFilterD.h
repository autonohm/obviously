/*
 * EuclidianFilterD.h
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#ifndef EUCLIDIANFILTERD_H_
#define EUCLIDIANFILTERD_H_

#include "obcore/filter/Filter.h"

namespace obvious
{

class Filter;

/**
 * @class EuclidianFilterD
 * Filter to push values from the input into the output buffer.
 * Values which amount is bigger than a threshold, won't be copied.
 */
class EuclidianFilterD : public Filter
{
public:
	EuclidianFilterD(void){}
	virtual ~EuclidianFilterD(){}

	/**
	 * Function to start the Filter
	 */
	FILRETVAL applyFilter(void);

	/**
	 * Function to set the value of the threshold parameter
	 * @param[in] val new threshold value
	 */
	void setThreshold(const double val){_threshold=val;}

	/**
	 * Function to set the pointer to the input buffer and its size
	 * @param[in] addr Adress of the input buffer
	 * @param[in] inputSize size of the input buffer
	 *
	 * The input buffer has to be allocated by the calling function. The size of the input buffer
	 * has to be the same as the size of the output buffer.
	 */
	FILRETVAL setInput(double *addr,const unsigned int inputSize);

	/**
	 * Function the set the adress of the output buffer.
	 * @param[in] addr Adress of the output buffer
	 *
	 * The output buffer has to be allocated by the calling function. The size of this buffer has
	 * to match the size of the _size parameter.
	 */
	void setOutput(double *addr){_output=addr;}
private:
	double _threshold;   ///< threshold parameter of the filter
	double *_input;      ///< adress of the input buffer
	unsigned int _size;  ///< size of both buffers
	double *_output;     ///< adress of the output buffer
};

}

#endif /* EUCLIDIANFILTERD_H_ */
