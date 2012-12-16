/*
 * EuclideanFilterVecD.h
 *
 *  Created on: 30.11.2012
 *      Author: phil
 */

#ifndef EUCLIDEANFILTERVECD_H_
#define EUCLIDEANFILTERVECD_H_

#include "obcore/filter/Filter.h"
#include <vector>

namespace obvious
{

using std::vector;

/**
 * @class EuclideanFilterVecD
 * Filter to push values from the input into the output buffer.
 * Uses the vector class of the stl.
 * Values which amount is bigger than a threshold, won't be copied.
 */
class EuclideanFilterVecD : public Filter
{
public:
	EuclideanFilterVecD(void)
    : _threshold(0.0),
      _output(NULL),
      _input(NULL) { }
	virtual ~EuclideanFilterVecD(){ }


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
	 * Function to set the pointer to the input vector
	 * @param[in] vec Adress of the input vector
	 */
	void setInput(vector<double> *vec){_input=vec;}

	/**
	 * Function to set the pointer to the output vector
	 * @param[in] vec Adress of the output vector
	 */
	void setOutput(vector<double> *vec){_output=vec;}
private:
	double _threshold;              ///< Threshold parameter of the filter
	vector<double> *_input;         ///< Adress of the input vector
	vector<double> *_output;        ///< Adress of the output vector
};

}

#endif /* EUCLIDEANFILTERVECD_H_ */
