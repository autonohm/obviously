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
  /**
   * @enum  axis  Enum for switching to xyz axis
   */
  enum Axis{
    x,    /// z axis
    y,    /// y axis
    z     /// z axis
  };
  /**
   * @enum Direction
   * Specify the direction of filtering
   */
  enum Direction {
    FILTER_BIGGER,    //!< FILTER_BIGGER
    FILTER_SMALLER    //!< FILTER_SMALLER
  };
  virtual ~Filter(void) { }
	/**
	 * Function to start the filter
	 * @return Returns error in an error case
	 */
	virtual FILRETVAL applyFilter(void) = 0;
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
	virtual FILRETVAL setInput(double *addr, const unsigned int inputSize) = 0;
  /**
   * Function to get output of filter
   * @param   addr
   */
  virtual void setOutput(double *addr) = 0;
  /**
   * Function to set threshold
   * @param   val   threshold value for filtering
   */
  virtual void setThreshold(const double& val) = 0;
  /**
   * Function to change direction of filter
   * @param   direction   @see enum Direction
   */
  virtual void setFilterDirection(Direction direction) = 0;
  //~~~~~~~~~~~~~~~~~~ Functions to GET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to get output address of filter
   * @return pointer on adress
   */
  virtual double* getOutput(void) const = 0;
  /**
   * Function to return valid size
   * @return  valid size of filtered points
   */
  virtual unsigned int getValidSize(void) const = 0;
};
}

#endif /* FILTER_H_ */
