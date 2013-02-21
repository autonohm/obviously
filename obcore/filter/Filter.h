/**
* @file   Filter.h
* @author Christian Pfitzner
* @date   03.12.2012

*/

#ifndef IFILTER_H_
#define IFILTER_H_
/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * Enum datatype for the return values of the filter classes
 */
enum FILRETVAL{
  FILTER_OK,  //!< FILTER_OK
  FILTER_ERROR//!< FILTER_ERROR
};
/**
 * @enum Direction
 * Specify the direction of filtering
 */
enum Direction {
  FILTER_BIGGER,    //!< FILTER_BIGGER
  FILTER_SMALLER    //!< FILTER_SMALLER
};

/**
 * @class IFilters
 * @brief Interface class for filters of obcore
 */
class IFilter
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
  virtual ~IFilter(void) { }
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
//	virtual FILRETVAL setInput(double *addr, const unsigned int inputSize) = 0;
  /**
   * Function to get output of filter
   * @param   addr
   */
  //virtual void setOutput(double *addr) = 0;
  /**
   * Function to set threshold
   * @param   val   threshold value for filtering
   */
  //virtual void setThreshold(const double& val) = 0;
  /**
   * Function to change direction of filter
   * @param   direction   @see enum Direction
   */
  //virtual void setFilterDirection(Direction direction) = 0;
  //~~~~~~~~~~~~~~~~~~ Functions to GET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to get output address of filter
   * @return pointer on adress
   */
  //virtual double* getOutput(void) const = 0;
  /**
   * Function to return valid size
   * @return  valid size of filtered points
   */
  //virtual unsigned int getValidSize(void) const = 0;
};
}

#endif /* IFILTER_H_ */

//##############################################################################
#ifndef FILTERBASE_H_
#define FILTERBASE_H_

#include "obcore/filter/Filter.h"
#include "obcore/base/Logger.h"
#include "obcore/Point3D.h"
/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class   FilterBase
 * @brief   Base class for filtering
 */
class Filter : public IFilter
{
public:
  /**
   * Standard constuctor with default parameters
   */
  Filter()
    : _direction(FILTER_BIGGER),
      _input(NULL),
      _output(NULL),
      _threshold(0.0),
      _size(0),
      _validSize(0) { }
  /**
   * Default destructor
   */
  virtual ~Filter() {  }
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  FILRETVAL setInput(double *addr,const unsigned int inputSize)
    {_input = addr; _size  = inputSize; return(FILTER_OK); }
  /**
   * Function to set threshold
   * @param   val   threshold value for filtering
   */
  void setThreshold(const double& val)         { _threshold = val; }
  /**
   * Function to get output of filter
   * @param   addr
   */
  void setOutput(double *addr)                 { _output    = addr; }
  /**
   * Function to change direction of filter
   * @param   direction   @see enum Direction
   */
  void setFilterDirection(Direction direction) { _direction = direction; }
  //~~~~~~~~~~~~~~~~~~ Functions to GET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to return valid size
   * @return  valid size of filtered points
   */
  unsigned int getValidSize(void) const        {return _validSize; }
  /**
   * Function to get output address of filter
   * @return pointer on adress
   */
  double* getOutput(void) const                { return _output; }
protected:
  Direction       _direction;             ///< BIGGER (default) Lowpass, SMALLER for Highpass
  double*        _input;                 ///< adress of the input buffer
  double*        _output;                ///< adress of the output buffer
  double         _threshold;             ///< threshold for filtering
  unsigned int   _size;                  ///< size of both buffers
  unsigned int   _validSize;             ///< number of valid points in data
};

};

#endif /* FILTER_H_ */
