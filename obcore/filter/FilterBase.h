/**
* @file   FilterBase.h
* @author Christian
* @date   03.12.2012
*
* @todo implement method for filtering min and max
*/

#ifndef FILTERBASE_H_
#define FILTERBASE_H_

#include "obcore/filter/Filter.h"
#include "obcore/base/Logger.h"

namespace obvious
{

class Filter;

/**
 * @class  FilterBase   Base class for filtering
 */
class FilterBase : public Filter
{
public:
  /**
   * Standard constuctor with default parameters
   */
  FilterBase()
    : _direction(FILTER_BIGGER),
      _threshold(0),
      _input(0),
      _output(0),
      _size(0),
      _validSize(0) { }
  /**
   * Default destructor
   */
  virtual ~FilterBase() {  }
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  FILRETVAL setInput(double *addr,const unsigned int inputSize)
  {
    _input = addr;
    _size  = inputSize;
    return(FILTER_OK);
  }
  /**
   * Function to set threshold
   * @param   val   threshold value for filtering
   */
  void setThreshold(const double& val)  { _threshold = val; }
  /**
   * Function to get output of filter
   * @param   addr
   */
  void setOutput(double *addr)          { _output    = addr; }
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
  unsigned int getValidSize(void) const {return _validSize; }
  /**
   * Function to get output address of filter
   * @return pointer on adress
   */
  double* getOutput(void) const         { return _output; }
protected:
  Direction     _direction;             ///< BIGGER (default) Lowpass, SMALLER for Highpass
  double*       _input;                 ///< adress of the input buffer
  double*       _output;                ///< adress of the output buffer
  double        _threshold;             ///< threshold for filtering
  unsigned int  _size;                  ///< size of both buffers
  unsigned int  _validSize;             ///< number of valid points in data
private:
};

};

#endif /* FILTERBASE_H_ */
