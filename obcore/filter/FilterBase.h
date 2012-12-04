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

namespace obvious
{

class Filter;

class FilterBase : public Filter
{
public:
  /**
   * Standard constuctor with default parameters
   */
  FilterBase()
    : _threshold(0),
      _input(0),
      _output(0),
      _size(0),
      _validSize(0) { }
  /**
   * Default destructor
   */
//  virtual ~FilterBase()
//  {
//    delete [] _input;
//    delete [] _output;
//  }
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
  FILRETVAL setInput(double *addr,const unsigned int inputSize)
  {
    _input = addr;
    _size  = inputSize;
    return(FILTER_OK);
  }
  /**
   * Function to return valid size
   * @return  valid size of filtered points
   */
  unsigned int getValidSize(void) const {return _validSize; }
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
protected:
  double*       _input;                 ///< adress of the input buffer
  double*       _output;                ///< adress of the output buffer
  double        _threshold;             ///< threshold for filtering
  unsigned int  _size;                  ///< size of both buffers
  unsigned int  _validSize;
private:
};

};

#endif /* FILTERBASE_H_ */
