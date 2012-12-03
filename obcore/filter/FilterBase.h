/**
* @file FilterBase.h
* @autor christian
* @date  03.12.2012
*
*
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
  FilterBase()
    : _threshold(0),
      _input(),
      _output(),
      _size(0),
      _validSize(0) { }
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
  double        _threshold;  ///< threshold for filtering
  double*       _input;      ///< adress of the input buffer
  unsigned int  _size;       ///< size of both buffers
  double*       _output;     ///< adress of the output buffer
  unsigned int  _validSize;
private:
};

};

#endif /* FILTERBASE_H_ */
