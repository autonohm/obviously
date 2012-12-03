/**
* @file CartesianFilter.h
* @autor christian
* @date  03.12.2012
*
*
*/

#ifndef CARTESIANFILTER_H_
#define CARTESIANFILTER_H_

#include "obcore/filter/Filter.h"
#include "obcore/base/Logger.h"

/**
 * @namespace of obviously library
 */
namespace obvious
{

class Filter;

/**
 * @class Class to filter points out of a given dataset of 3d points in a double
 * array
 *
 *
 */
class CartesianFilter : public Filter
{
public:
  /**
   * Default constructor with initialization
   */
  CartesianFilter()
    : _threshold(0.0),
      _axis(x),
      _input(NULL),
      _output(NULL),
      _size(0),
      _validSize(0)   {  }
  /**
   * Function to start filtering
   */
  FILRETVAL applyFilter(void);
  /**
   * @enum  axis  Enum for switching to xyz axis
   */
  enum Axis{
    x,    /// z axis
    y,    /// y axis
    z     /// z axis
  };
  /**
   * Function to set axis for cartesian filtering
   * @param   ax    axis to filter @see axis
   */
  void setAxis(const Axis ax)         { _axis      = ax; }
  /**
   * Function to set threshold
   * @param   val   threshold value for filtering
   */
  void setThreshold(const double& val){ _threshold = val; }
  /**
   * Function to set input of filter
   * @param   addr        address of input data set
   * @param   inputSize   size of input
   * @return  TRUE if filter is sucessfull
   */
  FILRETVAL setInput(double *addr,const unsigned int inputSize);
  /**
   * Function to get output of filter
   * @param   addr
   */
  void setOutput(double *addr)        { _output    = addr; }

  unsigned int getValidSize(void) const {return _validSize; }
protected:
private:
  double        _threshold;  ///< threshold for filtering
  Axis          _axis;       ///< axis to filter
  double*       _input;      ///< adress of the input buffer
  unsigned int  _size;       ///< size of both buffers
  double*       _output;     ///< adress of the output buffer
  unsigned int  _validSize;

};

};  //namespace

#endif /* CARTESIANFILTER_H_ */
