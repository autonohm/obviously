/**
* @file  CartesianFilter.h
* @author christian
* @date  03.12.2012
*
*
*/

#ifndef CARTESIANFILTER_H_
#define CARTESIANFILTER_H_

#include "obcore/filter/Filter.h"
#include "obcore/filter/FilterBase.h"
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
 */
class CartesianFilter : public FilterBase
{
public:
  /**
   * Default constructor with initialization
   */
  CartesianFilter()
    : FilterBase(),
      _axis(x) {  }
  /**
   * Function to start filtering
   *
   * @note This function musst be called after setting input and output of
   * base class.
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
protected:
private:
  Axis          _axis;       ///< axis to filter

};

};  //namespace

#endif /* CARTESIANFILTER_H_ */
