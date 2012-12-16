/**
* @file   CartesianFilter.h
* @author Christian Pfitzner
* @date   03.12.2012
*
*
*/

#ifndef CARTESIANFILTER_H_
#define CARTESIANFILTER_H_

#include "obcore/filter/FilterDistance.h"

/**
 * @namespace obvious
 */
namespace obvious
{
class FilterDistance;
/**
 * @class   CartesianFilter
 * Class to filter points out of a given dataset of 3d points in a double array
 */
class CartesianFilter : public FilterDistance
{
public:
  /**
   * Default constructor with initialization
   */
  CartesianFilter(void)
    : FilterDistance(),
      _axis(x){ }
  ~CartesianFilter(void) { }
  /**
   * Function to start filtering
   *
   * @note This function must be called after setting input and output of
   * base class.
   */
  FILRETVAL applyFilter(void);
  /**
   * Function to set specified axis for filtering
   * @param   a   defined axis
   */
  void setAxis(const Axis& a);
  /**
   * Function to set centroid for filtering
   * @param   center   center point
   */
  void setCentroid(const Point3D& center);
private:
  Axis            _axis;           ///< axis to filter
};

};  //namespace

#endif /* CARTESIANFILTER_H_ */
