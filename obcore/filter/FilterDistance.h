/**
* @file FilterDistance.h
* @autor christian
* @date  10.12.2012
*
*
*/

#ifndef FILTERDISTANCE_H_
#define FILTERDISTANCE_H_

#include "obcore/filter/Filter.h"

/**
 * @namespace obvious
 */
namespace obvious
{
 /**
  * @class FilterDistance
  * @brief Abstract class for distance filters as @see EuclideanFilterD.h and
  *        @see CartesianFilter.h
  */
class FilterDistance : public Filter
{
public:
  /**
   * Standard constructor with initialization
   */
  FilterDistance()
    : _centroid(Point3D(0.0, 0.0, 0.0)) { }
  /**
   * virtual destructor
   */
  virtual ~FilterDistance()    { }
  /**
   * Function to set ne centroid for filtering
   * @param   center      center of filter
   */
  virtual void setCentroid(const Point3D& center) = 0;
  /**
   * Function to get set centroid
   * @return  _centroid
   */
  const Point3D& getCentroid(void)   { return _centroid;  }
protected:
  /**
   * Default function for derived classes to call
   * @param   center      center of filter
   */
  virtual void setCentroidDefault(const Point3D& center) {_centroid = center; }
  Point3D         _centroid;          ///< centroid to filter
};

}; // namespace

#endif /* FILTERDISTANCE_H_ */
