/**
* @file   FilterBoundingBox.h
* @author Christian Pfitzner
* @date   10.12.2012
*
*
*/

#ifndef FILTERBOUNDINGBOX_H_
#define FILTERBOUNDINGBOX_H_

#include "obcore/filter/FilterDistance.h"
#include "obcore/filter/CartesianFilter.h"

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class
 */
class BoundingBoxFilter : public FilterDistance
{
public:
  /**
   * @struct Dimension
   */
  struct Dimension{
    double x;
    double y;
    double z;
  };
  /**
   * Standard constructor with initialization
   */
  BoundingBoxFilter(void)
    : FilterDistance()
      {
         _pFilter  = new CartesianFilter;
        _dim.x     = 0.0;
        _dim.y     = 0.0;
        _dim.z     = 0.0;
      }
  /**
   * Standard destructor
   */
  ~BoundingBoxFilter();
  /**
   * Function to start filtering
   *
   * @note This function must be called after setting input and output of
   * base class.
   */
  FILRETVAL applyFilter(void);
  /**
   * Function to get set centroid
   * @return  _centroid
   */
  void setCentroid(const Point3D& center);
  /**
   * Function to set up dimension of bounding box around centroid
   * @param     x   x value
   * @param     y   y value
   * @param     z   z value
   */
  void setDimension(double x, double y, double z);
  /**
   * Function to set up dimension of dice for filtering
   * @param     x   edge length of dice
   */
  void setDimension(double val);
private:
  CartesianFilter* _pFilter;    //!< object of cartesian filter
  Dimension        _dim;        //!< structure for dimension of box

  // functions private to avoid public implementation
  void setThreshold(const double& val)          { }
  void setFilterDirection(Direction direction)  { }
};

};



#endif /* FILTERBOUNDINGBOX_H_ */
