/**
* @file   Point3D.h
* @author Ch. Merkl and Christian Pfitzner
* @date   25.10.2012
*/

#ifndef __POINT_3D__
#define __POINT_3D__

#include "obcore/Axis.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <iostream>

/**
 * @namespace of obviously library
 */
namespace obvious {

/**
 * @class Point3D   Class for points in R3 euclidian space
 */
class Point3D
{
public:
    /**
     * Default constructor
     * @param[in]   x     x value
     * @param[in]   y     y value
     * @param[in]   z     z value
     */
    Point3D(const double x = 0.0, const double y = 0.0, const double z = 0.0)
      : _x(x), _y(y), _z(z) { }
    /**
     * Copy constructor
     * @param     p   object of Point3D
     */
    Point3D(const Point3D& p)
      : _x(p._x), _y(p._y), _z(p._z) { }
    /**
     * Standard destructor
     */
    ~Point3D(void) { }
    /**
     * Function to return x value
     * @return    x value
     */
    const double& x(void) const { return _x; }
    /**
     * Function to return y value
     * @return    y value
     */
    const double& y(void) const { return _y; }
    /**
     * Function to return z value
     * @return    z value
     */
    const double& z(void) const { return _z; }
    /**
     * Function to set x value
     * @param[in]   x   x value
     */
    void setX(const double& x) { _x = x; }
    /**
     * Function to set y value
     * @param[in]   y   y value
     */
    void setY(const double& y) { _y = y; }
    /**
     * Function to set z value
     * @param[in]   z   z value
     */
    void setZ(const double& z) { _z = z; }
    /**
     * Overloaded allocation operator=
     * @param     right   object of Point3D
     * @return    object of Point3D
     */
    Point3D& operator= (const Point3D& right) { _x  = right._x; _y  = right._y; _z  = right._z; return *this; }
    /**
     * Index operator [] to return ref on member for read and write access
     * @param[in]   index     index of member @see Axis
     */
    double& operator[](int idx) {
      if (idx == X)
      {
        return _x;
      }
      else if (idx == Y) return _y;
      else if (idx == Z) return _z;
    }
    /**
     * Overloaded comparison operator for class Point3D
     * @param     right   Object of Point3D
     * @return    TRUE if both objects have the same values
     */
    bool operator==(const Point3D& right) const
    {
      if ((_x == right._x) && (_y == right._y) &&(_z == right._z))
        return(true);
      else
        return(false);
    }
    /**
     * Overloaded comparison operator for class Point3D
     * @param[in]     right   Object of Point
     * @return    TRUE if objects are different
     */
    bool operator!=(const Point3D& right) const {
      return !(this->operator ==(right));
    }
    /**
     * Overloaded comparison operator LESS for class Point3D
     * @param[in]     right   Object of Point
     * @return    TRUE if x value is smaller
     */
    bool operator< (const Point3D& right) const {
      if (_x < right._x)
        return(true);
      else
        if (_y < right._y)
          return(true);
        else
          if (_z < right._z)
            return(true);
      return(false);
    }
    /**
     * Overloaded comparison operator BIGGER for class Point3D
     * @param[in]     right   Object of Point
     * @return    TRUE if objects are different
     */
    bool operator> (const Point3D& right) const {
      if (_x > right._x)
        return(true);
      else
        if (_y > right._y)
          return(true);
        else
          if (_z > right._z)
            return(true);
      return(false);
    }
    /**
     * Ostream operator of class point
     * @param     ostr    ostream object
     * @param[in] p       Point3D object
     * @return    ostream object
     */
    friend std::ostream &operator<<(std::ostream &ostr, Point3D &p)
    {
      return  ostr << "Point: \tx: " << p._x
                   <<        "\ty: " << p._y
                   <<        "\tz: " << p._z << std::endl;
    }
    const static unsigned int sizeP = 3;
protected:
    double _x;          //!< x element of point
    double _y;          //!< y element of point
    double _z;          //!< z element of point
};

} // namespace

#endif
