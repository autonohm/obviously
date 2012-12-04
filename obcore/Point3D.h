/**
* @file   Point3D.h
* @author Ch. Merkl
* @date   25.10.2012
*/

#ifndef __POINT_3D__
#define __POINT_3D__

#include "obcore/base/Logger.h"
#include <iostream>

/**
 * @namespace of obviously library
 */
namespace obvious {
/**
 * @class Class for points in R3 euclidian space
 */
class Point3D
{
public:
    //! default constructor
    Point3D(const double x = 0.0, const double y = 0.0, const double z = 0.0)
      : _x(x), _y(y), _z(z) { }
    //! copy constructor
    Point3D(const Point3D& point) : _x(point._x), _y(point._y), _z(point._z) { }
    //! virtual destructor
    virtual ~Point3D(void) { }

    //! get x value
    const double& x(void) const { return _x; }
    //! get y value
    const double& y(void) const { return _y; }
    //! get z value
    const double& z(void) const { return _z; }

    //! set x value
    void setX(const double& x) { _x = x; }
    //! set y value
    void setY(const double& y) { _y = y; }
    //! set z value
    void setZ(const double& z) { _z = z; }

    //! operator +=
    virtual Point3D& operator+=(const Point3D& right) { _x += right._x; _y += right._y; _z += right._z; return *this; }
    //! operator -=
    virtual Point3D& operator-=(const Point3D& right) { _x -= right._x; _y -= right._y; _z -= right._z; return *this; }
    //! operator =
    virtual Point3D& operator= (const Point3D& right) { _x  = right._x; _y  = right._y; _z  = right._z; return *this; }
    //! operator +
    virtual Point3D  operator+ (const Point3D& right) { return Point3D(_x + right._x, _y + right._y, _z + right._z);  }
    //! operator -
    virtual Point3D  operator- (const Point3D& right) { return Point3D(_x - right._x, _y - right._y, _z + right._z);  }
    //! operator to return member
    virtual double operator[](const unsigned int i) const
    {
      if (i == 0)
        return _x;
      else if (i == 1)
        return _y;
      else if (i == 2)
        return _z;
      // throw error message
      else
      {
        LOGMSG(DBG_ERROR, "Invalid index to access Point3D");
        return(0.0);
      }
    }
    //! comparison operator
    virtual bool operator==(const Point3D& right) const
    {
      if ((_x == right._x) && (_y == right._y) &&(_z == right._z))
        return(true);
      else
        return(false);
    }
    //! ostream operator of class point
    friend std::ostream &operator<<(std::ostream &ostr, Point3D &p)
    {
      return  ostr << "Point: \tx: " << p._x
                   <<        "\ty: " << p._y
                   <<        "\tz: " << p._z << std::endl;

    }
protected:
    double _x;
    double _y;
    double _z;
};

} // namespace

#endif
