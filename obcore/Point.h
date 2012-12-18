/*********************************************************************************/
/*                                                                               */
/* class Point		                                                         */
/*                                                                               */
/*      Author: Ch. Merkl                                                        */
/*  Created on: 10.09.2012                                                       */
/* Description:	template point class                                             */
/*                                                                               */
/*********************************************************************************/

#ifndef __POINT__
#define __POINT__

#include <iostream>
#include "obcore/math/mathbase.h"
#include "obcore/Axis.h"

namespace obvious {

template <typename T>
class Point
{
public:
    //! default constructor
    Point(const T& x = 0, const T& y = 0) { _x = x; _y = y; }
    //! copy constructor
    Point(const Point& point)     { _x = point._x; _y = point._y; }
    //! get x value
    const T& x(void) const        { return _x; }
    //! get y value
    const T& y(void) const        { return _y; }
    //! set x value
    void setX(const T& x)         { _x = x; }
    //! set y value
    void setY(const T& y)         { _y = y; }
    //! operator +=
    Point& operator+=(const Point& right) { _x += right.m_x; _y += right.m_y; return *this; }
    //! operator -=
    Point& operator-=(const Point& right) { _x -= right.m_x; _y -= right.m_y; return *this; }
    //! operator =
    Point& operator= (const Point& right) { _x  = right.m_x; _y  = right.m_y; return *this; }
    //! operator +
    Point  operator+ (const Point& right) { return Point(_x + right.m_x, _y + right.m_y);   }
    //! operator -
    Point  operator- (const Point& right) { return Point(_x - right.m_x, _y - right.m_y);   }
    /**
     * Index operator [] to return ref on member for read and write access
     * @param[in]   index     index of member @see Axis
     */
    T& operator[](int idx) {
      if (idx == X)
      {
        return _x;
      }
      else if (idx == Y) return _y;
    }
    /**
     * Overloaded comparison operator for class Point
     * @param     right   Object of Point
     * @return    TRUE if both objects have the same values
     */
    bool operator==(const Point& right) const {
      if ((_x == right._x) && (_y == right._y))
        return(true);
      else
        return(false);
    }
    /**
     * Overloaded comparison operator for class Point
     * @param[in]     right   Object of Point
     * @return    TRUE if objects are different
     */
    bool operator!=(const Point& right) const {
      return !(this->operator ==(right));
    }
    /**
     * Overloaded comparison operator LESS for class Point
     * @param[in]     right   Object of Point
     * @return    TRUE if x value is smaller
     */
    bool operator< (const Point& right) const {
      if (_x < right._x)
        return(true);
      else
        if (_y < right._y)
          return(true);
      return(false);
    }
    /**
     * Overloaded comparison operator BIGGER for class Point
     * @param[in]     right   Object of Point
     * @return    TRUE if objects are different
     */
    bool operator> (const Point& right) const {
      if (_x > right._x)
        return true;
      else
        if (_y > right._y)
          return(true);
      return(false);
    }
    /**
     * Ostream operator of class point
     * @param     ostr    ostream object
     * @param[in] p       Point object
     * @return    ostream object
     */
    friend std::ostream &operator<<(std::ostream &ostr, Point &p) {
      return  ostr << "Point: \tx: " << p._x
                    <<        "\ty: " << p._y << std::endl;
    }
    const static unsigned int sizeP = 2;
private:
    T _x;
    T _y;
};

typedef Point<unsigned int> PointU;
typedef Point<int> PointI;
typedef Point<float> PointF;
typedef Point<double> PointD;

}

#endif
