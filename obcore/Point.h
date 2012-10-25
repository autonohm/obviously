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

namespace obvious {

template <typename T>
class Point
{
public:
    //! default constructor
    Point(const T& x = 0, const T& y = 0) { m_x = x; m_y = y; }

    //! copy constructor
    Point(const Point& point)     { m_x = point.m_x; m_y = point.m_y; }

    //! get x value
    const T& x(void) const        { return m_x; }
    //! get y value
    const T& y(void) const        { return m_y; }

    //! set x value
    void setX(const T& x)         { m_x = x; }
    //! set y value
    void setY(const T& y)         { m_y = y; }

    //! operator +=
    Point& operator+=(const Point& right) { m_x += right.m_x; m_y += right.m_y; return *this; }
    //! operator -=
    Point& operator-=(const Point& right) { m_x -= right.m_x; m_y -= right.m_y; return *this; }
    //! operator =
    Point& operator= (const Point& right) { m_x  = right.m_x; m_y  = right.m_y; return *this; }
    //! operator +
    Point  operator+ (const Point& right) { return Point(m_x + right.m_x, m_y + right.m_y);   }
    //! operator -
    Point  operator- (const Point& right) { return Point(m_x - right.m_x, m_y - right.m_y);   }


private:
    T m_x;
    T m_y;
};

typedef Point<unsigned int> PointU;
typedef Point<int> PointI;
typedef Point<float> PointF;
typedef Point<double> PointD;

}

#endif
