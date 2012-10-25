/*********************************************************************************/
/*                                                                               */
/* rectangle                                                                     */
/*                                                                               */
/*      Author: Ch. Merkl                                                        */
/*  Created on: 25.10.2012                                                       */
/* Description:	template class                                                   */
/*                                                                               */
/*********************************************************************************/

#ifndef __RECT__
#define __RECT__

#include "Point.h"
#include "Size.h"

namespace obvious {

template <typename T>
class Rect
{
public:
    //! constructor
    /*!
      constructs a rect from the point topLeft with the size size.
    */
    Rect(const Point<T>& topLeft, const Size<T>& size) : m_topLeft(topLeft), m_size(size) { }
    //! constructor
    /*!
      constructs a rect from the point topLeft to the point bottomRight.
    */
    Rect(const Point<T>& topLeft, const Point<T>& bottomRight)
        : m_topLeft(topLeft), m_size(bottomRight.x() - topLeft.x() + 1, bottomRight.y() - topLeft.y() + 1) { }
    //! default constructor
    Rect(const T& x = 0, const T& y = 0, const T& width = 0, const T& height = 0) : m_topLeft(x, y), m_size(width, height) { }
    //! copy constructor
    Rect(const Rect<T>& rect) : m_topLeft(rect.m_topLeft), m_size(rect.m_size) { }

    //! get x value from the top left point.
    const T& x(void)      const     { return m_topLeft.x(); }
    //! get y value from the top left point.
    const T& y(void)      const     { return m_topLeft.y(); }
    //! get width
    const T& width(void)  const     { return m_size.width(); }
    //! get height
    const T& height(void) const     { return m_size.height(); }
    //! get size
    const Size<T>& size(void) const { return m_size; }

    //! set x value from the top left point
    void setX     (const T& x)      { m_topLeft.setX(x); }
    //! set y value from the top left point
    void setY     (const T& y)      { m_topLeft.setY(y); }
    //! set width
    void setWidth (const T& width)  { m_size.setWidth(width); }
    //! set height
    void setHeight(const T& height) { m_size.setHeight(height); }

private:
    Point<T> m_topLeft;
    Size<T>  m_size;
};

typedef Rect<unsigned int> RectU;
typedef Rect<int> RectI;
typedef Rect<float> RectF;
typedef Rect<double> RectD;

}

#endif
