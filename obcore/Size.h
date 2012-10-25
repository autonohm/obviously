/*********************************************************************************/
/*                                                                               */
/* class size                                                                    */
/*                                                                               */
/*      Author: Ch. Merkl, R. Koch                                               */
/*  Created on: 10.09.2012                                                       */
/* Description:	template class                                                   */
/*                                                                               */
/*********************************************************************************/

#ifndef __SIZE__
#define __SIZE__

namespace obvious {

template <typename T>
class Size
{
public:
    //! constructor
    Size(const T& width = 0, const T& height = 0) : m_width(width), m_height(height) { }

    //! get width
    const T& width (void) const { return m_width ; }
    //! get height
    const T& height(void) const { return m_height; }

    //! set width
    void setWidth (const T& width ) { m_width  = width ; }
    //! set height
    void setHeight(const T& height) { m_height = height; }

    //! operator !=
    bool operator!=(const Size<T>& right) const { return m_width != right.m_width || m_height != right.m_height; }
    //! operator ==
    bool operator==(const Size<T>& right) const { return m_width == right.m_width || m_height == right.m_height; }

private:
    T m_width;
    T m_height;
};

typedef Size<unsigned int> SizeU;
typedef Size<int> SizeI;
typedef Size<float> SizeF;
typedef Size<double> SizeD;

}

#endif
