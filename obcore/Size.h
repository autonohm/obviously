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
    Size(const T& width = 0, const T& height = 0) : _width(width), _height(height) { }

    //! copy constructor
    Size(const Size<T>& size) : _width(size._width), _height(height) { }

    //! get width
    const T& width (void) const { return _width ; }
    //! get height
    const T& height(void) const { return _height; }

    //! set width
    void setWidth (const T& width ) { _width  = width ; }
    //! set height
    void setHeight(const T& height) { _height = height; }

    //! operator !=
    bool operator!=(const Size<T>& right) const { return _width != right._width || _height != right._height; }
    //! operator ==
    bool operator==(const Size<T>& right) const { return _width == right._width || _height == right._height; }

private:
    T _width;
    T _height;
};

typedef Size<unsigned int> SizeU;
typedef Size<int> SizeI;
typedef Size<float> SizeF;
typedef Size<double> SizeD;

}

#endif
