#ifndef __CLOUD__
#define __CLOUD__

#include <vector>
#include "obcore/Point3D.h"
#include "obcore/rgbColor.h"
#include "obcore/math/MatRGB.h"
#include "obcore/math/MatD.h"
#include "obcore/RgbPoint3D.h"

namespace obvious {

class Cloud
{
public:

    enum Dimension {
        X = 0,
        Y = 1,
        Z = 2
    };

    class const_iterator
    {
    public:
        const_iterator(const unsigned int position = 0, const Cloud* cloud = 0) : _position(position), _cloud(cloud) { }
        const_iterator(const const_iterator& it) : _position(it._position), _cloud(it._cloud) { }

        const_iterator& operator=(const const_iterator& it)
        {
            _position = it._position;
            _cloud    = it._cloud;
            return *this;
        }

        const_iterator& operator++(void)
        {
            _position++;
            return *this;
        }

        const_iterator& operator--(void)
        {
            _position--;
            return *this;
        }

        RgbPoint3D operator*(void) const
        {
            return _cloud->at(_position);
        }

        bool operator<(const const_iterator& it) const { return _position < it._position; }
        bool operator<=(const const_iterator& it) const { return _position <= it._position; }
        bool operator>(const const_iterator& it) const { return _position > it._position; }
        bool operator>=(const const_iterator& it) const { return _position >= it._position; }
        bool operator==(const const_iterator& it) const { return _position == it._position; }
        bool operator!=(const const_iterator& it) const { return _position != it._position; }

    private:
        unsigned int _position;
        const Cloud* _cloud;
    };

    Cloud(const unsigned int size, const double* coords, const unsigned char* rgb = 0);
    Cloud(const std::vector<Point3D>& points, const std::vector<RGBColor>& color = std::vector<RGBColor>());

    RgbPoint3D at(const unsigned int index) const { return RgbPoint3D(_points.at(index, X), _points.at(index, Y), _points.at(index, Z), _rgb.rgb(index, 0)); }

    const_iterator begin(void) const { return const_iterator(0, this); }
    const_iterator end(void) const { return const_iterator(_size, this); }

private:
    unsigned int _size;
    MatD   _points;
    MatRGB _rgb;
};

} // end namespace

#endif
