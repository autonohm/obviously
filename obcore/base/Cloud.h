#ifndef __CLOUD__
#define __CLOUD__

#include <vector>
#include "obcore/Point3D.h"
#include "obcore/RGBColor.h"
#include "obcore/math/VecD.h"
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
        const_iterator(std::vector<VecD>::const_iterator itCoords = std::vector<VecD>::const_iterator(),
                       std::vector<RGBColor>::const_iterator itRgb = std::vector<RGBColor>::const_iterator())
            : _itCoords(itCoords), _itRgb(itRgb) { }

        bool operator<(const const_iterator& it) const { return _itCoords < it._itCoords && _itRgb < it._itRgb; }
        const_iterator& operator=(const const_iterator& it) { _itCoords = it._itCoords; _itRgb = it._itRgb; return *this; }
        const_iterator& operator++(void) { ++_itCoords; ++_itRgb; return *this; }
        RgbPoint3D operator*(void) { return RgbPoint3D((*_itCoords).at(X), (*_itCoords).at(Y), (*_itCoords).at(Z), *_itRgb); }

    private:
        std::vector<VecD>::const_iterator _itCoords;
        std::vector<RGBColor>::const_iterator _itRgb;
    };

    Cloud(const unsigned int size, const double* coords, const unsigned char* rgb = 0);
    Cloud(const std::vector<Point3D>& points, const std::vector<RGBColor>& color = std::vector<RGBColor>());

    const_iterator begin(void) const { return const_iterator(_coords.begin(), _rgb.begin()); }
    const_iterator end(void) const { return const_iterator(_coords.end(), _rgb.end()); }

private:

//    class Cube
//    {
//    public:
//        Cube(const double x, const double y, const double z, const double size) : _x(x), _y(y), _z(z), _size(size) { }
//
//    private:
//        std::vector<VecD&> _coords;
//    };

    std::vector<VecD> _coords;
    std::vector<RGBColor> _rgb;
};

} // end namespace

#endif
