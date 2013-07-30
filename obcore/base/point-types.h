#ifndef __POINT_TYPES__
#define __POINT_TYPES__

#include <stdint.h>

namespace obvious {

struct PointXyz
{
    PointXyz(void) { }
    PointXyz(const float valX, const float valY, const float valZ) : x(valX), y(valY), z(valZ) { }
    PointXyz(const PointXyz& point) : x(point.x), y(point.y), z(point.z) { }

    inline PointXyz& operator=(const PointXyz& point)
    {
        x = point.x;
        y = point.y;
        z = point.z;

        return *this;
    }

    inline bool operator==(const PointXyz& point) const { return x == point.x && y == point.y && z == point.z; }

    float x;
    float y;
    float z;
};

struct Normal : public PointXyz
{
    Normal(void) : PointXyz() { }
    Normal(const float valX, const float valY, const float valZ, const float valCurvature) : PointXyz(valX, valY, valZ), curvature(valCurvature) { }
    Normal(const Normal& normal) : PointXyz(normal.toPointXyz()), curvature(normal.curvature) { }

    PointXyz& toPointXyz(void) { return *reinterpret_cast<PointXyz*>(this); }
    const PointXyz& toPointXyz(void) const { return *reinterpret_cast<const PointXyz*>(this); }

    float curvature;
};

struct _Rgb
{
    _Rgb(void) { }
    _Rgb(unsigned char valR, unsigned char valG, unsigned char valB) : r(valR), g(valG), b(valB) { }
    _Rgb(const _Rgb& rgb) : r(rgb.r), g(rgb.g), b(rgb.b) { }

    _Rgb& operator=(const _Rgb& rgb)
    {
        r = rgb.r;
        g = rgb.g;
        b = rgb.b;

        return *this;
    }

    unsigned char r;
    unsigned char g;
    unsigned char b;
};

struct PointXyzRgb : public PointXyz, public _Rgb
{
    PointXyzRgb(void) : PointXyz(), _Rgb() { }
    PointXyzRgb(const PointXyzRgb& point) : PointXyz(point.x, point.y, point.z), _Rgb(point.r, point.g, point.z) { }
};

} // end namespace obvious

#endif
