#include "obcore/base/PointCloud.h"
#include "obcore/base/Point.h"

namespace obvious {

/* Implementation for point type Point. */
template <>
PointCloud<Point>::PointCloud(const std::size_t size)
    : _width(size),
      _height(1),
      _points(size)
{

}

template <>
PointCloud<Point>::PointCloud(const PointCloud& cloud)
    : _width(cloud._width),
      _height(cloud._height),
      _points(cloud._points)
{

}

template <>
PointCloud<Point>::PointCloud(const unsigned int width, const unsigned int height)
    : _width(width),
      _height(height),
      _points(width * width)
{

}


/* Implementation for point type PointRgb. */
template <>
PointCloud<PointRgb>::PointCloud(const std::size_t size)
    : _width(size),
      _height(1),
      _points(size)
{

}

template <>
PointCloud<PointRgb>::PointCloud(const PointCloud& cloud)
    : _width(cloud._width),
      _height(cloud._height),
      _points(cloud._points)
{

}

template <>
PointCloud<PointRgb>::PointCloud(const unsigned int width, const unsigned int height)
    : _width(width),
      _height(height),
      _points(width * height)
{

}

} // end namespace obvious
