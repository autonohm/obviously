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

} // end namespace obvious
