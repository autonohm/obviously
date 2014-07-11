#include "obcore/base/PointCloud.h"
#include "obcore/base/Point.h"

namespace obvious {


PointCloud<Point>::PointCloud(const std::size_t size)
    : _points(size)
{

}

} // end namespace obvious
