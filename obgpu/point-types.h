#ifndef __GPU_POINT_TYPES__
#define __GPU_POINT_TYPES__

#include <cstddef>

namespace obvious { namespace gpu {

struct PointXyz {
    float x;
    float y;
    float z;

    inline PointXyz operator-(const PointXyz& point)
    {
        PointXyz result;

        result.x = x - point.x;
        result.y = y - point.y;
        result.z = z - point.z;

        return result;
    }
};

} // end namespace gpu

} // end namespace obvious

#endif
