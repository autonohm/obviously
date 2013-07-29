#ifndef __GPU_PASS_THROUGH__
#define __GPU_PASS_THROUGH__

#include <limits>
#include <cstddef>

#include "obgpu/point-types.h"
#include "obgpu/PointCloud.h"
#include "obgpu/filter/Filter.h"

namespace obvious { namespace gpu { namespace filter {

class PassThrough : public Filter
{
public:
    PassThrough(void);
    virtual ~PassThrough(void);

    void setXAxis(const float min, const float max) { if (min < max) { _xMin = min; _xMax = max; } }
    void setYAxis(const float min, const float max) { if (min < max) { _yMin = min; _yMax = max; } }
    void setZAxis(const float min, const float max) { if (min < max) { _zMin = min; _zMax = max; } }

    virtual void filter(obvious::gpu::PointCloud* cloud);

private:
    float _xMin;
    float _xMax;
    float _yMin;
    float _yMax;
    float _zMin;
    float _zMax;

    float* _limits;
};

} // end namespace filter

} // end namespace gpu

} // end namespace obvious

#endif
