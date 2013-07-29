#ifndef __GPU_FILTER__
#define __GPU_FILTER__

#include "obgpu/PointCloud.h"

namespace obvious { namespace gpu { namespace filter {

class Filter
{
public:
    Filter(void) : _input(0) { }
    virtual ~Filter(void) { }

    void setInput(obvious::gpu::PointCloud* cloud) { _input = cloud; }
    virtual void filter(obvious::gpu::PointCloud* cloud) = 0;

protected:
    obvious::gpu::PointCloud* _input;
};

} // end namespace filter

} // end namespace gpu

} // end namespace obvious

#endif
