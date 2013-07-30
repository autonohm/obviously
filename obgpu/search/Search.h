#ifndef __GPU_SEARCH__
#define __GPU_SEARCH__

#include "obgpu/point-types.h"
#include "obgpu/PointCloud.h"
#include "obgpu/Device.h"

#include <vector>

using namespace obvious;

namespace obvious { namespace gpu { namespace search {

class Search : public Device
{
public:
    Search(void) : Device() { }
    virtual ~Search(void);

//    virtual void radiusSearch(const float radius, const gpu::PointXyz point) = 0;
    virtual void radiusSearch(const float radius, const gpu::PointXyz* d_point) = 0;
};

} // end namespace search

} // end namespace gpu

} // end namespace obvious

#endif
