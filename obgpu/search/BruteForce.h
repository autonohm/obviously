#ifndef __GPU_BRUTE_FORCE__
#define __GPU_BRUTE_FORCE__

#include "obgpu/search/Search.h"

namespace obvious { namespace gpu { namespace search {

class BruteForce : public Search
{
public:
    BruteForce(void) { }

//    virtual void radiusSearch(const float radius, const gpu::PointXyz point);
    virtual void radiusSearch(const float radius, const gpu::PointXyz* d_point);
};

} // end namespace search

} // end namespace gpu

} // end namespace obvious

#endif
