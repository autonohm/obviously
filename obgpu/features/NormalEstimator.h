#ifndef __GPU_NORMAL_ESTIMATOR__
#define __GPU_NORMAL_ESTIMATOR__

#include "obgpu/Device.h"

using namespace obvious;

namespace obvious { namespace gpu { namespace features {

class NormalEstimator : public Device
{
public:
    NormalEstimator(void) : Device(), h_searchRadius(0.0) { }

    void estimate(void);

private:
    float h_searchRadius;
};

} // end namespace features

} // end namespace gpu

} // end namespace obvious

#endif
