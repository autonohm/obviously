#ifndef __GPU_DEVICE__
#define __GPU_DEVICE__

#include "obgpu/PointCloud.h"

namespace obvious { namespace gpu {

class Device
{
public:
    Device(void);
    virtual ~Device(void);

    static int getDeviceCount(void);
    static void printDevices(void);

    void setSource(gpu::PointCloud* cloud);
    const std::vector<unsigned int>& inlier(void);

protected:
    void downloadInlier(void);

    gpu::PointCloud* d_source;
    std::vector<unsigned int> h_inlier;
    bool* d_inlier;
    bool h_sync;
};

} // end namespace gpu

} // end namespace obvious

#endif
