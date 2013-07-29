#ifndef __GPU_DEVICE__
#define __GPU_DEVICE__

namespace obvious { namespace gpu {

class Device
{
public:
    static int getDeviceCount(void);
    static void printDevices(void);
};

} // end namespace gpu

} // end namespace obvious

#endif
