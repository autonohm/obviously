#ifndef __GPU_HOST_POINT_CLOUD__
#define __GPU_HOST_POINT_CLOUD__

#include <stdint.h>

#include <cuda_runtime_api.h>

#include "obcore/base/point-types.h"
#include "obcore/base/PointCloud.h"

namespace obvious { namespace gpu { namespace host {

class HostPointCloud
{
public:

    enum Type {
        None = 0,
        XYZ = 1 << 0,
        RGB = 1 << 1,
        I = 1 << 2,
        T = 1 << 3,
        XYZRGB = XYZ | RGB,
        XYZI = XYZ | I,
        XYZT = XYZ | T,
        XYZRGBI = XYZ | RGB | I,
        XYZRGBT = XYZ | RGB | T,
        XYZRGBIT = XYZ | RGB | T | I
    };

    HostPointCloud(void) : _data(0), _type(None), _size(0) { }
    ~HostPointCloud(void);

    void upload(const obvious::PointCloud<obvious::PointXyz>& cloud);
    void download(obvious::PointCloud<obvious::PointXyz>& cloud);

private:
    void* _data;
    Type _type;
    unsigned int _size;
};

} // end namespace host

} // end namespace gpu

} // end namespace obvious

#endif
