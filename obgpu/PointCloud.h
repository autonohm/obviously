#ifndef __GPU_POINT_CLOUD__
#define __GPU_POINT_CLOUD__

#include <stdint.h>

#include "obcore/base/point-types.h"
#include "obcore/base/PointCloud.h"

namespace obvious { namespace gpu {

class PointCloud
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

    PointCloud(void) : _data(0), _type(None), _size(0) { }
    ~PointCloud(void);

    void upload(const obvious::PointCloud<obvious::PointXyz>& cloud);
    void download(obvious::PointCloud<obvious::PointXyz>& cloud);

    inline void* data(void) { return _data; }
    inline size_t size(void) const { return _size; }

private:
    void* _data;
    Type _type;
    unsigned int _size;
};

} // end namespace gpu

} // end namespace obvious

#endif
