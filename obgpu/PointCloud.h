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
        XYZRGBIT = XYZ | RGB | T | I,
        NORMAL = 1 << 4
    };

    PointCloud(void) : _data(0), _type(None), _size(0) { }
    PointCloud(const PointCloud& cloud, const bool* inlier);
    ~PointCloud(void);

    void upload(const obvious::PointCloud<obvious::PointXyz>& cloud);
    void upload(const obvious::PointCloud<obvious::Normal>& normals);
    void download(obvious::PointCloud<obvious::PointXyz>& cloud);
    void download(obvious::PointCloud<obvious::Normal>& normals);

    inline void* data(void) { return _data; }
    inline size_t size(void) const { return _size; }
    inline int type(void) const { return _type; }
    inline bool isNormal(void) const { return _type & NORMAL; }

private:
    void* _data;
    Type _type;
    unsigned int _size;
};

} // end namespace gpu

} // end namespace obvious

#endif
