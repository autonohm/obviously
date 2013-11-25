#ifndef __POINT_CLOUD_H__
#define __POINT_CLOUD_H__

#include "obcore/math/linalg/linalg.h"

namespace obvious {

class PointCloud
{
public:

    enum Type {
        None      = 0,
        Coords    = (1 << 0),
        Rgb       = (1 << 1),
        Thermo    = (1 << 2),
        Intensity = (1 << 3),
        All       = 0xff
    };

    PointCloud(void);
    PointCloud(const Type type, const size_t size);
    PointCloud(const PointCloud& cloud, const Type type = All);
    PointCloud(const PointCloud& cloud, const std::vector<bool>& mask, const Type type = All);
    PointCloud(const PointCloud& cloud, const std::vector<int>& indices, const Type type = All);
    ~PointCloud(void);

private:
    uint8_t _type;
    Matrix* _coords;
    uchar* _rgb;
    std::vector<uint16_t> _thermo;
    std::vector<uint16_t> _intensity;
};

} // end namespace obvious

#endif
