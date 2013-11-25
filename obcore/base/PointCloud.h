#ifndef __POINT_CLOUD_H__
#define __POINT_CLOUD_H__

#include "obcore/math/linalg/linalg.h"

#include <vector>
#include <stdint.h>

namespace obvious {

class PointCloud
{
public:

    struct Mask {
        std::vector<bool> mask;
        unsigned int valid;
    };

    enum Type {
        None      = 0,
        Coord     = (1 << 0),
        Rgb       = (1 << 1),
        Thermo    = (1 << 2),
        Intensity = (1 << 3),
        All       = 0xff
    };

    PointCloud(void);
    PointCloud(const int type, const size_t size);
    PointCloud(const PointCloud& cloud, const int type = All);
    PointCloud(const PointCloud& cloud, const Mask& mask, const int type = All);
    PointCloud(const PointCloud& cloud, const std::vector<int>& indices, const int type = All);
    ~PointCloud(void);

private:
    void copy(const PointCloud& cloud, const int type = All);
    void copy(const PointCloud& cloud, const Mask& mask, const int type = All);

    uint8_t _type;
    size_t _size;
    Matrix* _coords;
    unsigned char* _rgb;
    std::vector<uint16_t> _thermo;
    std::vector<uint16_t> _intensity;
};

} // end namespace obvious

#endif
