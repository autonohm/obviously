#include "PointCloud.h"

namespace obvious {

PointCloud::PointCloud(void)
    : _type(None),
      _coords(0),
      _rgb(0)
{

}

PointCloud::PointCloud(const Type type, const size_t size)
    : _type(type),
      _coords(new Matrix(3, size)),
      _rgb(type & Rgb ? new uchar[3 * sizeof(uchar)] : 0),
      _thermo(type & Thermo ? size : 0),
      _intensity(type & Intensity ? size : 0)
{

}

PointCloud::PointCloud(const PointCloud& cloud, const std::vector<bool>& mask, const Type type)
    : _type(type),
      _coords(0),
      _rgb(0)
{
    unsigned int size = 0;

    for (std::vector<bool>::const_iterator it = mask.begin(); it < mask.end(); ++it)
        if (*it) ++size;

    _coords = new Matrix(3, size);

    if (type & Rgb)
        new uchar[3 * sizeof(uchar)];
    if (type & Thermo)
        _thermo.resize(size);
    if (type & Intensity)
        _intensity.resize(size);
}

PointCloud::~PointCloud(void)
{
    delete _coords;
    delete _rgb;
}

} // end namespace obvious
