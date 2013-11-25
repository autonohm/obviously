#include "PointCloud.h"

#include <cstring>

namespace obvious {

PointCloud::PointCloud(void)
    : _type(None),
      _size(0),
      _coords(0),
      _rgb(0)
{

}

PointCloud::PointCloud(const int type, const size_t size)
    : _type(type),
      _size(size),
      _coords(type & Coord ? new Matrix(3, size) : 0),
      _rgb(type & Rgb ? new unsigned char[3 * size * sizeof(unsigned char)] : 0),
      _thermo(type & Thermo ? size : 0),
      _intensity(type & Intensity ? size : 0)
{

}

PointCloud::PointCloud(const PointCloud& cloud, const int type)
    : _type(cloud._type & type),
      _size(cloud._size),
      _coords(_type & Coord ? new Matrix(3, cloud._size) : 0),
      _rgb(_type & Rgb ? new unsigned char[3 * cloud._size * sizeof(unsigned char)] : 0),
      _thermo(_type & Thermo ? cloud._size : 0),
      _intensity(_type & Intensity ? cloud._size : 0)
{
    this->copy(cloud, type);
}

PointCloud::PointCloud(const PointCloud& cloud, const Mask& mask, const int type)
    : _type(cloud._type & type),
      _size(mask.valid),
      _coords(_type & Coord ? new Matrix(3, mask.valid) : 0),
      _rgb(_type & Rgb ? new unsigned char[3 * mask.valid * sizeof(unsigned char)] : 0),
      _thermo(_type & Thermo ? mask.valid : 0),
      _intensity(_type & Intensity ? mask.valid : 0)
{
    this->copy(cloud, mask, _type);
}

PointCloud::~PointCloud(void)
{
    delete _coords;
    delete _rgb;
}

void PointCloud::copy(const PointCloud& cloud, const int type)
{
    if (type & Coord)
    {
        _coords = cloud._coords;
    }
    if (type & Rgb)
    {
        std::memcpy(_rgb, cloud._rgb, 3 * _size * sizeof(unsigned char));
    }
    if (type & Thermo)
    {
        _thermo = cloud._thermo;
    }
    if (type & Intensity)
    {
        _intensity = cloud._intensity;
    }
}

void PointCloud::copy(const PointCloud& cloud, const Mask& mask, const int type)
{
    if (type & Coord)
    {
        unsigned int i = 0;
        unsigned int j = 0;

        for (std::vector<bool>::const_iterator it = mask.mask.begin(); it < mask.mask.end(); ++it)
        {
            if (*it)
            {
                *_coords[0][i] = *cloud._coords[0][j];
                *_coords[1][i] = *cloud._coords[1][j];
                *_coords[2][i] = *cloud._coords[2][j];
            }
        }
    }
    if (type & Rgb)
    {
        unsigned char* rgb = _rgb;
        unsigned char* rgbCloud = cloud._rgb;

        for (std::vector<bool>::const_iterator it = mask.mask.begin(); it < mask.mask.end(); ++it)
        {
            if (*it)
            {
                *rgb++ = *rgbCloud++;
                *rgb++ = *rgbCloud++;
                *rgb++ = *rgbCloud++;
            }
            else
            {
                rgbCloud++;
            }
        }
    }
    if (type & Thermo)
    {
        std::vector<uint16_t>::iterator t(_thermo.begin());
        std::vector<uint16_t>::const_iterator tCloud(cloud._thermo.begin());

        for (std::vector<bool>::const_iterator it = mask.mask.begin(); it < mask.mask.end(); ++it, ++tCloud)
        {
            if (*it)
            {
                *t = *tCloud;
                ++t;
            }
        }
    }
    if (type & Intensity)
    {
        std::vector<uint16_t>::iterator i(_intensity.begin());
        std::vector<uint16_t>::const_iterator iCloud(cloud._intensity.begin());

        for (std::vector<bool>::const_iterator it = mask.mask.begin(); it < mask.mask.end(); ++it, ++iCloud)
        {
            if (*it)
            {
                *i = *iCloud;
                ++i;
            }
        }
    }
}

} // end namespace obvious















