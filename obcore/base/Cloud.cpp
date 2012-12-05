#include "Cloud.h"

namespace obvious {

Cloud::Cloud(const unsigned int size, const double* coords, const unsigned char* rgb)
    : _coords(size),
      _rgb(rgb ? size : 0)
{
    for (std::vector<VecD>::iterator it = _coords.begin(); it < _coords.end(); ++it)
    {
        VecD vec(3);
        vec.at(X) = *coords++;
        vec.at(Y) = *coords++;
        vec.at(Z) = *coords++;
        (*it) = vec;
    }

    for (std::vector<RGBColor>::iterator it = _rgb.begin(); it < _rgb.end(); ++it)
    {
        (*it).setR(*rgb++);
        (*it).setG(*rgb++);
        (*it).setB(*rgb++);
    }
}

Cloud::Cloud(const std::vector<Point3D>& points, const std::vector<RGBColor>& color)
    : _coords(points.size(), 3),
      _rgb(color.size() ? points.size() : 0)
{
    std::vector<Point3D>::const_iterator itPoints = points.begin();

    for (std::vector<VecD>::iterator it = _coords.begin(); it < _coords.end(); ++it, ++itPoints)
    {
        VecD vec(3);
        vec.at(X) = (*itPoints).x();
        vec.at(Y) = (*itPoints).y();
        vec.at(Z) = (*itPoints).z();
        (*it) = vec;
    }

    _rgb = color;
}

} // end namespace obvious
