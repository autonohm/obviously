#include "Cloud.h"

namespace {
const unsigned int DIMENSIONS = 3;
}

namespace obvious {

Cloud::Cloud(const unsigned int size, const double* coords, const unsigned char* rgb)
    : _size(size),
      _points(size, DIMENSIONS),
      _rgb(rgb != 0 ? size : 0, 1)
{
    for (unsigned int row = 0; row < _points.rows(); row++)
        for (unsigned int col = 0; col < _points.cols(); col++, coords++)
            _points.at(row, col) = *coords;

    for (unsigned int row = 0; row < _rgb.rows(); row++)
    {
        _rgb.at(row, 0, MatRGB::Red)   = *rgb++;
        _rgb.at(row, 0, MatRGB::Green) = *rgb++;
        _rgb.at(row, 0, MatRGB::Blue)  = *rgb++;
    }
}

Cloud::Cloud(const std::vector<Point3D>& points, const std::vector<RGBColor>& color)
    : _size(points.size()),
      _points(points.size(), 3),
      _rgb(color.size(), 1)
{

    std::vector<Point3D>::const_iterator itP = points.begin();

    for (unsigned int row = 0; row < _points.rows(); row++, ++itP)
    {
        _points.at(row, X) = (*itP).x();
        _points.at(row, Y) = (*itP).y();
        _points.at(row, Z) = (*itP).z();
    }

    std::vector<RGBColor>::const_iterator itC = color.begin();

    for (unsigned int row = 0; row < _rgb.rows(); row++, ++itC)
        _rgb.setRgb(row, 0, *itC);
}

} // end namespace obvious
