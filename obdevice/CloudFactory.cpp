#include "CloudFactory.h"
#include "obdevice/KinectDevice.h"
#include "obcore/base/Cloud.h"
#include "obdevice/ThermoDevice.h"

#include <iostream>

namespace obvious {

CloudFactory::CloudFactory(KinectDevice* kinect, ThermoDevice* thermo)
    : _kinect(kinect),
      _thermo(thermo)
{

}

Cloud* CloudFactory::build(void)
{
    if (!_kinect || !_thermo || !_kinect->grab())
        return 0;

    _thermo->grab();

    if (!this->validParameter())
    {
        std::cout << __PRETTY_FUNCTION__ << "!validParameter()" << std::endl;

        const MatRGB& image = _kinect->image();
        std::vector<RGBColor> colors(image.rows() * image.cols());

        for (unsigned int row = 0, i = 0; row < image.rows(); row++)
            for (unsigned int col = 0; col < image.cols(); col++, i++)
                colors[i] = image.rgb(row, col);

        return new Cloud(_kinect->coords(), colors);
    }

    const std::vector<Point3D>& coords = _kinect->coords();



    std::vector<Point3D> validPoints;
    std::vector<RGBColor> validColors;
    MatRGB rgb = _thermo->image();

    for (unsigned int i = 0; i < imagePoints.size(); i++)
    {
        if (imagePoints[i].x <= static_cast<double>(rgb.cols() - 1) && imagePoints[i].x >= 0 && imagePoints[i].y <= static_cast<double>(rgb.rows() - 1) && imagePoints[i].y >= 0)
        {
            validPoints.push_back(Point3D(points[i].x, points[i].y, points[i].z));
            validColors.push_back(rgb.rgb(static_cast<unsigned int>(imagePoints[i].y), static_cast<unsigned int>(imagePoints[i].x)));
        }
    }

    return new Cloud(validPoints, validColors);
}

bool CloudFactory::validParameter(void) const
{
    return _thermo->calibration();
}

} // end namespace obvious
