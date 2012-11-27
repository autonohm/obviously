#include "CloudFactory.h"
#include "Kinect.h"
#include "Cloud.h"
#include "thermo-cam.h"

#include <iostream>

namespace obvious {

CloudFactory::CloudFactory(Kinect* kinect, ThermoCam* thermo)
    : _kinect(kinect),
      _thermo(thermo)
{

}

Cloud* CloudFactory::build(void)
{
    if (!_kinect || !_thermo || !_kinect->grab())
        return 0;

    _thermo->grab();
    const unsigned int size = _kinect->getRows() * _kinect->getCols();

    if (!this->validParameter())
    {
        return new Cloud(size, _kinect->getCoords(), _kinect->getRGB());
    }

    std::vector<cv::Point3f> points;
    const double* coords = _kinect->getCoords();

    for (unsigned int i = 0; i < size; i++)
    {
        cv::Point3f point;
        point.x = *coords++;
        point.y = *coords++;
        point.z = *coords++;
        points.push_back(point);
    }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(points, _r, _t, _intrinsic, _distCoeffs, imagePoints);
    std::vector<Point3D> validPoints;
    std::vector<RGBColor> validColors;
    MatRGB rgb = _thermo->getMatRGB();

    for (unsigned int i = 0; i < imagePoints.size(); i++)
    {
        if (imagePoints[i].x <= 300.0 && imagePoints[i].x >= 0 && imagePoints[i].y <= 280.0 && imagePoints[i].y >= 0)
        {
            validPoints.push_back(Point3D(points[i].x, points[i].y));
//            qDebug() << "x = " << static_cast<unsigned int>(imagePoints[i].x);
//            qDebug() << "y = " << static_cast<unsigned int>(imagePoints[i].y);
            validColors.push_back(rgb.rgb(static_cast<unsigned int>(imagePoints[i].y), static_cast<unsigned int>(imagePoints[i].x)));
        }
    }

    return new Cloud(validPoints, validColors);
}

void CloudFactory::setThermoParameter(const cv::Mat intrinsic, const cv::Mat distCoeffs, const cv::Mat r, const cv::Mat t)
{
    intrinsic.copyTo(_intrinsic);
    distCoeffs.copyTo(_distCoeffs);
    r.copyTo(_r);
    t.copyTo(_t);
}

bool CloudFactory::validParameter(void) const
{
    return _intrinsic.cols == 3 && _intrinsic.rows == 3 && _distCoeffs.rows == 5 && _distCoeffs.cols == 1 &&
        _r.cols == 3 && _r.rows == 1 && _t.cols == 3 && _t.rows == 1;
}

} // end namespace obvious
