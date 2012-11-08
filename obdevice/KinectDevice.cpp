#include "KinectDevice.h"
#include "Kinect.h"
#include "RgbPoint3D.h"

#include <iostream>

namespace obvious {

KinectDevice::KinectDevice(const std::string& configFile)
    : Device2D("Kinect RGB"),
      Device3D("Kinect XYZ"),
      m_kinect(new Kinect(configFile.c_str()))
{

}

KinectDevice::~KinectDevice(void)
{
    delete m_kinect;
}

bool KinectDevice::grab(void)
{
    this->deletePoints();

    if (!m_kinect->grab())
    {
        std::cout << __PRETTY_FUNCTION__  << std::endl;
        std::cout << "Can't grab Kinect!" << std::endl;
        throw "Can't grab Kinect!";

        return false;
    }

    m_rgb = m_kinect->getMat();
    const double* data = m_kinect->getCoords();

    for (unsigned int row = 0, i = 0; row < m_rgb.rows(); row++)
        for (unsigned int col = 0; col < m_rgb.cols(); col++, i += 3)
            m_points.push_back(new RgbPoint3D(data[i], data[i + 1], data[i + 2], m_rgb.rgb(row, col)));

    return true;
}

} // end namespace obvious
