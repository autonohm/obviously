#include "KinectDevice.h"
#include "Kinect.h"
#include "RgbPoint3D.h"

#include <iostream>

namespace obvious {

KinectDevice::KinectDevice(const std::string& configFile)
    : Device3D("Kinect"),
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

    const MatRGB  rgb  = m_kinect->getMat();
    std::cout << "MatRGB: cols = " << rgb.cols() << ", rows = " << rgb.rows() << std::endl;
    const double* data = m_kinect->getCoords();

    for (unsigned int row = 0, i = 0; row < rgb.rows(); row++)
        for (unsigned int col = 0; col < rgb.cols(); col++, i += 3)
            m_points.push_back(new RgbPoint3D(data[i], data[i + 1], data[i + 2], rgb.rgb(col, row)));

    return true;
}

} // end namespace obvious
