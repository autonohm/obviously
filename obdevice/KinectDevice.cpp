#include "KinectDevice.h"
#include "Kinect.h"

#include <iostream>

namespace obvious {

KinectDevice::KinectDevice(const std::string& configFile)
       : _kinect(new Kinect(configFile.c_str()))
{

}

KinectDevice::~KinectDevice(void)
{
    delete _kinect;
}

bool KinectDevice::grab(void)
{
    if (!_kinect->grab())
    {
        std::cout << __PRETTY_FUNCTION__  << std::endl;
        std::cout << "Can't grab Kinect!" << std::endl;
        throw "Can't grab Kinect!";

        return false;
    }

    _rgb = _kinect->getMatRGB();
    const double* data = _kinect->getCoords();
    _coords.resize(_kinect->getRows() * _kinect->getCols());

    for (unsigned int i = 0; i < _coords.size(); i++, data += 3)
        _coords[i] = Point3D(data[0], data[1], data[2]);

    return true;
}

} // end namespace obvious
