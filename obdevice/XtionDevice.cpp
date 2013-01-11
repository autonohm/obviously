/**
* @file XtionDevice.cpp
* @autor christian
* @date  07.01.2013
*
*
*/

#include "XtionDevice.h"
#include "Xtion.h"

#include <iostream>

namespace obvious {

XtionDevice::XtionDevice(const char* configFile)
       : _xtion(new Xtion(configFile))
{
  std::cout << "build XtionDevice" << std::endl;
}

XtionDevice::~XtionDevice(void)
{
    delete _xtion;
}

bool XtionDevice::grab(void)
{
    if (!_xtion->grab())
    {
        std::cout << __PRETTY_FUNCTION__  << std::endl;
        std::cout << "Can't grab Xtion!" << std::endl;
        throw "Can't grab Xtion!";

        return false;
    }

    const double* data = _xtion->getCoords();
    _coords.resize(_xtion->getRows() * _xtion->getCols());

    for (unsigned int i = 0; i < _coords.size(); i++, data += 3)
        _coords[i] = Point3D(data[0], data[1], data[2]);

    return true;
}

double* XtionDevice::getCoords(void)
{
  return(_xtion->getCoords());
}

unsigned int XtionDevice::getRows(void)
{
  return(_xtion->getRows());
}

unsigned int XtionDevice::getCols(void)
{
  return(_xtion->getCols());
}

} // end namespace obvious





