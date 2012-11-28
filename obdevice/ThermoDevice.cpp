#include "ThermoDevice.h"
#include "thermo-cam.h"

#include <iostream>

namespace obvious {

ThermoDevice::ThermoDevice(const std::string& configFile)
    : _camera(new ThermoCam(configFile))
{

}

ThermoDevice::~ThermoDevice(void)
{
    delete _camera;
}

bool ThermoDevice::grab(void)
{
    _camera->grab();
    _rgb = _camera->getMatRGB();

    return true;
}

} // end namespace obvious
