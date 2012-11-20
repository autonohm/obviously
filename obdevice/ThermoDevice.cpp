#include "ThermoDevice.h"
#include "thermo-cam.h"

#include <iostream>

namespace obvious {

ThermoDevice::ThermoDevice(const std::string& configFile)
    : Device2D("Thermocam"),
      m_camera(new ThermoCam(configFile))
{

}

ThermoDevice::~ThermoDevice(void)
{
    delete m_camera;
}

bool ThermoDevice::grab(void)
{
    m_camera->grab();
    m_rgb = m_camera->getMatRGB();

    return true;
}

} // end namespace obvious
