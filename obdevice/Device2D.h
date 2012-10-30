#ifndef __DEVICE_2D__
#define __DEVICE_2D__

#include "InputDevice.h"
#include "MatRGB.h"

namespace obvious {

class Device2D : public InputDevice
{
public:
    Device2D(const std::string& name) : InputDevice(name) { }
    virtual ~Device2D(void) { }

    MatRGB image(void) const { return m_rgb; }

protected:
    MatRGB m_rgb;
};

} // end namespace obvious


#endif
