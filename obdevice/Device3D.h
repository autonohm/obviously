#ifndef __DEVICE_3D__
#define __DEVICE_3D__

#include "InputDevice.h"

#include <vector>

namespace obvious {

class Point3D;

class Device3D : public InputDevice
{
public:
    Device3D(const std::string& name) : InputDevice(name) { }
    virtual ~Device3D(void);

    const std::vector<Point3D*>& points(void) const { return m_points; }

protected:
    void deletePoints(void);

    std::vector<Point3D*> m_points;
};

} // end namespace obvious

#endif
