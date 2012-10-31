#ifndef __KINECT_THERMO_DEVICE__
#define __KINECT_THERMO_DEVICE__

#include "KinectDevice.h"
#include "ThermoDevice.h"
#include "MatD.h"
#include "MatRGB.h"

namespace xmlpp {
class Node;
}

namespace obvious {

class KinectThermoDevice : public KinectDevice, public ThermoDevice
{
public:
    KinectThermoDevice(const std::string& configKinect, const std::string& configThermo, const xmlpp::Node* calibration);
    virtual ~KinectThermoDevice(void);

    virtual bool grab(void);
    MatRGB imageKinect(void) const { return KinectDevice::image(); }
    MatRGB imageThermo(void) const { return ThermoDevice::image(); }

private:
    MatD m_R;
    MatD m_T;
    MatD m_cameraMatrix;
};

} // end namespace obvious

#endif
