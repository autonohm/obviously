#ifndef __KINECT_DEVICE__
#define __KINECT_DEVICE__

#include "Device3D.h"

namespace obvious {

class Kinect;

class KinectDevice : public Device3D
{
public:
    KinectDevice(const std::string& configFile);
    virtual ~KinectDevice(void);

    virtual bool grab(void);

private:
    Kinect* m_kinect;
};

} // end namespace obvious

#endif
