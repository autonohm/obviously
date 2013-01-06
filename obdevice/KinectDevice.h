#ifndef __KINECT_DEVICE__
#define __KINECT_DEVICE__

#include "Device2D.h"
#include "Device3D.h"

#include <string>

namespace obvious {

class Kinect;

class KinectDevice : public Device2D, public Device3D
{
public:
    KinectDevice(const std::string& configFile);
    virtual ~KinectDevice(void);

    virtual bool grab(void);

private:
    Kinect* _kinect;
};

} // end namespace obvious

#endif
