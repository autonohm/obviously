#ifndef __CLOUD_FACTORY__
#define __CLOUD_FACTORY__

namespace obvious {

class KinectDevice;
class ThermoDevice;
class Cloud;

class CloudFactory
{
public:
    CloudFactory(KinectDevice* kinect, ThermoDevice* thermo);

    Cloud* build(void);

private:
    bool validParameter(void) const;

    KinectDevice* _kinect;
    ThermoDevice* _thermo;
};

} //end namespace obvious

#endif
