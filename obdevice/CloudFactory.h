#ifndef __CLOUD_FACTORY__
#define __CLOUD_FACTORY__

#include "obcore/math/MatD.h"

class ThermoCam;

namespace obvious {

class Kinect;
class Cloud;

class CloudFactory
{
public:
    CloudFactory(Kinect* kinect, ThermoCam* thermo);

    Cloud* build(void);

private:
    bool validParameter(void) const;

    Kinect* _kinect;
    ThermoCam* _thermo;

    MatD _intrinsic;
    MatD _distCoeffs;
    MatD _r;
    MatD _t;
};

} //end namespace obvious

#endif
