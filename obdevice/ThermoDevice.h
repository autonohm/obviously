#ifndef __THREMO_DEVICE__
#define __THERMO_DEVICE__

#include "Device2D.h"

class ThermoCam;

namespace obvious {

class ThermoDevice : public Device2D
{
public:
    ThermoDevice(const std::string& configFile);
    virtual ~ThermoDevice(void);

    virtual bool grab(void);

private:
    ThermoCam* _camera;
};

} // end namespace obvious

#endif
