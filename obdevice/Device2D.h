#ifndef __DEVICE_2D__
#define __DEVICE_2D__

#include "MatRGB.h"

namespace obvious {

class Device2D
{
public:
    virtual ~Device2D(void) { }

    virtual bool grab(void) = 0;

    MatRGB image(void) const { return _rgb; }

protected:
    MatRGB _rgb;
};

} // end namespace obvious


#endif
