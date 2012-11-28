#ifndef __DEVICE_3D__
#define __DEVICE_3D__

#include "obcore/Point3D.h"

#include <vector>

namespace obvious {

class Device3D
{
public:
    virtual ~Device3D(void);

    virtual bool grab(void) = 0;

    const std::vector<Point3D>& coords(void) const { return _coords; }

protected:
    void deletePoints(void);

    std::vector<Point3D> _coords;
};

} // end namespace obvious

#endif
