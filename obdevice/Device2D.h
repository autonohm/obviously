#ifndef __DEVICE_2D__
#define __DEVICE_2D__

#include "obcore/math/MatRGB.h"
#include "obdevice/CameraCalibration.h"

namespace xmlpp {
class Node;
}

namespace obvious {

class Device2D
{
public:
    Device2D(const xmlpp::Node* node = 0) : _cali(!node ? 0 : new CameraCalibration(node)) { }
    virtual ~Device2D(void) { delete _cali; }

    virtual bool grab(void) = 0;

    const MatRGB& image(void) const { return _rgb; }
    const CameraCalibration* calibration(void) const { return _cali; }
    void setCalibration(CameraCalibration* cali) { _cali = cali; }

protected:
    MatRGB _rgb;
    CameraCalibration* _cali;
};

} // end namespace obvious


#endif
