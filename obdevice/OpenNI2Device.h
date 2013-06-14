#ifndef __OPEN_NI2_DEVICE__
#define __OPEN_NI2_DEVICE__

#include <OpenNI.h>

#include <string>
#include <vector>

#include "obcore/math/MatRGB.h"

namespace obvious {

class OpenNI2Device
{
public:
    OpenNI2Device(const std::string& deviceURI = std::string());
    ~OpenNI2Device(void);

    bool init(void);
    bool grab(void);
    int width(void) const { return _width; }
    int height(void) const { return _height; }
    const std::vector<float>& z(void) const { return _z; }
    const std::vector<float>& coords(void) const { return _coords; }
    const MatRGB& image(void) const { return _rgb; }

private:
    openni::Status _status;
    openni::Device _device;
    openni::VideoStream _depth;
    openni::VideoStream _color;
    openni::VideoFrameRef _frameDepth;
    openni::VideoFrameRef _frameColor;

    int _width;
    int _height;
    std::vector<float> _z;
    std::vector<float> _coords;
    MatRGB _rgb;
};

} // end namespace obvious

#endif
