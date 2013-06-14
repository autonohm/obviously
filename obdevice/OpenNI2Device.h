#ifndef __OPEN_NI2_DEVICE__
#define __OPEN_NI2_DEVICE__

#include <OpenNI.h>

#include <string>
#include <vector>

class OpenNI2Device
{
public:
    OpenNI2Device(const std::string& deviceURI = std::string(openni::ANY_DEVICE));
    ~OpenNI2Device(void);

    bool init(void);
    bool grab(void);
    int width(void) const { return _width; }
    int height(void) const { return _height; }
    const std::vector<float>& z(void) const { return _z; }

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
};

#endif
