#ifndef __OPEN_NI_DEVICE__
#define __OPEN_NI_DEVICE__

#include <OpenNI.h>

#include <string>
#include <vector>

//#include "obcore/math/MatRGB.h"

namespace obvious {

class OpenNiDevice
{
public:

    enum Flag {
        Any   = 0xff,
        Depth = 1,
        Color = 2,
        Ir    = 4,
        All   = Depth | Color | Ir
    };

    OpenNiDevice(const Flag flags = Any, const std::string& deviceURI = std::string());
    ~OpenNiDevice(void);

    Flag flags(void) const { return _flags; }
    bool init(void);
    bool grab(void);
    int width(void) const { return _width; }
    int height(void) const { return _height; }
    const std::vector<float>& z(void) const { return _z; }
    const std::vector<float>& coords(void) const { return _coords; }
//    const MatRGB& image(void) const { return _flags & Color ? _imgRgb : _imgIr; }
//    const MatRGB& ir(void) const { return _imgIr; }
//    const MatRGB& rgb(void) const { return _imgRgb; }

private:
    openni::Status _status;
    openni::Device _device;
    openni::VideoStream _depth;
    openni::VideoStream _color;
    openni::VideoStream _ir;
    openni::VideoFrameRef _frameDepth;
    openni::VideoFrameRef _frameColor;
    openni::VideoFrameRef _frameIr;

    Flag _flags;
    int _width;
    int _height;
    std::vector<float> _z;
    std::vector<float> _coords;
    unsigned char* _imgRgb;
    unsigned char* _imgIr;
//    MatRGB _imgRgb;
//    MatRGB _imgIr;
};

} // end namespace obvious

#endif
