#include "OpenNI2Device.h"

#include <iostream>

namespace obvious {

OpenNI2Device::OpenNI2Device(const Flag flags, const std::string& deviceURI)
   : _flags(flags)
{
    _status = openni::OpenNI::initialize();

    std::cout << "After initialization:" << std::endl << openni::OpenNI::getExtendedError();

    if ((_status = _device.open(deviceURI.length() ? deviceURI.c_str() : openni::ANY_DEVICE)) != openni::STATUS_OK)
    {
        std::cout << "Device " << deviceURI << " open failed:" << std::endl << openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        return;
    }
    else
    {
        std::cout << "Device " << deviceURI << " opened." << std::endl;
    }

    if (_flags & Depth)
    {
        if ((_status = _depth.create(_device, openni::SENSOR_DEPTH)) == openni::STATUS_OK)
        {
            std::cout << "Found depth stream." << std::endl;

            if ((_status = _depth.start()) != openni::STATUS_OK)
            {
                std::cout << "Couldn't start depth stream:" << std::endl << openni::OpenNI::getExtendedError();
                openni::OpenNI::shutdown();
                return;
            }

            std::cout << "Depth stream started." << std::endl;
        }
        else
        {
            std::cout << "Couldn't find depth stream:" << std::endl << openni::OpenNI::getExtendedError();
        }
    }

    if (_flags & Color)
    {
        if ((_status = _color.create(_device, openni::SENSOR_COLOR)) == openni::STATUS_OK)
        {
            std::cout << "Found color stream." << std::endl;

            if ((_status = _color.start()) != openni::STATUS_OK)
            {
                std::cout << "Couldn't start color stream:" << std::endl << openni::OpenNI::getExtendedError();
                openni::OpenNI::shutdown();
                return;
            }

            std::cout << "Color stream started." << std::endl;
        }
        else
        {
            std::cout << "Couldn't find any color stream:" << std::endl << openni::OpenNI::getExtendedError();
        }
    }

    if (_flags & Ir)
    {
        if ((_status = _ir.create(_device, openni::SENSOR_IR)) == openni::STATUS_OK)
        {
            std::cout << "Found ir stream." << std::endl;

            if ((_status = _ir.start()) != openni::STATUS_OK)
            {
                std::cout << "Couldn't start ir stream:" << std::endl << openni::OpenNI::getExtendedError();
                openni::OpenNI::shutdown();
                return;
            }

            std::cout << "Ir stream started." << std::endl;
        }
        else
        {
            std::cout << "Couldn't find any ir stream:" << std::endl << openni::OpenNI::getExtendedError();
        }
    }

    if (!_depth.isValid() && !_color.isValid() && !_ir.isValid())
    {
        std::cout << "Device " << deviceURI << ": no valid streams." << std::endl;
        openni::OpenNI::shutdown();
        return;
    }
}

OpenNI2Device::~OpenNI2Device(void)
{
    if (_status == openni::STATUS_OK)
        openni::OpenNI::shutdown();
}

bool OpenNI2Device::init(void)
{
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;
    openni::VideoMode irVideoMode;

    if (_depth.isValid() && _color.isValid() && _ir.isValid())
    {
        depthVideoMode = _depth.getVideoMode();
        colorVideoMode = _color.getVideoMode();
        irVideoMode    = _ir.getVideoMode();

        const int depthWidth = depthVideoMode.getResolutionX();
        const int depthHeight = depthVideoMode.getResolutionY();
        const int colorWidth = colorVideoMode.getResolutionX();
        const int colorHeight = colorVideoMode.getResolutionY();
        const int irWidth = irVideoMode.getResolutionX();
        const int irHeight = irVideoMode.getResolutionY();

        if (depthWidth == colorWidth && colorWidth == irWidth && depthHeight == colorHeight && colorHeight == irHeight)
        {
            _width = depthWidth;
            _height = depthHeight;
            _z.resize(_width * _height);
            _coords.resize(_width * _height * 3);
            _imgRgb = MatRGB(_height, _width);
            _imgIr = MatRGB(_height, _width);

            if (_flags == Any)
                _flags = static_cast<Flag>(Depth | Color | Ir);

            return true;
        }

        std::cout << "Error - expect color and depth to be in same resolution: D: "
                  << depthWidth << ", " << depthHeight << ", C: "
                  << colorWidth << ", " << colorHeight << std::endl;

        return false;
    }
    else if (_depth.isValid() && _ir.isValid())
    {
        depthVideoMode = _depth.getVideoMode();
        irVideoMode = _ir.getVideoMode();

        _width = depthVideoMode.getResolutionX();
        _height = depthVideoMode.getResolutionY();
        _z.resize(_width * _height);
        _coords.resize(_width * _height * 3);
        _imgIr = MatRGB(_height, _width);

        if (_flags == Any)
            _flags = static_cast<Flag>(Depth | Ir);

        return true;
    }
    else if (_color.isValid())
    {
        colorVideoMode = _color.getVideoMode();
        _width = colorVideoMode.getResolutionX();
        _height = colorVideoMode.getResolutionY();
        _imgRgb = MatRGB(_height, _width);

        if (_flags == Any)
            _flags = Color;

        return true;
    }

    std::cout << "Error - expects at least one of the streams to be valid..." << std::endl;
    return false;
}

bool OpenNI2Device::grab(void)
{
    if (!_depth.isValid() && !_color.isValid() && !_ir.isValid())
    {
        std::cout << "No stream available." << std::endl;
        return false;
    }

    if (_depth.isValid())
    {
        _depth.readFrame(&_frameDepth);

        const openni::DepthPixel* data = reinterpret_cast<const openni::DepthPixel*>(_frameDepth.getData());
        std::vector<float>::iterator itZ(_z.begin());
        std::vector<float>::iterator itCoords(_coords.begin());

        for (int row = 0; row < _height; ++row)
        {
            for (int col = 0; col < _width; ++col, ++itZ, ++data)
            {
                float x;
                float y;
                float z;

                openni::CoordinateConverter::convertDepthToWorld(_depth, col, row, *data, &x, &y, &z);
                *itZ = *data * 0.001;
                *itCoords++ = x * 0.001;
                *itCoords++ = y * 0.001;
                *itCoords++ = z * 0.001;
            }
        }
    }

    if (_color.isValid())
    {
        _color.readFrame(&_frameColor);

        const openni::RGB888Pixel* data = reinterpret_cast<const openni::RGB888Pixel*>(_frameColor.getData());
        const int size = _width * _height;
        MatRGB::iterator red  (_imgRgb.begin(MatRGB::Red  ));
        MatRGB::iterator green(_imgRgb.begin(MatRGB::Green));
        MatRGB::iterator blue (_imgRgb.begin(MatRGB::Blue ));

        for (int i = 0; i < size; ++i, ++data, ++red, ++green, ++blue)
        {
            *red   = data->r;
            *green = data->g;
            *blue  = data->b;
        }
    }

    if (_ir.isValid())
    {
        _ir.readFrame(&_frameIr);

        const uint16_t* data = reinterpret_cast<const uint16_t*>(_frameIr.getData());
        const int size = _width * _height;
        MatRGB::iterator red  (_imgIr.begin(MatRGB::Red  ));
        MatRGB::iterator green(_imgIr.begin(MatRGB::Green));
        MatRGB::iterator blue (_imgIr.begin(MatRGB::Blue ));

        for (int i = 0; i < size; ++i, ++data, ++red, ++green, ++blue)
        {
            *red   = *data >> 2;
            *green = *data >> 2;
            *blue  = *data >> 2;
        }
    }

    return true;
}

} // end namespace obvious
