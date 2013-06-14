#include "OpenNI2Device.h"

#include <iostream>

namespace obvious {

OpenNI2Device::OpenNI2Device(const std::string& deviceURI)
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

    if (!_depth.isValid() || !_color.isValid())
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

    if (_depth.isValid() && _color.isValid())
    {
        depthVideoMode = _depth.getVideoMode();
        colorVideoMode = _color.getVideoMode();

        int depthWidth = depthVideoMode.getResolutionX();
        int depthHeight = depthVideoMode.getResolutionY();
        int colorWidth = colorVideoMode.getResolutionX();
        int colorHeight = colorVideoMode.getResolutionY();

        if (depthWidth == colorWidth && depthHeight == colorHeight)
        {
            _width = depthWidth;
            _height = depthHeight;
            _z.resize(_width * _height);
            _coords.resize(_width * _height * 3);
            _rgb = MatRGB(_height, _width);

            return true;
        }

        std::cout << "Error - expect color and depth to be in same resolution: D: "
                  << depthWidth << ", " << depthHeight << ", C: "
                  << colorWidth << ", " << colorHeight << std::endl;

        return false;
    }
    else if (_depth.isValid())
    {
        depthVideoMode = _depth.getVideoMode();
        _width = depthVideoMode.getResolutionX();
        _height = depthVideoMode.getResolutionY();
        _z.resize(_width * _height);
        _coords.resize(_width * _height * 3);

        return true;
    }
    else if (_color.isValid())
    {
        colorVideoMode = _color.getVideoMode();
        _width = colorVideoMode.getResolutionX();
        _height = colorVideoMode.getResolutionY();
        _rgb = MatRGB(_height, _width);

        return true;
    }

    std::cout << "Error - expects at least one of the streams to be valid..." << std::endl;
    return false;
}

bool OpenNI2Device::grab(void)
{
    if (!_depth.isValid() && !_color.isValid())
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
        MatRGB::iterator red  (_rgb.begin(MatRGB::Red  ));
        MatRGB::iterator green(_rgb.begin(MatRGB::Green));
        MatRGB::iterator blue (_rgb.begin(MatRGB::Blue ));

        for (int i = 0; i < size; ++i, ++data, ++red, ++green, ++blue)
        {
            *red   = data->r;
            *green = data->g;
            *blue  = data->b;
        }
    }

    return true;
}

} // end namespace obvious
