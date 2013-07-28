#ifndef __POINT_CLOUD__
#define __POINT_CLOUD__

#include <vector>
#include <memory>

namespace obvious {

template <typename T>
class PointCloud
{
public:

    typedef typename std::vector<T>::iterator iterator;
    typedef typename std::vector<T>::const_iterator const_iterator;

    PointCloud(void)
    : _points(new std::vector<T>()),
      _width(0),
      _height(0)
    {

    }

    PointCloud(const unsigned int size, const T& init = T())
        : _points(new std::vector<T>(size, init)),
          _width(0),
          _height(1)
    {

    }

    PointCloud(const unsigned int width, const unsigned int height, const T& init = T())
        : _points(new std::vector<T>(width * height, init)),
          _width(width),
          _height(height)
    {

    }

    PointCloud(PointCloud<T>& cloud)
        : _points(cloud._points),
          _width(cloud._width),
          _height(cloud._height)
    {

    }

//    ~PointCloud(void);

    inline iterator begin(void) { return _points.begin(); }
    inline const_iterator begin(void) const { return _points.begin(); }
    inline iterator end(void) { return _points.end(); }
    inline const_iterator end(void) const { return _points.end(); }
    inline unsigned int size(void) const { return _points.size(); }
    inline void clear(void) { _points.clear(); _width = 0; _height = 0; }
    inline void resize(const size_t size) { _points.resize(size); _width = 0; _height = 0; }
    inline std::vector<T>& points(void) { return _points; }
    inline const std::vector<T>& points(void) const { return _points; }

    PointCloud<T>& operator=(PointCloud<T>& cloud)
    {
        _points = cloud._points;
        _width = cloud._width;
        _height = cloud._height;

        return *this;
    }

private:
    std::vector<T> _points;
    unsigned int _width;
    unsigned int _height;
};

}

#endif
