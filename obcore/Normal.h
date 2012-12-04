/**
* @file   Normal.h
* @author Christian Pfitzner
* @date   04.12.2012
*
* @todo   example application for normals
*/

#ifndef Normal_H_
#define Normal_H_

#include "obcore/Point3D.h"
#include "obcore/math/mathbase.h"

/**
 * @namespace of obviously library
 */
namespace obvious{

class Point3D;

const unsigned int L = 4;

/**
 * @class Normal  Class to generate normal objects derived from Point3D
 */
class Normal : public Point3D
{
public:
  /**
   * Standard constructor
   * @param     x   x value
   * @param     y   y value
   * @param     z   z value
   * @param     l   length
   */
  Normal(const double& x = 0, const double& y = 0, const double& z = 0, const double& l = 0)
    : Point3D(x, y, z),
      _length(l) { getLength(); }
  /**
   * Copy constructor
   * @param     n   normal object
   */
  Normal(const Normal& n)
    : Point3D(n._x, n._y, n._z),
      _length(n._length) { }
  /**
   * Overload of operator+=
   * @param[in]  right   Normal object
   * @return     ref to this
   */
  virtual Normal& operator+=(const Normal& right)
      { _x += right._x; _y += right._y; _z += right._z; _length = getLength(); return *this; }
  /**
   * Overload of operator-=
   * @param[in]  right   Normal object
   * @return     ref to this
   */
  virtual Normal& operator-=(const Normal& right)
      { _x -= right._x; _y -= right._y; _z -= right._z; _length = getLength(); return *this; }
  /**
   * Overload of allocation perator=
   * @param[in]  right   Normal object
   * @return     ref to this
   */
  virtual Normal& operator= (const Normal& right)
      { _x  = right._x; _y  = right._y; _z  = right._z; _length = getLength(); return *this; }
  /**
   * Overload of operator+
   * @param[in]  right   Normal object
   * @return     ref to this
   */
  virtual Normal  operator+ (const Normal& right)
      { return Normal(_x + right._x, _y + right._y, _z + right._z);  }
  /**
   * Overload of operator-
   * @param[in]  right   Normal object
   * @return     ref to this
   */
  virtual Normal  operator- (const Normal& right)
      { return Normal(_x - right._x, _y - right._y, _z + right._z);  }
  /**
  * Index operator [] to return ref on member for read and write access
  * @param[in]   idx     index of member @see Axis of Point3D
  */
  double& operator[](int idx)
  {
    if (idx == X)
    {
      return _x;
    }
    else if (idx == Y) return _y;
    else if (idx == Z) return _z;
    else if (idx == L) return _length;
  }
  /**
   * Function to calculate length of vector
   * @return    length of normal
   */
  const double& getLength(void)
  {
    _length   = sqrt(_x*_x + _y*_y + _z*_z);
    return _length;
  }
  /**
   * Function to get unit vector of normal
   * @return    length of normal (1)
   */
  const double& makeUnit(void)
  {
    getLength();
    _x = _x / _length;
    _y = _y / _length;
    _z = _z / _length;
    return(_length = 1.0);
  }
protected:
  double _length;
private:
};

} // namespace

#endif /* Normal_H_ */
