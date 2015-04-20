#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "obcore/base/types.h"
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class Quaternion
 * @author Stefan May
 * @date 05.12. 2014
 */
class Quaternion
{

public:

  /**
   * Default constructor, Rotation about 0 degrees
   */
  Quaternion();

  /**
   * Constructor
   * @param w q.w
   * @param x q.x
   * @param y q.y
   * @param z q.z
   */
  Quaternion(obfloat w, obfloat x, obfloat y, obfloat z);

  /**
   * Constructor
   * @param R 2x2 or 3x3 rotation matrix, for 2x2 a rotation about the z-axis is created
   */
  Quaternion(Matrix R);

  /**
   * Create quaternion rotation about the x-axis
   * @param psi rotation angle
   * @return Quaternion instance
   */
  static Quaternion QuaternionAroundX(obfloat psi);

  /**
   * Create quaternion rotation about the y-axis
   * @param theta rotation angle
   * @return Quaternion instance
   */
  static Quaternion QuaternionAroundY(obfloat theta);

  /**
   * Create quaternion rotation about the z-axis
   * @param phi rotation angle
   * @return Quaternion instance
   */
  static Quaternion QuaternionAroundZ(obfloat phi);

  /**
   * Destructor
   */
  virtual ~Quaternion() {;}

  /**
   * Const accessor
   * @return w-component
   */
  double w() const {return _w;};

  /**
   * Const accessor
   * @return x-component of axis
   */
  double x() const {return _x;};

  /**
   * Const accessor
   * @return y-component of axis
   */
  double y() const {return _y;};

  /**
   * Const accessor
   * @return z-component of axis
   */
  double z() const {return _z;};

  /**
   * Add two quaternions
   * @param q1 first quaternion
   * @param q2 second quaternion
   * @return resulting quaternion
   */
  friend Quaternion operator + (const Quaternion &q1, const Quaternion &q2);

  /**
   * Multiply two quaternions
   * @param q1 first quaternion
   * @param q2 second quaternion
   * @return resulting quaternion
   */
  friend Quaternion operator * (const Quaternion &q1, const Quaternion &q2);

  /**
   * Add quaternion
   * @param q quaternion
   * @return resulting quaternion, i.e., this instance
   */
  Quaternion& operator += (const Quaternion &q)
  {
    _w += q._w;
    _x += q._x;
    _y += q._y;
    _z += q._z;
    return *this;
  }

  /**
   * Subtract quaternion
   * @param q quaternion
   * @return resulting quaternion, i.e., this instance
   */
  Quaternion& operator -= (const Quaternion &q)
  {
    _w -= q._w;
    _x -= q._x;
    _y -= q._y;
    _z -= q._z;
    return *this;
  }

  /**
   * Conjugate quaternion
   */
  void conjugate()
  {
    _x = -_x;
    _y = -_y;
    _z = -_z;
  }

  /**
   * Convert quaterion to 3x3 rotation matrix
   * @return matrix
   */
  Matrix convertToMatrix();

private:

  obfloat _w;

  obfloat _x;

  obfloat _y;

  obfloat _z;
};

inline Quaternion::Quaternion()
{
  _w = 1.0;
  _x = 0.0;
  _y = 0.0;
  _z = 0.0;
}

inline Quaternion::Quaternion(obfloat w, obfloat x, obfloat y, obfloat z)
{
  _w = w;
  _x = x;
  _y = y;
  _z = z;
}

inline Quaternion operator + (const Quaternion &q1, const Quaternion &q2)
{
  Quaternion q(q1);
  q += q2;
  return q;
}

} /* namespace obvious */
#endif /* QUATERNION_H_ */
