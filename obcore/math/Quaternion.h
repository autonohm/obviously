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
  virtual ~Quaternion();

  /**
   * Const accessor
   * @return w-component
   */
  double w() const;

  /**
   * Const accessor
   * @return x-component of axis
   */
  double x() const;

  /**
   * Const accessor
   * @return y-component of axis
   */
  double y() const;

  /**
   * Const accessor
   * @return z-component of axis
   */
  double z() const;

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
  Quaternion& operator += (const Quaternion &q);

  /**
   * Subtract quaternion
   * @param q quaternion
   * @return resulting quaternion, i.e., this instance
   */
  Quaternion& operator -= (const Quaternion &q);

  /**
   * Conjugate quaternion
   */
  void conjugate();

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

} /* namespace obvious */
#endif /* QUATERNION_H_ */
