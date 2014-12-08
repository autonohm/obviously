#include "Quaternion.h"

#include "obcore/base/Logger.h"
#include "math.h"

namespace obvious
{

Quaternion::Quaternion()
{
  _w = 1.0;
  _x = 0.0;
  _y = 0.0;
  _z = 0.0;
}

Quaternion::Quaternion(obfloat w, obfloat x, obfloat y, obfloat z)
{
  _w = w;
  _x = x;
  _y = y;
  _z = z;
}

Quaternion::Quaternion(Matrix R)
{

  if(R.getCols()==2 && R.getRows()==2)
  {
    double trace = R.trace() + 1.0;

    _x = 0.0;
    _y = 0.0;

    if(trace>1e-6)
    {
      obfloat s = 0.5 / sqrt(trace+1.0);
      _w = 0.25 / s;
      _z = (R(1,0) - R(0,1)) * s;
    }
    else
    {
      obfloat s = 2.0 * sqrt(2.0 - R(0,0) - R(1,1));
      _w = (R(1,0)-R(0,1)) / s;
      _z = 0.25 * s;
    }
  }
  else if(R.getCols()==3 && R.getRows()==3)
  {
    double trace = R.trace();

    if(trace>1e-6)
    {
      obfloat s = 0.5 / sqrt(trace+1.0);
      _w = 0.25 / s;
      _x = (R(2,1) - R(1,2)) * s;
      _y = (R(0,2) - R(2,0)) * s;
      _z = (R(1,0) - R(0,1)) * s;
    }
    else
    {
      if(R(0,0)>R(1,1) && R(0,0)>R(2,2))
      {
        obfloat s = 2.0 * sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
        _w = (R(2,1) - R(1,2)) / s;
        _x = 0.25 * s;
        _y = (R(0,1)+R(1,0)) / s;
        _z = (R(0,2)+R(2,0)) / s;
      }
      else if(R(1,1)>R(2,2))
      {
        obfloat s = 2.0 * sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
        _w = (R(0,2)-R(2,0)) / s;
        _x = (R(0,1)+R(1,0)) / s;
        _y = 0.25 * s;
        _z = (R(1,2)+R(2,1)) / s;
      }
      else
      {
        obfloat s = 2.0 * sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
        _w = (R(1,0)-R(0,1)) / s;
        _x = (R(0,2)+R(2,0)) / s;
        _y = (R(1,2)+R(2,1)) / s;
        _z = 0.25 * s;
      }
    }
  }
  else
  {
    LOGMSG(DBG_ERROR, "No square matrix passed.");
  }
}

Quaternion Quaternion::QuaternionAroundX(obfloat psi)
{
  Quaternion q;
  q._w = cos(psi/2.0);
  q._x = sin(psi/2.0);
  q._y = 0.0;
  q._z = 0.0;
  return q;
}

Quaternion Quaternion::QuaternionAroundY(obfloat theta)
{
  Quaternion q;
  q._w = cos(theta/2.0);
  q._x = 0.0;
  q._y = sin(theta/2.0);
  q._z = 0.0;
  return q;
}

Quaternion Quaternion::QuaternionAroundZ(obfloat phi)
{
  Quaternion q;
  q._w = cos(phi/2.0);
  q._x = 0.0;
  q._y = 0.0;
  q._z = sin(phi/2.0);
  return q;
}

Quaternion::~Quaternion()
{

}

double Quaternion::w() const
{
  return _w;
}

double Quaternion::x() const
{
  return _x;
}

double Quaternion::y() const
{
  return _y;
}

double Quaternion::z() const
{
  return _z;
}

Quaternion operator + (const Quaternion &q1, const Quaternion &q2)
{
  Quaternion q(q1._w, q1._x, q1._y, q1._z);
  q._w += q2._w;
  q._x += q2._x;
  q._y += q2._y;
  q._z += q2._z;
  return q;
}

Quaternion operator * (const Quaternion &q1, const Quaternion &q2)
{
  Quaternion q;
  q._w = q1._w*q2._w - q1._x*q2._x - q1._y*q2._y - q1._z*q2._z;
  q._x = q1._y*q2._z - q1._z*q2._y + q1._w*q2._x + q1._x*q2._w;
  q._y = q1._z*q2._x - q1._x*q2._z + q1._w*q2._y + q1._y*q2._w;
  q._z = q1._x*q2._y - q1._y*q2._x + q1._w*q2._z + q1._z*q2._w;
  return q;
}

Quaternion& Quaternion::operator += (const Quaternion &q)
{
  _w += q._w;
  _x += q._x;
  _y += q._y;
  _z += q._z;
  return *this;
}

Quaternion& Quaternion::operator -= (const Quaternion &q)
{
  _w -= q._w;
  _x -= q._x;
  _y -= q._y;
  _z -= q._z;
  return *this;
}

void Quaternion::conjugate()
{
  _x = -_x;
  _y = -_y;
  _z = -_z;
}

Matrix Quaternion::convertToMatrix()
{
  Matrix M(3, 3);
  double ww = _w*_w;
  double xx = _x*_x;
  double yy = _y*_y;
  double zz = _z*_z;

  M(0, 0) = ww + xx - yy - zz;
  M(0, 1) = -2.0 * (_w*_z - _x*_y);
  M(0, 2) =  2.0 * (_w*_y + _x*_z);
  M(1, 0) =  2.0 * (_w*_z + _x*_y);
  M(1, 1) = ww - xx + yy - zz;
  M(1, 2) = -2.0 * (_w*_x - _y*_z);
  M(2, 0) = -2.0 * (_w*_y - _x*_z);
  M(2, 1) =  2.0 * (_w*_x + _y*_z);
  M(2, 2) = ww - xx - yy + zz;
  return M;
}

} /* namespace obvious */
