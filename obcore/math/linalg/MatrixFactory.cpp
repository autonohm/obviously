#include "MatrixFactory.h"
#include <math.h>
#include "linalg.h"

namespace obvious
{

Matrix MatrixFactory::TranslationMatrix33(double tx, double ty)
{
  Matrix M(3, 3);
  M.setIdentity();
  M(0,2) = tx;
  M(1,2) = ty;
  return M;
}

Matrix MatrixFactory::TranslationMatrix44(double tx, double ty, double tz)
{
  Matrix M(4, 4);
  M.setIdentity();
  M(0,3) = tx;
  M(1,3) = ty;
  M(2,3) = tz;
  return M;
}

Matrix MatrixFactory::TransformationMatrix33(double phi, double tx, double ty)
{
  Matrix M = MatrixFactory::TranslationMatrix33(tx, ty);
  double cphi   = cos(phi);
  double sphi   = sin(phi);
  M(0,0) = cphi;     M(0,1) = -sphi;
  M(1,0) = sphi;     M(1,1) = cphi;
  return M;
}

Matrix MatrixFactory::TransformationMatrix44(double phi, double theta, double psi, double tx, double ty, double tz)
{
  Matrix M = MatrixFactory::TranslationMatrix44(tx, ty, tz);
  double cphi   = cos(phi);
  double ctheta = cos(theta);
  double cpsi   = cos(psi);
  double sphi   = sin(phi);
  double stheta = sin(theta);
  double spsi   = sin(psi);
  M(0,0) = cphi*ctheta;     M(0,1) = cphi*stheta*spsi-sphi*cpsi;    M(0,2) = cphi*stheta*cpsi + sphi*spsi;
  M(1,0) = sphi*ctheta;     M(1,1) = sphi*stheta*spsi+cphi*cpsi;    M(1,2) = sphi*stheta*cpsi - cphi*spsi;
  M(2,0) = -stheta;         M(2,1) = ctheta*spsi;                   M(2,2) = ctheta*cpsi;
  return M;
}

}
