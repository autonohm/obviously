#include "OutOfBoundsFilter3D.h"
#include "obcore/base/System.h"
#include <string.h>

namespace obvious
{

OutOfBoundsFilter3D::OutOfBoundsFilter3D(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
{
  _xMin = xMin;
  _xMax = xMax;
  _yMin = yMin;
  _yMax = yMax;
  _zMin = zMin;
  _zMax = zMax;
  _T = new Matrix(4, 4);
}

OutOfBoundsFilter3D::~OutOfBoundsFilter3D()
{
  delete _T;
}

void OutOfBoundsFilter3D::setPose(Matrix* T)
{
  _T->copy(*T);
}

void OutOfBoundsFilter3D::filter(double** scene, unsigned int size, bool* mask)
{
  Matrix S(size, 3, *scene);
  S.transform(*_T);

  for(unsigned int i=0; i<size; i++)
  {
    double* pt = S[i];
    if(pt[0]<_xMin || pt[0]>_xMax || pt[1]<_yMin || pt[1]>_yMax || pt[2]<_zMin || pt[2]>_zMax)
      mask[i] = false;
  }
}

}

