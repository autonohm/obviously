#include "OutOfBoundsFilter2D.h"
#include "obcore/base/System.h"
#include <string.h>

namespace obvious
{

OutOfBoundsFilter2D::OutOfBoundsFilter2D(double xMin, double xMax, double yMin, double yMax)
{
  _xMin = xMin;
  _xMax = xMax;
  _yMin = yMin;
  _yMax = yMax;
  _T = new Matrix(3, 3);;
}

OutOfBoundsFilter2D::~OutOfBoundsFilter2D()
{
  delete _T;
}

void OutOfBoundsFilter2D::setPose(Matrix* T)
{
  _T->copy(*T);
}

void OutOfBoundsFilter2D::filter(double** scene, unsigned int size, bool* mask)
{
  Matrix S(size, 2, *scene);
  S.transform(*_T);

  for(unsigned int i=0; i<size; i++)
  {
    double* pt = S[i];
    if(pt[0]<_xMin || pt[0]>_xMax || pt[1]<_yMin || pt[1]>_yMax)
      mask[i] = false;
  }
}

}

