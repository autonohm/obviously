#include "OutOfBoundsFilter3D.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
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
  *_T = *T;
}

void OutOfBoundsFilter3D::filter(double** scene, unsigned int size, bool* mask)
{
  Matrix S(size, 3, *scene);

  // Transform measurements back to world coordinate system
  S.transform(*_T);

//  int cnt = 0;
  for(unsigned int i=0; i<size; i++)
  {
    // check bounds of world
    if(S(i,0)<_xMin || S(i,0)>_xMax || S(i,1)<_yMin || S(i,1)>_yMax || S(i,2)<_zMin || S(i,2)>_zMax)
    {
      mask[i] = false;
    }
//    else
//      cnt++;
  }

//  LOGMSG(DBG_DEBUG, "Bounding Box left: " << cnt << " points");

}

}

