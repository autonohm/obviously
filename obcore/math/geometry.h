#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include "obcore/base/CartesianCloud.h"
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

void calculatePerspective(Matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample=1);

bool axisAngle(Matrix M, double* axis, double* angle);

}

#endif
