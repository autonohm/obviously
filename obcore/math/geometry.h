#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include "obcore/base/CartesianCloud.h"
#include <gsl/gsl_matrix.h>

namespace obvious
{

void calculatePerspective(gsl_matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample=1);

void calculatePerspective_cblas(gsl_matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample);

void calculatePerspectiveUndistorted_cblas(gsl_matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample);

bool axisAngle(Matrix M, gsl_vector* axis, double* angle);

}

#endif
