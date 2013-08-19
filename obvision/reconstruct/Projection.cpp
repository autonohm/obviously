#include "Projection.h"
#include "obcore/math/Matrix.h"
#include "obcore/base/Logger.h"
#include <cmath>

using obvious::Matrix;

Projection::Projection(double proj_data[12])
{
	_projection=new Matrix(3,4);
	_projection->setData(proj_data);
}

Projection::~Projection()
{
	delete _projection;
}

double* Projection::operator [] (unsigned int i)
{
	return (*_projection)[i];
}

void Projection::project2Space(const unsigned int col, const unsigned int row, const double depth, Matrix* point3)
{
	double fx;
	double fy;
	double Tx;
	double Ty;

	fx = (*_projection)[0][0];
	fy = (*_projection)[1][1];
	Tx = (*_projection)[0][2];
	Ty = (*_projection)[1][2];

	double x = (depth/fx)*(col-Tx);
	double y = (depth/fy)*(row-Ty);
	//double lambda_inv = 1./sqrt(x * x + y * y + 1.);
	//double z = depth * lambda_inv;

	// calculate point
	(*point3)[0][0]=x;
	(*point3)[1][0]=y;
	(*point3)[2][0]=depth;
	(*point3)[3][0]=1.0;
}

void Projection::project2Plane(const Matrix* point3D, double* col, double* row)
{
	Matrix point2D(3,1);
	point2D = *_projection * (*point3D);

	double scale = point2D[2][0];
	*col = point2D[0][0] / scale;
	*row = point2D[1][0] / scale;
}
