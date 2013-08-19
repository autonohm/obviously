#ifndef RAYCASTPROJECTIVE3D_H
#define RAYCASTPROJECTIVE3D_H

#include <vector>
#include "obcore/math/Matrix.h"
#include "TsdSpace.h"

class Projection;

namespace obvious
{

enum AXSPARMODE
{
	X_AXS,
	X_AXS_N,
	Y_AXS,
	Y_AXS_N,
	Z_AXS,
	Z_AXS_N
};

/**
 * @class RayCastProjective3D
 * @brief Implementation of different ray casting methods
 * @author Philipp Koch, Stefan May
 */
class RayCastProjective3D
{
public:

	RayCastProjective3D(const unsigned int cols, const unsigned int rows, Projection *projection, TsdSpace* space);

	~RayCastProjective3D();

	void setTransformation(double *TData);

	double* getTransformation();

	void calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* ctr, unsigned int subsampling=1);

	bool generatePointCloud(double **pointCloud, double **cloudNormals, unsigned char **cloudRgb, unsigned int *nbr);

	bool generatePointCloudPositive(double **pointCloud, double **cloudNormals, unsigned char **cloudRgb, unsigned int *nbr);

private:

	bool rayCastFromCurrentView(const unsigned int row, const unsigned int col, double coordinates[3], double normal[3], unsigned char rgb[3], double* depth);

	void calcRayFromCurrentView(const unsigned int row, const unsigned int col, double dirVec[3]);

	bool rayCastParallelAxis(double *footPoint,double *dirVec,std::vector<double> *pointCloud,std::vector<double> *cloudNormals, std::vector<unsigned char>* cloudRgb,const unsigned int steps);

	bool calcRayParallelAxis(const unsigned int row, const unsigned int col, double *footPoint, double *dirVec, unsigned int *steps, AXSPARMODE mode);

	 Matrix *_T;

	 Matrix *_Tinv;

	 double _tr[3];

	 Matrix*** _rays;

	 TsdSpace* _space;

	 Projection* _projection;

	 unsigned int _cols;

	 unsigned int _rows;
};

}

#endif //RAYCASTPROJECTIVE3D
