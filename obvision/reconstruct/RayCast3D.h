#ifndef RAYCAST3D_H
#define RAYCAST3D_H

#include <vector>
#include "obcore/math/Matrix.h"
#include "TsdSpace.h"

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
 * @class RayCast3D
 * @brief Implementation of axis-parallel ray casting methods
 * @author Philipp Koch, Stefan May
 */
class RayCast3D
{
public:

  /**
   * Constructor
   */
	RayCast3D(TsdSpace* space);

  /**
   * Destructor
   */
	virtual ~RayCast3D();

	virtual void calcCoordsFromCurrentPose(Sensor* rayCastFromSensorPosesensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size);

	virtual void calcCoordsFromCurrentPoseMask(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, bool* mask, unsigned int* size);

	virtual bool rayCastFromSensorPose(double ray[3], double coordinates[3], double normal[3], unsigned char rgb[3], double* depth, Sensor* sensor);

  /**
   * @param size Contains number of coords found
   */
	virtual bool generatePointCloud(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size);

  /**
   *
   */
	virtual bool generatePointCloudPositive(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size);

private:

  /**
   *
   */
	bool rayCastParallelAxis(double* footPoint,double* dirVec,std::vector<double>* pointCloud, std::vector<double>* cloudNormals, std::vector<unsigned char>* cloudRgb,const unsigned int steps);

  /**
   *
   */
	bool calcRayParallelAxis(const unsigned int row, const unsigned int col, double* footPoint, double* dirVec, unsigned int* steps, AXSPARMODE mode);

protected:

	 TsdSpace* _space;

};

}

#endif //RAYCAST3D
