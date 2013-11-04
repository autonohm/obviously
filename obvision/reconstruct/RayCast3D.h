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
	Y_AXS,
	Z_AXS,
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

	virtual bool calcCoordsFromCurrentPose(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, const std::vector<TsdSpace*>& spaces,
											                    const std::vector<double>& offsets, const unsigned int u, const unsigned int v);

	virtual bool calcCoordsFromCurrentPose(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, TsdSpace& space,
	                                                                const double offset[3], const unsigned int u, const unsigned int v);

	virtual void calcCoordsFromCurrentPoseMask(Sensor* sensor, double* coords, double* normals, unsigned char* rgb, bool* mask, unsigned int* size);


  /**
   * @param size Contains number of coords found
   */
  virtual bool calcCoordsAxisParallel(double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size);

  void setSpace(TsdSpace* space){_space = space;}
private:

  bool rayCastFromSensorPose(double ray[3], double coordinates[3], double normal[3], unsigned char rgb[3], double* depth, Sensor* sensor);

  /**
   *
   */
	bool rayCastParallelAxis(double* footPoint,double* dirVec,std::vector<double>* pointCloud, std::vector<double>* cloudNormals, std::vector<unsigned char>* cloudRgb,const unsigned int steps);

  /**
   *
   */
	void calcRayParallelAxis(const unsigned int row, const unsigned int col, double* footPoint, double* dirVec, unsigned int* steps, AXSPARMODE mode);

protected:

	 TsdSpace* _space;

};

}

#endif //RAYCAST3D
