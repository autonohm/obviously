#ifndef RAYCAST3D_H
#define RAYCAST3D_H

#include <vector>
#include "obcore/math/linalg/linalg.h"
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
	RayCast3D();

  /**
   * Destructor
   */
	virtual ~RayCast3D();

	virtual void calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size);

  virtual void calcCoordsFromCurrentPoseMask(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, bool* mask, unsigned int* size);

	/**
	 * Overloaded method to cast a single ray trough several spaces. The method returns in case of a found coordinate or at
	 * the end of the TsdSpace input vector.
	 * @param sensor Pointer to sensor
	 * @param coords Coordinates of found point
	 * @param normals Normals of found point
	 * @param rgb Color of found point
	 * @param spaces Input vector with TsdSpaces to traverse
	 * @param offsets Offset vector of the spaces in world coordinates system
	 * @param u Coloumn the ray is casted through
	 * @param v Row the ray is casted through
	 * @return True in case of a found point, false otherwise
	 */
	/*virtual bool calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, const std::vector<TsdSpace*>& spaces,
											                    const std::vector<double>& offsets, const unsigned int u, const unsigned int v);*/


  /**
   * @param size Contains number of coords found
   */
  virtual bool calcCoordsAxisParallel(TsdSpace* space, double** pointCloud, double** cloudNormals, unsigned char** cloudRgb, unsigned int* size);

private:

  bool rayCastFromSensorPose(TsdSpace* space, double pos[3], double ray[3], double coordinates[3], double normal[3], unsigned char rgb[3], double* depth);

  /**
   *
   */
	bool rayCastParallelAxis(TsdSpace* space, double* footPoint,double* dirVec,std::vector<double>* pointCloud, std::vector<double>* cloudNormals, std::vector<unsigned char>* cloudRgb,const unsigned int steps);

  /**
   *
   */
	void calcRayParallelAxis(TsdSpace* space, const unsigned int row, const unsigned int col, double* footPoint, double* dirVec, unsigned int* steps, AXSPARMODE mode);

protected:

};

}

#endif //RAYCAST3D
