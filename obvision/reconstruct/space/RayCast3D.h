#ifndef RAYCAST3D_H
#define RAYCAST3D_H

#include <vector>
#include "obcore/math/linalg/linalg.h"
#include "TsdSpace.h"

namespace obvious
{

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > stdVecEig3f;

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

	/**
	 *
	 * @param space
	 * @param sensor
	 * @param coords
	 * @param normals
	 * @param rgb
	 * @param size
	 */
	virtual void calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size, const unsigned int subs = 1);
	virtual void callCoordsFromCurrentPose(TsdSpace* space, const Eigen::Vector3f& pos, const stdVecEig3f& rays, stdVecEig3f* const coords, Eigen::Vector3f* const normal);
	/**
	 *
	 * @param space
	 * @param sensor
	 * @param coords
	 * @param normals
	 * @param rgb
	 * @param mask
	 * @param size
	 */
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


private:

  bool rayCastFromSensorPose(TsdSpace* space, obfloat pos[3], obfloat ray[3], obfloat coordinates[3], obfloat normal[3], unsigned char rgb[3], obfloat* depth);

  bool rayCastDummy(TsdSpace* space, const Eigen::Vector3f& pos, const Eigen::Vector3f& ray, Eigen::Vector3f* const coords, const float length = 10.0);

  obfloat _xmin;
  obfloat _ymin;
  obfloat _zmin;

  obfloat _xmax;
  obfloat _ymax;
  obfloat _zmax;

  obfloat _idxMin;
  obfloat _idxMax;
};

}

#endif //RAYCAST3D
