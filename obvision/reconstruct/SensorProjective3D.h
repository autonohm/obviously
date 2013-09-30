#ifndef SENSOR_PROJECTIVE_3D_H
#define SENSOR_PROJECTIVE_3D_H

#include "obcore/math/Matrix.h"
#include "Sensor.h"

namespace obvious
{

/**
 * @class SensorProjective3D
 * @brief Generic class for 3D measurement units using a pinhole camera model
 * @author Stefan May
 */
class SensorProjective3D : public Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] cols number of image columns
   * @param[in] rows number of image rows
   * @param[in] PData 3x4 projection matrix
   * @param[in] voxelSize edge length of voxel
   */
  SensorProjective3D(unsigned int cols, unsigned int rows, double PData[12], double voxelSize);

  /**
   * Destructor
   */
  ~SensorProjective3D();

  /**
   * Project point to 3D space at certain distance at given pixel
   * @param[in] col column index
   * @param[in] row row index
   * @param[in] depth distance from pixel
   * @param[out] coord 3D coordinates (homogeneous) of projected point/direction vector
   */
  void project2Space(const unsigned int col, const unsigned int row, const double depth, Matrix* coord);

  /**
   * Parallel version of back projection
   * @param[in] M matrix of homogeneous 3D coordinates
   * @param[out] indices vector of beam indices
   */
  void backProject(Matrix* M, int* indices);

  /**
   *
   */
  void calcRayFromCurrentPose(const unsigned int row, const unsigned int col, double dirVec[3]);

private:

  Matrix* _P;

  Matrix*** _rays;

};

}

#endif
