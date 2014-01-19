#ifndef SENSOR_PROJECTIVE_3D_H
#define SENSOR_PROJECTIVE_3D_H

#include "obvision/reconstruct/Sensor.h"

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
   * Constructor
   * @param[in] cols number of image columns
   * @param[in] rows number of image rows
   * @param[in] PData 3x4 projection matrix
   * @param[in] maxRange maximum range
   * @param[in] minRange minimum range
   */
  SensorProjective3D(unsigned int cols, unsigned int rows, double PData[12], double maxRange=INFINITY, double minRange=0.0);

  /**
   * Copy constructor
   * @param sensor sensor instance to be copied
   */
  SensorProjective3D(SensorProjective3D* sensor);

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
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(Matrix* M, int* indices, Matrix* T=NULL);

private:

  void init(unsigned int cols, unsigned int rows, double PData[12]);

  Matrix* _P;

};

}

#endif
