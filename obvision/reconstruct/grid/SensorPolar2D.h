#ifndef SENSOR_POLAR_2D_H
#define SENSOR_POLAR_2D_H

#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class SensorPolar2D
 * @brief Generic class for 2D measurement units using polar sampling
 * @author Stefan May
 */
class SensorPolar2D : public Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] beams number of beams
   * @param[in] angularRes angular resolution, i.e. angle between beams in rad
   * @param[in] phiMin minimum angle from which beams are counted positive counter-clockwisely (rad)
   * @param[in] maxRange maximum range
   * @param[in] minRange minimum range
   */
  SensorPolar2D(unsigned int beams, double angularRes, double phiMin, double maxRange=INFINITY, double minRange=0.0);

  /**
   * Destructor
   */
  ~SensorPolar2D();


  /**
   * Assign an arbitrary 2D coordinate to a measurement beam
   * @param[in] coordinate vector
   * @return beam index
   */
  int backProject(double data[2]);

  /**
   * Parallel version of back projection
   * @param[in] M matrix of homogeneous 2D coordinates
   * @param[out] indices vector of beam indices
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(Matrix* M, int* indices, Matrix* T=NULL);

  /**
   * Get angular resolution
   * @return angular resolution
   */
  double getAngularResolution();

  /**
   * Get the minimum angle
   * @return minimum angle
   */
  double getPhiMin();

  void setTransformation(obvious::Matrix& T);

private:

  double _angularRes;

  double _phiMin;

  double _phiLowerBound;

  double _phiUpperBound;
};

}

#endif
