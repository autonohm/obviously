#ifndef SENSOR_POLAR_3D_H
#define SENSOR_POLAR_3D_H

#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class SensorPolar3D
 * @brief Generic class for 3D measurement units using polar sampling
 * @author Stefan May
 */
class SensorPolar3D : public Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] beams number of beams in scanning plane
   * @param[in] thetaRes angular resolution, i.e. angle between beams in rad
   * @param[in] thetaMin minimum angle from which beams are counted positive counter-clockwisely (rad)
   * @param[in] phiRes angular resolution, i.e. angle between scanning planes in rad
   */
  SensorPolar3D(unsigned int beams, double thetaRes, double thetaMin, double phiRes=0.25, double maxRange=INFINITY, double minRange=0.0);

  /**
   * Destructor
   */
  ~SensorPolar3D();

  /**
   * Parallel version of back projection
   * @param[in] M matrix of homogeneous 3D coordinates
   * @param[out] indices vector of beam indices
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(Matrix* M, int* indices, Matrix* T=NULL);

  void setDistanceMap(vector<float> phi, vector<float> dist);

private:

  double _thetaRes;

  double _phiRes;

  double _thetaMin;

  double _thetaLowerBound;

  double _thetaUpperBound;

  double** _distanceMap;

  int** _indexMap;

};

}

#endif
