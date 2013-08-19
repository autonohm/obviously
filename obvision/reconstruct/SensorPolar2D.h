#ifndef SENSOR_POLAR_2D_H
#define SENSOR_POLAR_2D_H

#include "obcore/math/Matrix.h"
#include "Sensor.h"

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
   * @param[in] minPhi minimum angle from which beams are counted positive counter-clockwisely (rad)
   */
  SensorPolar2D(unsigned int beams, double angularRes, double minPhi);

  /**
   * Destructor
   */
  ~SensorPolar2D();

  /**
   * Accessor to sensor translation
   * @param[out] tr translation vector
   */
  void getPosition(double tr[2]);

  /**
   * Get size of measurement vector
   * @return number of beams
   */
  unsigned int getRealMeasurementSize();

  /**
   * Copy measurement data to internal buffer
   * @param data source with 2D coordinates
   */
  void setRealMeasurementData(double* data);

  /**
   * Get measurement vector
   * @return vector of distance data
   */
  double* getRealMeasurementData();

  /**
   * Copy measurement mask
   * @param mask source mask
   */
  void setRealMeasurementMask(bool* mask);

  /**
   * Get validity mask
   * @return validity mask vector. True signals a valid measurement
   */
  bool* getRealMeasurementMask();

  /**
   * Calculate ray of specific beam
   * @param[in] beam beam index
   * @param[out] ray vector
   */
  void calcRay(unsigned int beam, double ray[2]);

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
   */
  void backProject(Matrix* M, int* indices);

private:

  int phi2Index(double phi);

  Matrix* _Pose;

  double* _data;

  bool* _mask;

  unsigned int _size;

  double _angularRes;

  double _minPhi;

  double _phiLowerBound;

  double _phiUpperBound;

};

}

#endif
