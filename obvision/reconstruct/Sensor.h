#ifndef SENSOR_H_
#define SENSOR_H_

#include "obcore/math/Matrix.h"

namespace obvious
{

/**
 * @class Sensor
 * @brief Abstract class for 2D and 3D measurement units
 * @author Stefan May
 */
class Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] dim dimensionality of representation
   */
  Sensor(unsigned int dim);

  /**
   * Destructor
   */
  virtual ~Sensor();

  /**
   * Transform current sensor pose
   * @param[in] T transformation matrix
   */
  virtual void transform(Matrix* T);

  /**
   * Accessor to sensor pose
   * @return pose
   */
  virtual Matrix* getPose();

  /**
   * Accessor to sensor translation
   * @param[out] tr translation vector
   */
  virtual void getPosition(double* tr);

  /**
   * Get size of measurement vector
   * @return number of beams
   */
  virtual unsigned int getRealMeasurementSize();

  /**
   * Copy measurement data to internal buffer
   * @param data source with 2D coordinates
   */
  virtual void setRealMeasurementData(double* data);

  /**
   * Get measurement vector
   * @return vector of distance data
   */
  virtual double* getRealMeasurementData();

  /**
   * Copy measurement mask
   * @param mask source mask
   */
  virtual void setRealMeasurementMask(bool* mask);

  /**
   * Get validity mask
   * @return validity mask vector. True signals a valid measurement
   */
  virtual bool* getRealMeasurementMask();

  virtual void backProject(Matrix* M, int* indices) = 0;

protected:

  Matrix* _Pose;

  unsigned int _dim;

  unsigned int _size;

  double* _data;

  bool* _mask;

};

}

#endif
