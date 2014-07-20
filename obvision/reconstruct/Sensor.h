#ifndef SENSOR_H_
#define SENSOR_H_

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/reconstruct_defs.h"
#include <vector>
#include <cmath>

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
  Sensor(unsigned int dim, double maxRange, double minRange);

  /**
   * Destructor
   */
  virtual ~Sensor();

  /**
   * Get size of first dimension
   * @return size
   */
  virtual unsigned int getWidth();

  /**
   * Get size of second dimension
   * @return size
   */
  virtual unsigned int getHeight();

  /**
   * Get maximum range
   * @return maximum range
   */
  virtual double getMaximumRange();

  /**
   * Get minimum range
   * @return minimum range
   */
  virtual double getMinimumRange();

  /**
   * Access matrix of measurement rays with parameterizable normalization
   * @param norm normalization value
   * @return Ray matrix, i.e. R(dimensionality, measurement size)
   */
  virtual Matrix* getNormalizedRayMap(double norm);

  /**
   * Transform current sensor pose in his own coordinate system
   * P' = P T = [Rp | tp] [R | t] = [Rp R | Rp t + tp], where P is the old and P' is the new pose
   * @param[in] T transformation matrix
   */
  virtual void transform(Matrix* T);

  /**
   * Translate current sensor pose
   * @param tr[in] tr translation vector
   */
  virtual void translate(double* tr);

  /**
   * Accessor to sensor transformation
   * @return (dim+1)x(dim+1) transformation matrix (R | t; 0 1)
   */
  Matrix getTransformation();

  /**
   * Set transformation matrix
   * @param T transformation matrix
   */
  void setTransformation(Matrix T);

  /**
   * Reset sensor pose to identity
   */
  void resetTransformation();

  /**
   * Accessor to sensor translation
   * @param[out] tr translation vector
   */
  virtual void getPosition(obfloat* tr);

  /**
   * Get size of measurement vector
   * @return number of beams
   */
  virtual unsigned int getRealMeasurementSize();

  /**
   * Copy measurement data to internal buffer
   * @param data source with 2D coordinates
   * @param scale scale factor to multiply distances
   */
  virtual void setRealMeasurementData(double* data, double scale=1.0);

  virtual void setRealMeasurementData(vector<float> data, float scale=1.f);

  /**
   * Get measurement vector
   * @return vector of distance data
   */
  virtual double* getRealMeasurementData();

  /**
   * Convert distance data in measurement array to Cartesian coordinates in sensor coordinate system
   * @param coords Output array of size dim*size. Output is grouped in n-tuples [x1 y1 ....]
   */
  unsigned int dataToCartesianVector(double* &coords);

  Matrix dataToHomogeneousCoordMatrix();

  virtual void setRealMeasurementAccuracy(double* accuracy);

  virtual double* getRealMeasurementAccuracy();

  virtual bool hasRealMeasurmentAccuracy();

  /**
   * Copy measurement mask
   * @param mask source mask
   */
  virtual void setRealMeasurementMask(bool* mask);

  virtual void setRealMeasurementMask(vector<unsigned char> mask);

  /**
   * Get validity mask
   * @return validity mask vector. True signals a valid measurement
   */
  virtual bool* getRealMeasurementMask();

  /**
   * @return flag indicate availability of RGB data
   */
  virtual bool hasRealMeasurmentRGB();

  /**
   * Copy rgb data to internal buffer
   * @param rgb color/texture data
   */
  virtual void setRealMeasurementRGB(unsigned char* rgb);

  /**
   * Get color data
   * @return pointer to internal color data (may be null for certain sensors)
   */
  virtual unsigned char* getRealMeasurementRGB();

  /**
   * Project coordinate back to sensor index
   * @param[in] M matrix of coordinates (homogeneous)
   * @param[out] indices vector of projection results (must be allocated outside)
   * @param[in] T temporary transformation matrix of coordinates
   */
  virtual void backProject(Matrix* M, int* indices, Matrix* T=NULL) = 0;

protected:

  Matrix* _T;

  unsigned int _dim;

  double _maxRange;

  double _minRange;

  unsigned int _size;

  double* _data;

  double* _accuracy;

  bool* _mask;

  unsigned char* _rgb;

  unsigned int _width;

  unsigned int _height;

  double _rayNorm;

  // Ray matrix in world coordinate frame
  Matrix* _rays;

  // Ray matrix in sensor coordinate frame
  Matrix* _raysLocal;
};

}

#endif
