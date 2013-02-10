#ifndef SENSOR_POLAR_2D_H
#define SENSOR_POLAR_2D_H

#include "obcore/math/Matrix.h"

namespace obvious
{

/**
 * @class SensorPolar2D
 * @brief Generic class for 2D measurement units using polar sampling
 * @author Stefan May
 */
class SensorPolar2D
{
public:

   /**
    * Standard constructor
    * @param[in] beams number of beams
    * @param[in] angularRes angular resolution, i.e. angle between beams in degree
    * @param[in] minPhi minimum angle from which beams are counted positive counter-clockwisely
    */
   SensorPolar2D(unsigned int beams, double angularRes, double minPhi);

   /**
    * Destructor
    */
   ~SensorPolar2D();

   /**
    * Transform current sensor pose
    * @param[in] T 3x3 transformation matrix
    */
   void transform(Matrix* T);

   /**
    * Accessor to sensor pose
    * @return pose
    */
   Matrix* getPose();

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
    * Get measurement vector
    * @return vector of distance data
    */
   double* getRealMeasurementData();

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
    * Assign an arbitrary coordinate to a measurement beam
    * @param[in] coordinate vector
    * @return beam index
    */
   int backProject(double* data);

private:

   int phi2Index(double phi);

   Matrix* _Pose;

   double* _data;

   bool* _mask;

   unsigned int _size;

   double _angularRes;

   double _minPhi;

};

}

#endif
