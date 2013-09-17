#ifndef RAYCASTPOLAR3D_H
#define RAYCASTPOLAR3D_H

#include <vector>
#include "obcore/math/Matrix.h"
#include "TsdSpace.h"
#include "SensorPolar3D.h"
#include "RayCast3D.h"

namespace obvious
{

/**
 * @class RayCastPolar3D
 * @brief Implementation of polar ray casting
 * @author Stefan May
 */
class RayCastPolar3D : public RayCast3D
{
public:

  /**
   *
   */
  RayCastPolar3D(SensorPolar3D* sensor, TsdSpace* space);

  /**
   *
   */
  ~RayCastPolar3D();

  /**
   *
   */
  void calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* size);

private:

  SensorPolar3D* _sensor;
};

}

#endif //RAYCASTPOLAR3D
