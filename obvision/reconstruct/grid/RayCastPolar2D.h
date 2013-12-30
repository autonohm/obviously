#ifndef RAYCASTPOLAR2D_H
#define RAYCASTPOLAR2D_H

#include <vector>
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

namespace obvious
{

/**
 * @class RayCastPolar2D
 * @brief
 * @author Stefan May
 */
class RayCastPolar2D
{
public:

  RayCastPolar2D();

  ~RayCastPolar2D();

  void calcCoordsFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, unsigned int* ctr);

  void calcCoordsAligned(TsdGrid* grid, double* coords, double* normals, unsigned int* cnt);

private:

  bool rayCastFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, const unsigned int beam, double coordinates[2], double normal[2]);

  void calcRayFromCurrentView(const unsigned int beam, double dirVec[2]);

};

}

#endif
