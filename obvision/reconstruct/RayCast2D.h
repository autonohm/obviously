#ifndef RAYCAST2D_H
#define RAYCAST2D_H

#include <vector>
#include "obcore/math/Matrix.h"
#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/SensorPolar2D.h"

namespace obvious
{

/**
 * @class RayCast2D
 * @brief
 * @author Stefan May, Philipp Koch
 */
class RayCast2D
{
public:

   RayCast2D();

   ~RayCast2D();

   void calcCoordsFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, unsigned int* ctr);

private:

   bool rayCastFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, const unsigned int beam, double coordinates[2], double normal[2]);

   void calcRayFromCurrentView(const unsigned int beam, double dirVec[2]);

};

}

#endif
