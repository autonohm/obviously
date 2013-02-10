#include "RayCast2D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCast2D::RayCast2D()
{

}

RayCast2D::~RayCast2D()
{

}

void RayCast2D::calcCoordsFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, unsigned int* cnt)
{
	Timer t;
	*cnt = 0;

   double c[2];
   double n[2];
   Matrix M(3,1);
   Matrix N(3,1);
   Matrix* T = sensor->getPose();
   Matrix Ti(3, 3);
   Ti = T->getInverse();
   M[2][0] = 1.0;
   N[2][0] = 0.0; // no translation for normals

   for (unsigned int beam = 0; beam < sensor->getRealMeasurementSize(); beam++)
   {
      if (rayCastFromCurrentView(grid, sensor, beam, c, n)) // Ray returned with coordinates
      {
         M[0][0] = c[0];
         M[1][0] = c[1];
         N[0][0] = n[0];
         N[1][0] = n[1];
         M       = Ti * M;
         N       = Ti * N;
         for (unsigned int i = 0; i < 2; i++)
         {
            coords[*cnt] = M[i][0];
            normals[(*cnt)++] = N[i][0];
         }
      }
   }

	LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
	LOGMSG(DBG_DEBUG, "Ray casting finished! Found " << *cnt << " coordinates");
}


bool RayCast2D::rayCastFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, const unsigned int beam, double coordinates[2], double normal[2])
{
   int xDim = grid->getCellsX();
   int yDim = grid->getCellsY();
   double cellSize = grid->getCellSize();

   double tr[2];
   sensor->getPosition(tr);

	double ray[2];
	double position[2];
	double position_prev[2];

	sensor->calcRay(beam, ray);
	ray[0] *= cellSize;
	ray[1] *= cellSize;

	// Interpolation weight
	double interp;

	double xmin   = ((double)(ray[0] > 0.0 ? 0 : (xDim-1)*cellSize) - tr[0]) / ray[0];
	double ymin   = ((double)(ray[1] > 0.0 ? 0 : (yDim-1)*cellSize) - tr[1]) / ray[1];
	double idxMin = max(xmin, ymin);
	idxMin        = max(idxMin, 0.0);

	double xmax   = ((double)(ray[0] > 0.0 ? (xDim-1)*cellSize : 0) - tr[0]) / ray[0];
	double ymax   = ((double)(ray[1] > 0.0 ? (yDim-1)*cellSize : 0) - tr[1]) / ray[1];
	double idxMax = min(xmax, ymax);

	if (idxMin >= idxMax)
		return false;

	double tsdf_prev;
	position[0] = tr[0] + idxMin * ray[0];
	position[1] = tr[1] + idxMin * ray[1];
	grid->interpolateBilinear(position, &tsdf_prev);

	bool found = false;
	for(int i=idxMin; i<idxMax; i++)
	{
		// calculate current position
		memcpy(position_prev, position, 2 * sizeof(*position));

		position[0] += ray[0];
		position[1] += ray[1];

		double tsdf;
		if (!grid->interpolateBilinear(position, &tsdf))
			continue;

		// check sign change
		if(tsdf_prev > 0 && tsdf_prev < 0.99999 && tsdf < 0)
		{
			interp = tsdf_prev / (tsdf_prev - tsdf);
			found = true;
			break;
		}

		tsdf_prev = tsdf;
	}

	if(!found) return false;

	coordinates[0] = position_prev[0] + ray[0] * interp;
	coordinates[1] = position_prev[1] + ray[1] * interp;

	if(!grid->interpolateNormal(coordinates, normal))
		return false;

	return true;
}

}
