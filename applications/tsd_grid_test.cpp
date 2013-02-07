#include <iostream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/RayCast2D.h"

using namespace std;
using namespace obvious;

int main(void)
{
	LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

	double* pcl        = NULL;
	int     beams      = 271;
	double  angularRes = 1.0;
	double  minPhi     = -45.0;

	// translation of sensor
	double dimX = 10.0;
	double dimY = 10.0;
	double cellSize = 0.02;
	double tx = dimX/2.0;
	double ty = dimY/2.0;

	// rotation about z-axis of sensor
	double phi = 20.0 * M_PI / 180.0;
	double tf[9] = {cos(phi), -sin(phi), tx,
			          sin(phi),  cos(phi), ty,
			          0,         0,        1};
	Matrix T(3, 3);
	T.setData(tf);
	SensorPolar2D sensor(beams, angularRes, minPhi);
	sensor.transform(&T);

   TsdGrid* grid = new TsdGrid(dimX, dimY, cellSize);
	grid->setMaxTruncation(6.0*cellSize);

	grid->push(&sensor);

	RayCast2D rayCaster;
	double* coords = new double[beams*2];
	double* normals = new double[beams*2];
	unsigned int cnt;
	rayCaster.calcCoordsFromCurrentView(grid, &sensor, coords, normals, &cnt);
	LOGMSG(DBG_DEBUG, "Found " << cnt << " coordinate tuples");
	unsigned char* image = new unsigned char[grid->getCellsX() * grid->getCellsY()];
	grid->grid2GrayscaleImage(image);
	for(int i=0; i<cnt; i+=2)
	{
	   int x = ((double)(coords[i] + tx)) / cellSize;
	   int y = ((double)(coords[i+1] + ty)) / cellSize;
	   image[y*grid->getCellsX() + x] = 128;
	}
	serializePGM("/tmp/tsd_grid.pgm", image, grid->getCellsX(), grid->getCellsY(), true);
	delete [] image;

   delete [] coords;
   delete [] normals;
}


