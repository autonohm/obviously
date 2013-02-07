#include <iostream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include "obvision/reconstruct/TsdGrid.h"

using namespace std;
using namespace obvious;

int main(void)
{
	LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);

	double* pcl        = NULL;
	int     beams      = 541;
	double  angularRes = 0.5;
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

	unsigned char* image = new unsigned char[grid->getCellsX() * grid->getCellsY()];
	grid->grid2GrayscaleImage(image);
	serializePGM("/tmp/tsd_grid.pgm", image, grid->getCellsX(), grid->getCellsY(), true);
	delete [] image;
}


