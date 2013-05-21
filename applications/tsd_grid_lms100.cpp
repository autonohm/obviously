#include <iostream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obdevice/SickLMS100.h"

#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/RayCast2D.h"
#include "obvision/icp/icp_def.h"

#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

int main(void)
{
	LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

	// Initialization of TSD grid
	double dimX = 10.0;
	double dimY = 10.0;
	double cellSize = 0.05;

   TsdGrid* grid = new TsdGrid(dimX, dimY, cellSize);
   grid->setMaxTruncation(6.0*cellSize);


   // Initialization of 2D viewer
   unsigned int w = grid->getCellsX();
   unsigned int h = grid->getCellsY();
   unsigned char* image = new unsigned char[w * h];
   Obvious2D viewer(4*w, 4*h, "tsd_grid_test");


   // Translation of sensor
   double tx = dimX/2.0;
   double ty = dimY/2.0;

	// Rotation about z-axis of sensor
	double phi = 0.0 * M_PI / 180.0;
	double tf[9] = {cos(phi), -sin(phi), tx,
			          sin(phi),  cos(phi), ty,
			          0,         0,        1};
	Matrix Tinit(3, 3);
	Tinit.setData(tf);


	// Sensor initialization
	SickLMS100 lms;
   int     rays       = lms.getNumberOfRays();
   double  angularRes = lms.getAngularRes();
   double  minPhi     = lms.getStartAngle();

   cout << "Rays: " << rays << " angular resolution: " << angularRes << " " << " min Phi: " << minPhi << endl;
	SensorPolar2D sensor(rays, angularRes, minPhi);


	RayCast2D rayCaster;
   double* mCoords = new double[rays*2];
   double* mNormals = new double[rays*2];


   // Compose ICP modules
   int iterations                 = 30;
   PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
   IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(0.5, 0.01, iterations);
   assigner->addPostFilter(filterD);
   IRigidEstimator* estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();
	Icp* icp = new Icp(assigner, estimator);
   icp->setMaxRMS(0.0);
   icp->setMaxIterations(iterations);

   // Set first model
   lms.grab();
   sensor.setRealMeasurementData(lms.getRanges());
   sensor.transform(&Tinit);
   grid->push(&sensor);

	while(viewer.isAlive())
	{
	   lms.grab();

	   unsigned int mSize = rays;
	   rayCaster.calcCoordsFromCurrentView(grid, &sensor, mCoords, mNormals, &mSize);
	   LOGMSG(DBG_DEBUG, "Raycast resulted in " << mSize << " coordinates");

	   double* sCoords = lms.getCoords();

	   gsl_matrix_view model = gsl_matrix_view_array(mCoords, mSize, 2);
	   gsl_matrix_view scene = gsl_matrix_view_array(sCoords, rays, 2);

	   icp->reset();
	   icp->setModel(&model.matrix);
	   icp->setScene(&scene.matrix);

	   double rms;
      unsigned int pairs;
      unsigned int it;
      icp->iterate(&rms, &pairs, &it);
      LOGMSG(DBG_DEBUG, "ICP result - RMS: " << rms << " pairs: " << pairs << " iterations: " << it << endl;)

      Matrix* T = icp->getFinalTransformation();
      //T->print();

      Matrix T2D(3, 3);
      T2D[0][0] = (*T)[0][0]; T2D[0][1] = (*T)[0][1]; T2D[0][2] = (*T)[0][3];
      T2D[1][0] = (*T)[1][0]; T2D[1][1] = (*T)[1][1]; T2D[1][2] = (*T)[1][3];
      T2D[2][0] = (*T)[3][0]; T2D[2][1] = (*T)[3][1]; T2D[2][2] = (*T)[3][3];

      //T2D.print();
      Matrix Tinv = T2D.getInverse();

      sensor.setRealMeasurementData(lms.getRanges());
	   sensor.transform(&T2D);
	   Matrix* Tsensor = sensor.getPose();
	   //Tsensor->print();
	   grid->push(&sensor);

	   grid->grid2GrayscaleImage(image);
	   /*for(int i=0; i<mSize; i+=2)
	   {
	      int x = ((double)(mCoords[i] + tx)) / cellSize;
	      int y = ((double)(mCoords[i+1] + ty)) / cellSize;
	      image[y*w + x] = 0;
	   }*/

	   viewer.draw(image, w, h, 1, 0, 0);
	}

	serializePGM("/tmp/tsd_grid.pgm", image, w, h, true);

	delete [] image;
   delete [] mCoords;
   delete [] mNormals;
}


