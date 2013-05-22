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
   double dimX = 8.0;
   double dimY = 8.0;
   double cellSize = 0.05;

   TsdGrid* grid = new TsdGrid(dimX, dimY, cellSize);
   grid->setMaxTruncation(6.0*cellSize);


   // Initialization of 2D viewer
   // image display is only possible for image dimensions divisible by 4
   unsigned int w = grid->getCellsX();
   unsigned int h = grid->getCellsY();
   unsigned char* image = new unsigned char[3*w*h];
   double ratio = double(w)/double(h);
   double screen_width = 800;
   Obvious2D viewer(screen_width, screen_width/ratio, "tsd_grid_lms100");


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
   double* map      = new double[grid->getCellsX()*grid->getCellsY()*2];

   // Compose ICP modules
   int iterations                 = 50;
   PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
   IPostAssignmentFilter* filterD = (IPostAssignmentFilter*) new DistanceFilter(1.5, 0.01, iterations);
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

      Matrix* M = new Matrix(mSize, 2, mCoords);
      Matrix* S = new Matrix(rays, 2, sCoords);

      icp->reset();
      icp->setModel(M->getBuffer());
      icp->setScene(S->getBuffer());

      double rms;
      unsigned int pairs;
      unsigned int it;
      icp->iterate(&rms, &pairs, &it);
      LOGMSG(DBG_DEBUG, "ICP result - RMS: " << rms << " pairs: " << pairs << " iterations: " << it << endl;)

      Matrix* T = icp->getFinalTransformation();
      sensor.setRealMeasurementData(lms.getRanges());
      sensor.transform(T);
      Matrix* TSensor = sensor.getPose();
      Matrix TSensorInv = T->getInverse();

      grid->push(&sensor);
      grid->grid2ColorImage(image);
      unsigned int mapSize;
      rayCaster.calcCoordsAligned(grid, map, NULL, &mapSize);
      for(int i=0; i<mapSize/2; i++)
      {
         double x = map[2*i];
         double y = map[2*i+1];
         int u = x / cellSize;
         int v = h-(y / cellSize);
         if(u>0 && u<w && v>0 && v<h)
         {
            int idx = 3*(v*w+u);
            image[idx] = 0;
            image[idx+1] = 0;
            image[idx+2] = 0;
         }
      }

      delete M;
      delete S;
      viewer.draw(image, w, h, 3, 0, 0);
   }

   //output for grayscale data
   //serializePGM("/tmp/tsd_grid.pgm", image, w, h, true);

   delete [] image;
   delete [] mCoords;
   delete [] mNormals;
}


