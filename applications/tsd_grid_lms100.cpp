#include <iostream>
#include <sstream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obdevice/SickLMS100.h"

#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/RayCastPolar2D.h"
#include "obvision/icp/icp_def.h"

#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

TsdGrid* _grid;

void callbackAssignment(double** m, double** s, unsigned int size)
{
  static unsigned int cnt = 0;
  char filename[64];
  sprintf(filename, "/tmp/assignment%05d.dat", cnt++);

  ofstream f;
  f.open(filename);


  for(unsigned int i=0; i<size; i++)
  {
    f << m[i][0] << " " << m[i][1] << " " << s[i][0] << " " << s[i][1] << endl;
  }

  f.close();

  delete [] m;
  delete [] s;
}

void callbackSerializeTsdGrid()
{
  _grid->serialize("/tmp/tsdgrid.dat");
}

int main(int argc, char* argv[])
{
  LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_off, DBG_DEBUG, DBG_DEBUG);

  // Initialization of TSD grid
  const double dimX = 36.0;
  const double dimY = 36.0;
  const double cellSize = 0.06;

  // choose estimator type
  enum Est{PTP, PTL};
  Est _estType;
  if (argc >=1)
    _estType = PTL;
  else
    _estType = PTP;

  _grid = new TsdGrid(dimX, dimY, cellSize);
  _grid->setMaxTruncation(4.0*cellSize);


  // Initialization of 2D viewer
  // image display is only possible for image dimensions divisible by 4
  const unsigned int w = _grid->getCellsX();
  const unsigned int h = _grid->getCellsY();
  unsigned char* image = new unsigned char[3*w*h];
  double ratio = double(w)/double(h);
  double screen_width = 1000;
  Obvious2D viewer(screen_width, screen_width/ratio, "tsd_grid_lms100");
  viewer.registerKeyboardCallback('s', callbackSerializeTsdGrid);

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

  Matrix LastScanPose = Tinit;

  // Sensor initialization
  SickLMS100 lms;
  int     rays       = lms.getNumberOfRays();
  double  angularRes = lms.getAngularRes();
  double  minPhi     = lms.getStartAngle();

  cout << "Rays: " << rays << " angular resolution: " << angularRes << " " << " min Phi: " << minPhi << endl;
  SensorPolar2D sensor(rays, angularRes, minPhi);


  RayCastPolar2D rayCaster;
  double* mCoords = new double[rays*2];
  double* mNormals = new double[rays*2];
  double* map      = new double[_grid->getCellsX()*_grid->getCellsY()*2];

  // Compose ICP modules
  int iterations                 = 25;
  PairAssignment* assigner       = (PairAssignment*)  new AnnPairAssignment(2);
  OutOfBoundsFilter2D* filterBounds = new OutOfBoundsFilter2D(_grid->getMinX(), _grid->getMaxX(), _grid->getMinY(), _grid->getMaxY());
  filterBounds->setPose(&Tinit);
  assigner->addPreFilter(filterBounds);
  DistanceFilter* filterDist = new DistanceFilter(2.0, 0.01, iterations-3);
  assigner->addPostFilter(filterDist);

//  // choose estimator
//  IRigidEstimator* estimator;
//  if (_estType == PTP)
//    estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();
//  else
//    IRigidEstimator* estimator    = (IRigidEstimator*) new PointToLine2DEstimator();

  IRigidEstimator* estimator    = (IRigidEstimator*) new ClosedFormEstimator2D();


  Icp* icp = new Icp(assigner, estimator);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);
  icp->setConvergenceCounter(iterations);
  //icp->setAssignmentCallback(callbackAssignment);

  // Set first model
  lms.grab();
  sensor.setRealMeasurementData(lms.getRanges());
  sensor.transform(&Tinit);

  //sensor.setRealMeasurementAccuracy(sensor.getRealMeasurementAccuracy());

  //for(unsigned int i=0; i<10; i++)
  _grid->push(&sensor);

  double lastPhi = 0;
  double lastX = 0;
  double lastY = 0;

  unsigned int initCount = 0;
  while(viewer.isAlive())
  {
    lms.grab();

    unsigned int mSize = 0;
    rayCaster.calcCoordsFromCurrentView(_grid, &sensor, mCoords, mNormals, &mSize);
//    LOGMSG(DBG_DEBUG, "Raycast resulted in " << mSize << " coordinates");

    double* sCoords = lms.getCoords();

    Matrix* M = new Matrix(mSize/2, 2, mCoords);
    Matrix* N = new Matrix(mSize/2, 2, mNormals);
    Matrix* S = new Matrix(rays, 2, sCoords);

    icp->reset();
    icp->setModel(M, N);
    icp->setScene(S);

    double rms;
    unsigned int pairs;
    unsigned int it;

    icp->iterate(&rms, &pairs, &it);
//    LOGMSG(DBG_DEBUG, "ICP result - RMS: " << rms << " pairs: " << pairs << " iterations: " << it << endl;)

    Matrix* T = icp->getFinalTransformation();

    Matrix* Pose = sensor.getPose();
    double poseX  = (*Pose)[0][2];
    double poseY  = (*Pose)[1][2];
    double curPhi = acos((*Pose)[0][0]);

    double deltaX   = poseX - lastX;
    double deltaY   = poseY - lastY;
    double deltaPhi = fabs(curPhi - lastPhi);
    double sqrt_delta = sqrt(deltaX*deltaX + deltaY*deltaY);

    filterBounds->setPose(sensor.getPose());

    /*T->print();*/
    //usleep(2000000);

    sensor.setRealMeasurementData(lms.getRanges());
    sensor.transform(T);

    if (initCount < 5 || sqrt_delta > 0.05 || deltaPhi > 0.05)
    {
      //sensor.setRealMeasurementAccuracy(sensor.getRealMeasurementAccuracy());
      _grid->push(&sensor);
      LastScanPose = *T;
      //std::cout << "Pushed to grid" << std::endl;
      if(initCount < 5) initCount++;
      lastPhi = curPhi;
      lastX = poseX;
      lastY = poseY;
    }

    // Visualize data
    _grid->grid2ColorImage(image);
    unsigned int mapSize;
    rayCaster.calcCoordsAligned(_grid, map, NULL, &mapSize);
    for(unsigned int i=0; i<mapSize/2; i++)
    {
      double x = map[2*i];
      double y = map[2*i+1];
      int u = x / cellSize;
      int v = h-(y / cellSize);
      if(u>0 && u<(int)w && v>0 && v<(int)h)
      {
        int idx = 3*(v*w+u);
        image[idx] = 0;
        image[idx+1] = 0;
        image[idx+2] = 0;
      }
    }
    double position[2];
    sensor.getPosition(position);
    int x, y;
    double dx, dy;
    _grid->coord2Cell(position, &x, &y, &dx, &dy);
    int idx = ((h-y)*w+x);
    image[3*idx]   = 255;
    image[3*idx+1] = 0;
    image[3*idx+2] = 0;
    viewer.draw(image, w, h, 3, 0, 0);

    delete M;
    delete N;
    delete S;
  }

  //output for grayscale data
  //serializePGM("/tmp/tsd_grid.pgm", image, w, h, true);

  delete [] image;
  delete [] mCoords;
  delete [] mNormals;
}


