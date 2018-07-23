#include <iostream>
#include <sstream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obdevice/SickLMS100.h"

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/reconstruct/grid/RayCastAxisAligned2D.h"
#include "obvision/registration/icp/icp_def.h"

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
}

void callbackSerializeTsdGrid()
{
  //_grid->serialize("/tmp/tsdgrid.dat");
  cout << "WARNING: re-implement this method" << endl;
}

int main(int argc, char* argv[])
{
  LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

  // Initialization of TSD grid
  const double cellSize = 0.03;

  // choose estimator type
  enum Est{PTP, PTL};
  Est estType;
  if (argc >=1)
    estType = PTL;
  else
    estType = PTP;

  _grid = new TsdGrid(cellSize, LAYOUT_16x16, LAYOUT_1024x1024);
  _grid->setMaxTruncation(4.0*cellSize);


  // Initialization of 2D viewer
  // image display is only possible for image dimensions divisible by 4
  const unsigned int w = _grid->getCellsX();
  const unsigned int h = _grid->getCellsY();
  double ratio = double(w)/double(h);
  unsigned int screen_width = 700;
  unsigned int screen_height = (unsigned int)(((double)screen_width)/ratio);
  //unsigned int screen_width = w;
  //unsigned int screen_height = h;
  unsigned char* image = new unsigned char[3*screen_width*screen_height];
  Obvious2D viewer(screen_width, screen_height, "tsd_grid_lms100");
  viewer.registerKeyboardCallback('s', callbackSerializeTsdGrid);

  // Translation of sensor
  double tx = _grid->getMaxX()/2.0;
  double ty = _grid->getMaxY()/2.0;

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
  //double  minPhi     = lms.getStartAngle();
  double minPhi = deg2rad(-135.0);

  cout << "Rays: " << rays << " angular resolution: " << angularRes << " " << " min Phi: " << minPhi << endl;
  SensorPolar2D sensor(rays, angularRes, minPhi, 200.0, 0.0);
  sensor.transform(&Tinit);

  RayCastPolar2D rayCaster;
  double* mCoords = new double[rays*2];
  double* mNormals = new double[rays*2];

  double* sCoords = new double[rays*2];
  double* sDists = new double[rays];
  bool* sMask = new bool[rays];
  unsigned int sSize;

  // Compose ICP modules
  int iterations                 = 25;
  PairAssignment* assigner       = (PairAssignment*)  new FlannPairAssignment(2, 0.0, true);
  OutOfBoundsFilter2D* filterBounds = new OutOfBoundsFilter2D(_grid->getMinX(), _grid->getMaxX(), _grid->getMinY(), _grid->getMaxY());
  filterBounds->setPose(&Tinit);
  assigner->addPreFilter(filterBounds);
  DistanceFilter* filterDist = new DistanceFilter(2.0, 0.01, iterations-3);
  assigner->addPostFilter(filterDist);

  // choose estimator
  IRigidEstimator* estimator;
  if (estType == PTP)
    estimator     = (IRigidEstimator*) new ClosedFormEstimator2D();
  else
    estimator    = (IRigidEstimator*) new PointToLine2DEstimator();

  Icp* icp = new Icp(assigner, estimator);
  icp->setMaxRMS(0.0);
  icp->setMaxIterations(iterations);
  icp->setConvergenceCounter(iterations);
  //icp->setAssignmentCallback(callbackAssignment);

  // Set first model
  lms.grab();
  sensor.setRealMeasurementData(lms.getRanges());


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
    LOGMSG(DBG_DEBUG, "Raycast resulted in " << mSize << " coordinates");

    //double* coords = lms.getCoords();
    double* ranges = lms.getRanges();

    sSize = 0;
    for(int i=0; i<rays; i++)
    {
      sMask[i] = false;
      if(fabs(ranges[i])>0.01)
      {
        //sCoords[2*sSize] = coords[2*i];
        //sCoords[2*sSize+1] = coords[2*i+1];
        sCoords[2*sSize] = -ranges[i] * sin(i*angularRes-minPhi);
        sCoords[2*sSize+1] = ranges[i] * cos(i*angularRes-minPhi);
        sDists[sSize] = ranges[i];
        sMask[i] = true;
        sSize++;
      }
    }


    Matrix* M = new Matrix(mSize/2, 2, mCoords);
    Matrix* N = new Matrix(mSize/2, 2, mNormals);
    Matrix* S = new Matrix(sSize, 2, sCoords);

    icp->reset();
    icp->setModel(M, N);
    icp->setScene(S);

    double rms;
    unsigned int pairs;
    unsigned int it;

    icp->iterate(&rms, &pairs, &it);
    LOGMSG(DBG_DEBUG, "ICP result - RMS: " << rms << " pairs: " << pairs << " iterations: " << it << endl;)

    Matrix T = icp->getFinalTransformation();

    sensor.setRealMeasurementData(ranges);
    sensor.setRealMeasurementMask(sMask);
    sensor.transform(&T);


    Matrix TSensor = sensor.getTransformation();
    TSensor.print();
    filterBounds->setPose(&TSensor);

    double poseX  = TSensor(0,2);
    double poseY  = TSensor(1,2);
    double curPhi = acos(TSensor(0,0));

    double deltaX   = poseX - lastX;
    double deltaY   = poseY - lastY;
    double deltaPhi = fabs(curPhi - lastPhi);
    double sqrt_delta = sqrt(deltaX*deltaX + deltaY*deltaY);

    cout << "Phi: " << curPhi << " X: " << poseX << " Y: " << poseY << endl;


    if (initCount < 5 || sqrt_delta > 0.05 || deltaPhi > 0.05)
    {
      //sensor.setRealMeasurementAccuracy(sensor.getRealMeasurementAccuracy());
      _grid->push(&sensor);
      LastScanPose = T;
      //std::cout << "Pushed to grid" << std::endl;
      if(initCount < 5) initCount++;
      lastPhi = curPhi;
      lastX = poseX;
      lastY = poseY;
    }

    // Visualize data
    _grid->grid2ColorImage(image, screen_width, screen_height);
    memset(image, 0, screen_width*screen_height*3*sizeof(unsigned char));

    unsigned int mapSize;
    double* map      = new double[_grid->getCellsX()*_grid->getCellsY()*2];
    RayCastAxisAligned2D rayCasterMap;
    rayCasterMap.calcCoords(_grid, map, NULL, &mapSize);
    for(unsigned int i=0; i<mapSize/2; i++)
    {
      double x = map[2*i];
      double y = map[2*i+1];
      int u = x / cellSize * screen_width / w;
      int v = y / cellSize * screen_height / h;
      if(u>0 && u<(int)screen_width && v>0 && v<(int)screen_height)
      {
        int idx = 3*(v*screen_width+u);
        image[idx] = 255;
        image[idx+1] = 255;
        image[idx+2] = 255;
      }
    }
    delete [] map;

    // Plot scene
    Matrix* Sh = new Matrix(3, sSize);
    for(unsigned int i=0; i<sSize; i++)
    {
      (*Sh)(0, i) = sCoords[2*i];
      (*Sh)(1, i) = sCoords[2*i+1];
      (*Sh)(2, i) = 1.0;
    }
    *Sh = TSensor * *Sh;
    for(unsigned int i=0; i<sSize; i++)
    {
      int u = (*Sh)(0, i) / cellSize * screen_width / w;
      int v = (*Sh)(1, i) / cellSize * screen_height / h;
      if(u>0 && u<(int)screen_width && v>0 && v<(int)screen_height)
      {
        int idx = 3*(v*screen_width+u);
        image[idx] = 255;
        image[idx+1] = 0;
        image[idx+2] = 0;
      }
    }
    delete Sh;

    // Plot pose
    int uSensor = TSensor(0,2) / cellSize / w * screen_width;
    int vSensor = TSensor(1,2) / cellSize / h * screen_height;
    int idx = 3*(vSensor*screen_width+uSensor);
    image[idx] = 0;
    image[idx+1] = 255;
    image[idx+2] = 255;
    image[idx+3] = 0;
    image[idx+4] = 255;
    image[idx+5] = 255;

    viewer.draw(image, screen_width, screen_height, 3, 0, 0);

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


