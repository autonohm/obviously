#include <iostream>
#include <cmath>
#include <unistd.h>

#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"

#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

int main(void)
{
  LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

  // Initialization of TSD grid
  double cellSize = 0.01;

  TsdGrid* grid = new TsdGrid(cellSize, LAYOUT_32x32, LAYOUT_8192x8192);
  grid->setMaxTruncation(6.0*cellSize);

  // Translation of sensor
  double t[2];
  grid->getCentroid(t);

  // Rotation about z-axis of sensor
  double phi = 0.0 * M_PI / 180.0;
  double tf[9] = {cos(phi), -sin(phi), t[0],
      sin(phi),  cos(phi), t[1],
      0,         0,        1};
  Matrix T(3, 3);
  T.setData(tf);


  // Sensor initialization
  int beams = 1081;
  double angularRes = deg2rad(0.25);
  double minPhi     = deg2rad(-95.0);
  double maxRange   = 30.0;

  SensorPolar2D sensor(beams, angularRes, minPhi, maxRange);

  // Sample data, to be replaced with real measurements
  double* data = new double[beams];
  bool* mask = new bool[beams];
  for(int i=0; i<beams; i++)
  {
#if 1
    // circular structure
    data[i] = maxRange / 2.0;
    mask[i] = true;
#else
    // plain wall
    mask[i] = false;
    data[i] = NAN;
    double theta = angularRes*((double)i)+minPhi;
    if(theta<0)
    {
      double dist = abs(maxRange / 2.0 / sin(theta));
      if(dist<maxRange)
      {
        data[i] = dist;
        mask[i] = true;
      }
    }
#endif
  }
  sensor.setRealMeasurementData(data);
  sensor.setRealMeasurementMask(mask);
  delete[] data;
  delete[] mask;

  RayCastPolar2D rayCaster;
  double* coords = new double[beams*2];
  double* normals = new double[beams*2];

  unsigned int cnt;
  sensor.transform(&T);
  grid->pushTree(&sensor);

  rayCaster.calcCoordsFromCurrentView(grid, &sensor, coords, normals, &cnt);
  LOGMSG(DBG_DEBUG, "Found " << cnt/2 << " coordinate tuples");


  // Initialization of 2D viewer
  double ratio = double(grid->getCellsX())/double(grid->getCellsY());
  unsigned w = 600;
  unsigned h = (unsigned int)(((double)w)/ratio);

  LOGMSG(DBG_DEBUG, "Display map " << w << "x" << h);
  Obvious2D viewer(w, h, "tsd_grid_test");

  unsigned char* image = new unsigned char[3 * w * h];
  grid->grid2ColorImage(image, w, h);

  // Debug output
  for(unsigned int i=0; i<cnt; i+=100)
    cout << " x: " << coords[i] << " y: " << coords[i+1] << " d:" << sqrt(coords[i]*coords[i] + coords[i+1]*coords[i+1]) << endl;

  while(viewer.isAlive())
  {
    viewer.draw(image, w, h, 3, 0, 0);
    usleep(200000);
  }

  serializePPM("/tmp/tsd_grid.pgm", image, w, h, true);

  delete [] image;
  delete [] coords;
  delete [] normals;
}


