#include <iostream>
#include <cmath>
#include <unistd.h>

#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/RayCastPolar2D.h"

#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

int main(void)
{
  LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

  // Initialization of TSD grid
  double dimX = 100.0;
  double dimY = 100.0;
  double cellSize = 0.01;
  unsigned int partitionSize = 20;

  TsdGrid* grid = new TsdGrid(dimX, dimY, cellSize, partitionSize);
  grid->setMaxTruncation(6.0*cellSize);

  // Translation of sensor
  double tx = dimX/2.0;
  double ty = dimY/2.0;

  // Rotation about z-axis of sensor
  double phi = 0.0 * M_PI / 180.0;
  double tf[9] = {cos(phi), -sin(phi), tx,
      sin(phi),  cos(phi), ty,
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
    // circular structure
    data[i] = maxRange / 2.0;
    mask[i] = true;

#if 0
    // plain wall
    mask[i] = false;
    if(i>0 && i<beams-1)
    {
      double theta = angularRes*((double)i)+minPhi;
      data[i] = maxRange / 3.0 / sin(theta);
      mask[i] = true;
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
  grid->push(&sensor);

  rayCaster.calcCoordsFromCurrentView(grid, &sensor, coords, normals, &cnt);
  LOGMSG(DBG_DEBUG, "Found " << cnt/2 << " coordinate tuples");


  // Initialization of 2D viewer
  unsigned int w = grid->getCellsX();
  unsigned int h = grid->getCellsY();
  double ratio = double(w)/double(h);
  unsigned screen_width = 600;
  unsigned screen_height = (unsigned int)(((double)screen_width)/ratio);

  LOGMSG(DBG_DEBUG, "Display map " << screen_width << "x" << screen_height);
  Obvious2D viewer(screen_width, screen_height, "tsd_grid_test");

  unsigned char* image = new unsigned char[3 * screen_width * screen_height];
  grid->grid2ColorImage(image, screen_width, screen_height);
  /*for(unsigned int i=0; i<cnt; i+=2)
  {
    if(i%100==0) cout << " x: " << coords[i] << " y: " << coords[i+1] << " d:" << sqrt(coords[i]*coords[i] + coords[i+1]*coords[i+1]) << endl;
    int x = (coords[i] + tx) / grid->getMaxX() * screen_width;
    int y = (coords[i+1] + ty) / grid->getMaxY() * screen_height;
    unsigned int idx = y*w + x;
    image[3*idx] = 255;
    image[3*idx+1] = 255;
    image[3*idx+2] = 255;
  }*/


  while(viewer.isAlive())
  {
    viewer.draw(image, screen_width, screen_height, 3, 0, 0);
    usleep(200000);
  }

  //serializePPM("/tmp/tsd_grid.pgm", image, w, h, true);

  delete [] image;

  delete [] coords;
  delete [] normals;
}


