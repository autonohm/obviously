#include <iostream>
#include <math.h>
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obvision/reconstruct/TsdGrid.h"
#include "obvision/reconstruct/RayCast2D.h"

#include "obgraphic/Obvious2D.h"

using namespace std;
using namespace obvious;

int main(void)
{
  LOGMSG_CONF("tsd_grid_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

  // Initialization of TSD grid
  double dimX = 24.0;
  double dimY = 24.0;
  double cellSize = 0.04;

  TsdGrid* grid = new TsdGrid(dimX, dimY, cellSize);
  grid->setMaxTruncation(6.0*cellSize);


  // Initialization of 2D viewer
  unsigned int w = grid->getCellsX();
  unsigned int h = grid->getCellsY();
  unsigned char* image = new unsigned char[w * h];
  double ratio = double(w)/double(h);
  double screen_width = 600;
  Obvious2D viewer(screen_width, screen_width/ratio, "tsd_grid_test");


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
  int beams = 37;
  double angularRes = deg2rad(10.0);
  double minPhi     = deg2rad(-180.0);

  SensorPolar2D sensor(beams, angularRes, minPhi);

  // Sample data, to be replaced with real measurements
  double* data = new double[beams];
  bool* mask = new bool[beams];
  for(int i=0; i<beams; i++)
  {
    // circular structure
    data[i] = std::min(dimX, dimY) / 3.0;
    mask[i] = true;

#if 0
    // plain wall
    mask[i] = false;
    if(i>0 && i<beams-1)
    {
      double theta = angularRes*((double)i)+minPhi;
      data[i] = std::min(dimX, dimY) / 3.0/sin(theta);
      mask[i] = true;
    }
#endif
  }
  sensor.setRealMeasurementData(data);
  sensor.setRealMeasurementMask(mask);
  delete[] data;
  delete[] mask;

  RayCast2D rayCaster;
  double* coords = new double[beams*2];
  double* normals = new double[beams*2];

  unsigned int cnt;
  sensor.transform(&T);
  grid->push(&sensor);

  rayCaster.calcCoordsFromCurrentView(grid, &sensor, coords, normals, &cnt);
  LOGMSG(DBG_DEBUG, "Found " << cnt/2 << " coordinate tuples");


  grid->grid2GrayscaleImage(image);
  for(unsigned int i=0; i<cnt; i+=2)
  {
    cout << " x: " << coords[i] << " y: " << coords[i+1] << " d:" << sqrt(coords[i]*coords[i] + coords[i+1]*coords[i+1]) << endl;
    int x = round(((double)(coords[i] + tx)) / cellSize);
    int y = h-round(((double)(coords[i+1] + ty)) / cellSize);
    image[y*w + x] = 0;
  }

  while(viewer.isAlive())
  {
    viewer.draw(image, w, h, 1, 0, 0);
    usleep(200000);
  }

  serializePGM("/tmp/tsd_grid.pgm", image, w, h, true);

  delete [] image;
  delete [] coords;
  delete [] normals;
}


