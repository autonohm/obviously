#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include "obvision/reconstruct/RayCast3D.h"
#include "obcore/base/Logger.h"

using namespace std;
using namespace obvious;

int main(void)
{
  LOGMSG_CONF("tsd_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);

  int rows, cols;

  // translation of sensor
  double tx = 0.5;
  double ty = 0.5;
  double tz = 0.0;

  // rotation about y-axis of sensor
  double theta = -10 * M_PI / 180;

  double tf[16]={cos(theta),  0, sin(theta), tx,
                 0,           1, 0,          ty,
                 -sin(theta), 0, cos(theta), tz,
                 0,           0, 0,          1};
  Matrix T(4, 4);
  T.setData(tf);

  rows = 480;
  cols = 640;

  double*** buf;
  System<double>::allocate (cols, rows, 3, buf);

  // Setup synthetic perspective projection
  double su = 500;
  double sv = 500;
  double tu = 320;
  double tv = 240;
  double PData[12]  = {su, 0, tu, 0, 0, sv, tv, 0, 0, 0, 1, 0};

  double voxelSize = 0.02;

  SensorProjective3D sensor(cols, rows, PData, voxelSize);
  sensor.transform(&T);

  TsdSpace space(1.0, 1.0, 1.0, voxelSize);
  space.setMaxTruncation(3.0*voxelSize);

  double distZ[cols*rows];

  // Background with distance = 0.8m -> s=0.8
  for(int u=0; u<cols; u++)
    for(int v=0; v<rows; v++)
    {
      double s = 0.8;
      double x = s*(((double)u) - tu) / su;
      double y = s*(((double)v) - tv) / sv;
      double z = s;
      buf[u][v][0] = x;
      buf[u][v][1] = y;
      buf[u][v][2] = z;
      distZ[v*cols+u] = sqrt(x*x+y*y+z*z);
    }

  // Centered square with distance = 0.5m -> s=0.5
  for(int u=cols/4; u<3*cols/4; u++)
    for(int v=rows/4; v<3*rows/4; v++)
    {
      double s = 0.5;
      double x = s*(((double)u) - tu) / su;
      double y = s*(((double)v) - tv) / sv;
      double z = s;
      buf[u][v][0] = x;
      buf[u][v][1] = y;
      buf[u][v][2] = z;
      distZ[v*cols+u] = sqrt(x*x+y*y+z*z);
    }

  /*for(int u=0; u<cols; u++)
	   for(int v=0; v<rows; v++)
	   {
	      distZ[v*cols+u] -= v * 0.0004;
	   }*/

  unsigned char* texture = new unsigned char[cols*rows*3];
  for(int u=0; u<cols; u++)
     for(int v=0; v<rows; v++)
     {
       unsigned int idx = (v*cols+u)*3;
       texture[idx]   = u;
       texture[idx+1] = u+v;
       texture[idx+2] = u+2*v;
     }

  sensor.setRealMeasurementData(distZ);
  sensor.setRealMeasurementRGB(texture);
  space.push(&sensor);

  unsigned char* buffer = new unsigned char[space.getXDimension()*space.getYDimension()*3];
  for(unsigned int i=0; i<space.getZDimension(); i++)
  {
    char path[64];
    sprintf(path, "/tmp/slice%04d.ppm", i);
    space.buildSliceImage(i, buffer);
    serializePPM(path, buffer, space.getXDimension(), space.getYDimension(), 0);
  }
  delete[] buffer;

  RayCast3D raycaster(&space);
  unsigned int cnt;
  double* cloud = new double[rows*cols*3];
  double* normals = new double[rows*cols*3];
  unsigned char* rgb = new unsigned char[rows*cols*3];
  raycaster.calcCoordsFromCurrentPose(&sensor, cloud, normals, rgb, &cnt);

  cout << "getModel returned with " << cnt << " coordinates" << endl;

  VtkCloud vcloud;
  vcloud.setCoords(cloud, cnt/3, 3, normals);
  if(sensor.hasRealMeasurmentRGB()) vcloud.setColors(rgb, cnt/3, 3);

  Obvious3D viewer("TSD Cloud");
  viewer.showSensorPose(T);
  viewer.showAxes(true);
  viewer.addCloud(&vcloud);
  viewer.startRendering();

  delete [] cloud;
  delete [] normals;
  delete [] rgb;
  delete [] texture;
}


