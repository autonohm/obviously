/**
 * @date 26.9.2013
 * @author Stefan May
 */

#include <cstdio>

#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include "obvision/reconstruct/space/SensorProjective3D.h"
#include "obvision/reconstruct/space/SensorPolar3D.h"
#include "obvision/reconstruct/space/RayCast3D.h"

using namespace obvious;

#define LASERSCANNER 0
#define KINECT 1

void func(){};

int main(int argc, char* argv[])
{
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <sensor>" << endl;
    cout << " sensor: 0=Laser scanner, 1=Kinect" << endl;
    exit(1);
  }

  int sensortype = atoi(argv[1]);

  unsigned int subsamplingW = 1;
  unsigned int subsamplingH = 1;
  double factorStart        = 0.8;
  double factorEnd          = 1.0;

  Sensor* sensor;

  if(sensortype==KINECT)
  {
    // Projection matrix (needs to be determined by calibration)
    // ------------------------------------------------------------------
    double Pdata[12] = {585.05108211, 0.0, 315.83800193, 0.0, 0.0, 585.05108211, 242.94140713, 0., 0.0, 0.0, 1.0, 0.0};
    Matrix P(3, 4, Pdata);
    unsigned int cols = 640;
    unsigned int rows = 480;
    sensor = new SensorProjective3D(cols, rows, Pdata);
    subsamplingW = 64;
    subsamplingH = 48;
  }
  else
  {
    double thetaRes = deg2rad(0.25);
    double thetaMin = deg2rad(-135.0);
    double phiRes   = deg2rad(1.0);
    sensor = new SensorPolar3D(1081, thetaRes, thetaMin, phiRes);
    subsamplingH = 72;
    subsamplingW = 24;
  }

  unsigned int width = sensor->getWidth();
  unsigned int height = sensor->getHeight();

  cout << "Sensor width: " << width << ", height: " << height << endl;

  double** coordsStart;
  double** coordsEnd;
  System<double>::allocate(width*height, 3, coordsStart);
  System<double>::allocate(width*height, 3, coordsEnd);

  double* rays = new double[width*height*3];
  memset(rays, 0, width*height*3*sizeof(double));
  unsigned int size = 0;
  for(unsigned int w=0; w<width; w+=subsamplingW)
    for(unsigned int h=0; h<height; h+=subsamplingH)
    {
      double ray[3];
      sensor->calcRayFromCurrentPose(h, w, ray);

      double fStart = factorStart;
      double fEnd   = factorEnd;
      if(sensortype==KINECT)
      {
        fStart *= 1.0 / ray[2];
        fEnd *= 1.0 / ray[2];
      }

      coordsStart[size][0] = ray[0] * fStart;
      coordsStart[size][1] = ray[1] * fStart;
      coordsStart[size][2] = ray[2] * fStart;
      coordsEnd[size][0] = ray[0] * fEnd;
      coordsEnd[size][1] = ray[1] * fEnd;
      coordsEnd[size][2] = ray[2] * fEnd;
      size++;
    }

  // Displaying stuff
  // ------------------------------------------------------------------
  double bg[3] = {1.0, 1.0, 1.0};
  Obvious3D viewer3D("Visualize ray casting for kinect", 1024, 768, 0, 0, bg);
  viewer3D.addLines(coordsStart, coordsEnd, size);

  if(sensortype==KINECT)
  {
    double** projectionPlane;
    System<double>::allocate(4, 3, projectionPlane);
    sensor->calcRayFromCurrentPose(0, 0, projectionPlane[0]);
    sensor->calcRayFromCurrentPose(height-subsamplingH, 0, projectionPlane[1]);
    sensor->calcRayFromCurrentPose(height-subsamplingH, width-subsamplingW, projectionPlane[2]);
    sensor->calcRayFromCurrentPose(0, width-subsamplingW, projectionPlane[3]);
    for(unsigned int i=0; i<3; i++)
    {
      projectionPlane[0][i] /= (projectionPlane[0][2]/(factorStart-0.01));
      projectionPlane[1][i] /= (projectionPlane[1][2]/(factorStart-0.01));
      projectionPlane[2][i] /= (projectionPlane[2][2]/(factorStart-0.01));
      projectionPlane[3][i] /= (projectionPlane[3][2]/(factorStart-0.01));
    }

    viewer3D.addPlane(projectionPlane[1], projectionPlane[0], projectionPlane[2]);

    System<double>::deallocate(projectionPlane);
  }
  else
  {
    double center[3] = {0.0, 0.0, 0.0};
    viewer3D.addSphere(center, factorStart-0.01);
    //double light[3] = {1.0, 1.0, 1.0};
    //viewer3D.addLight(light, center);
  }

  viewer3D.startRendering();

  System<double>::deallocate(coordsStart);
  System<double>::deallocate(coordsEnd);

  delete sensor;

}
