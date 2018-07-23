#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/reconstruct/space/SensorProjective3D.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"
#include "obvision/reconstruct/space/SensorPolar2DWith3DPose.h"
#include "obcore/base/Logger.h"

#include "obcore/math/mathbase.h"

using namespace std;
using namespace obvious;

#define square 0

Obvious3D* _viewer;
SensorPolar2DWith3DPose* _sensor;
TsdSpace* _space;
VtkCloud* _vcloud;
VtkCloud* _vcloud2;
vector<double> _coords;
vector<unsigned char> _rgbs;

double NOISE_RANGE = 0.015;
double WALL_DISTANCE = 1;
int WOBBLE_RANGE_DEG = 5;
double Z_STEP_SIZE = 0.01 / 4;
double ANGULAR_RES_DEG = 5;

void generateSyntheticData(SensorPolar2DWith3DPose& sensor)
{

  int beams = sensor.getRealMeasurementSize();
  //double angularRes = sensor.getAngularResolution();
  //double minPhi = sensor.getPhiMin();
  double maxRange = sensor.getMaximumRange();
  //double minRange             = sensor.getMaximumRange();
  //double lowReflectivityRange = sensor.getLowReflectivityRange();

  // Sample data, to be replaced with real measurements
  double* data = new double[beams];
  unsigned char* rgb = new unsigned char[beams * 3];

  double coords[beams * 3];
  unsigned char rgb2[beams * 3];

  Matrix T = sensor.getTransformation();

  Matrix* rays = sensor.getNormalizedRayMap(1);

  for(int i = 0; i < beams; i++)
  {
    double noise = -NOISE_RANGE + ((double)rand() / RAND_MAX) * NOISE_RANGE * 2;
#if square
    double n[2] =
    { 0, 0};
    double n2[2] =
    { 0, 0};

    //wall on x
    n[0] = std::abs(WALL_DISTANCE / (*rays)(0, i));
    //wall on y
    n[1] = std::abs(WALL_DISTANCE / (*rays)(1, i));

    //take nearest wall for distance
    double min = n[0] < n[1] ? n[0] : n[1];
    min += noise;

    //check maxRange
    data[i] = min <= maxRange ? min : NAN;

    //wall on x
    n2[0] = std::abs((WALL_DISTANCE - 0.1) / (*rays)(0, i));
    //wall on y
    n2[1] = std::abs((WALL_DISTANCE - 0.1) / (*rays)(1, i));

    //take nearest wall for distance
    double min2 = n2[0] < n2[1] ? n2[0] : n2[1];
    min2 += noise;

#else

    double n[3];

    //wall on x
    n[0] = std::abs(WALL_DISTANCE * (*rays)(0, i));
    //wall on y
    n[1] = std::abs(WALL_DISTANCE * (*rays)(1, i));
    //wall on z
    n[2] = std::abs(WALL_DISTANCE * (*rays)(2, i));

    double phi = atan(n[2] / (sqrt(n[0] * n[0] + n[1] * n[1])));

    double min = 1 / cos(phi) + noise;
    double min2 = min + 0.05;
#endif

    //color it red
    rgb[i * 3 + 0] = 255;

    //check maxRange
    data[i] = min <= maxRange ? min : NAN;

    if(data[i] != NAN && 1)
    {
      _coords.push_back(T(0, 3) + min2 * (*rays)(0, i));
      _coords.push_back(T(1, 3) + min2 * (*rays)(1, i));
      _coords.push_back(T(2, 3) + min2 * (*rays)(2, i));
      _rgbs.push_back(0);
      _rgbs.push_back(0);
      _rgbs.push_back(255);  //blue
    }
  }

  sensor.setRealMeasurementData(data);
  sensor.setRealMeasurementRGB(rgb);
  sensor.setStandardMask();
}

void extractEulerAngleXYZ(Matrix t, double& rotXangle, double& rotYangle, double& rotZangle)
{
  rotXangle = atan2(-t(1, 2), t(2, 2));
  double cosYangle = sqrt(pow(t(0, 0), 2) + pow(t(0, 1), 2));
  rotYangle = atan2(t(0, 2), cosYangle);
  double sinXangle = sin(rotXangle);
  double cosXangle = cos(rotXangle);
  rotZangle = atan2(cosXangle * t(1, 0) + sinXangle * t(2, 0), cosXangle * t(1, 1) + sinXangle * t(2, 1));
}

void pushNewData()
{

  Matrix currentT = _sensor->getTransformation();
  double rotXangle, rotYangle, rotZangle;
  extractEulerAngleXYZ(currentT, rotXangle, rotYangle, rotZangle);

  rotYangle = rotYangle / M_PI * 180;
  rotYangle *= -1;
  rotYangle += -WOBBLE_RANGE_DEG + rand() % (WOBBLE_RANGE_DEG * 2 + 1);
  rotYangle = rotYangle * M_PI / 180;

  rotXangle = rotXangle / M_PI * 180;
  rotXangle *= -1;
  rotXangle += -WOBBLE_RANGE_DEG + rand() % (WOBBLE_RANGE_DEG * 2 + 1);
  rotXangle = rotXangle * M_PI / 180;

  double tf[16] = {cos(rotYangle), sin(rotYangle) * sin(rotXangle), sin(rotYangle) * cos(rotXangle), 0, 0,
      cos(rotXangle), -sin(rotXangle), 0, -sin(rotYangle), cos(rotYangle) * sin(rotXangle), cos(rotYangle)
          * cos(rotXangle), 0, 0, 0, 0, 1};

  Matrix T(4, 4);
  T.setData(tf);
  _sensor->transform(&T);

  currentT = _sensor->getTransformation();
  currentT(2, 3) += Z_STEP_SIZE;
  _sensor->setTransformation(currentT);

  generateSyntheticData(*_sensor);
  _space->pushForward(_sensor);
}

void _cbRegNewImage(void)
{
  pushNewData();
  std::cout << __PRETTY_FUNCTION__ << "get t" << std::endl;
  Matrix currentT = _sensor->getTransformation();
  std::cout << __PRETTY_FUNCTION__ << " shot t" << std::endl;
  _viewer->showSensorPose(currentT);


  // _vcloud2->setCoords(_coords.data(), _coords.size() / 3, 3, NULL);
  // _vcloud2->setColors(_rgbs.data(), _coords.size(), 3);
//return;
  unsigned int cnt;
std::cout << __PRETTY_FUNCTION__ << "allocate stuff" << std::endl;
  unsigned int cells = _space->getXDimension() * _space->getYDimension() * _space->getZDimension();
  double* coords = new double[cells * 3];
  double* normals = new double[cells * 3];
  unsigned char* rgb = new unsigned char[cells * 3];
  RayCastAxisAligned3D raycaster;
  std::cout << __PRETTY_FUNCTION__ << "raycast" << std::endl;
  raycaster.calcCoords(_space, coords, NULL, rgb, &cnt);
  //raycaster.calcCoords(_space, coords, NULL, NULL, &cnt);

  std::cout << __PRETTY_FUNCTION__ << " show coords" << std::endl;
  _vcloud->setCoords(coords, cnt / 3, 3, NULL);
  //_vcloud->setColors(rgb, cnt / 3, 3);
  _viewer->update();

 // delete[] coords;
 // delete[] normals;
}

int main(void)
{
  LOGMSG_CONF("tsd_test.log", Logger::file_off | Logger::screen_on, DBG_DEBUG, DBG_DEBUG);

  obfloat voxelSize = 0.01;
  _space = new TsdSpace(voxelSize, LAYOUT_8x8x8, 256, 256, 512);
  _space->setMaxTruncation(3.0 * voxelSize);

// translation of sensor
  obfloat tr[3];
  _space->getCentroid(tr);
  tr[2] = 0.3;

// rotation about y-axis of sensor
  double theta = 0 * M_PI / 180;

  double tf[16] = {cos(theta), 0, sin(theta), tr[0], 0, 1, 0, tr[1], -sin(theta), 0, cos(theta), tr[2], 0, 0, 0, 1};
  Matrix T(4, 4);
  T.setData(tf);

// Sensor initialization

  int beams = 360 / ANGULAR_RES_DEG;
  double angularResRad = deg2rad(ANGULAR_RES_DEG);
  double minPhi = deg2rad(-180.0);
  double maxRange = 3.0;
  double minRange = 0.3;
  double lowReflectivityRange = 0.5;

  _sensor = new SensorPolar2DWith3DPose(beams, angularResRad, minPhi, maxRange, minRange, lowReflectivityRange);

  _sensor->transform(&T);

  /*unsigned char* buffer = new unsigned char[space.getXDimension()*space.getYDimension()*3];
   for(unsigned int i=0; i<space.getZDimension(); i++)
   {
   char path[64];
   sprintf(path, "/tmp/slice%04d.ppm", i);
   _space->buildSliceImage(i, buffer);
   serializePPM(path, buffer, _space->getXDimension(), _space->getYDimension(), 0);
   }
   delete[] buffer;*/

  _vcloud = new VtkCloud;
  _vcloud2 = new VtkCloud;
  _viewer = new Obvious3D("TSD Cloud");

  _viewer->addAxisAlignedCube(0, _space->getMaxX(), 0, _space->getMaxY(), 0, _space->getMaxZ());
  _viewer->showAxes(true);
  _viewer->addCloud(_vcloud);
  _viewer->addCloud(_vcloud2);

  for(int i = 0; i < 150; ++i)
  {
    pushNewData();
    if(!(i % 100))
      cout << i << endl;
  }
  _space->serializeSliceImages(Z);
  cout << "finished" << endl;
  _viewer->registerKeyboardCallback("space", _cbRegNewImage, "Register new image");
  _viewer->startRendering();

//delete[] coords;
//delete[] normals;
//if(rgb)
//  delete[] rgb;
  delete _viewer;
  delete _sensor;
  delete _space;
}

