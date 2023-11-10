#include "SensorVelodyne3D.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <limits>

namespace obvious
{

SensorVelodyne3D::SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange, double minRange,
                                   double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _azimRes                  = azimRes;
  _inclRes                  = inclRes;
  unsigned int raysAzim     = 0;
  double       azimAngle    = 0.0;
  const double resetInclMin = inclMin; // to reset variable inclMin each time
                                       // after exiting inner for-loop (=-15°
                                       // here for VLP16)

  raysAzim = round(static_cast<unsigned>(2 * M_PI / azimRes));

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// PLEASE LET SOMEONE CHECK IF THE AZIM+1 THING IS CORRECT! DO I REALLY HAVE 361 values FOR AZIMUT? yes right?
  /// +1 bei allocate _indexMap und bei _width
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // inherited from class sensor
  // Size of first dimension, i.e., # of samples of a 2D scan or width of image
  _width = raysAzim + 1;
  // Size of second dimension, i.e., 1 for a 2D scan or height of image sensor
  _height = raysIncl;
  // Number of measurement samples, i.e. _width x _height
  _size = _width * _height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i]         = true;

  // evtl mit _height u _width ersetzen
  obvious::System<int>::allocate(_width, _height, _indexMap);

  // returnRayIndex(azimAngle, inclAngle, &azimIndex, &inclIndex);

  setIndexMap(_width, _height);

  _rays = new obvious::Matrix(3, _size);

  obvious::Matrix R = obvious::Matrix(*_T, 0, 0, 3, 3);

  double inclAngle = 0.0;
  double xCoord    = 2.0;
  double yCoord    = 2.0;
  double zCoord    = 2.0;
  returnAngles(xCoord, yCoord, zCoord, &inclAngle, &azimAngle);

  unsigned int n = 0; // counts all rays of inner loop and outer loop to store
                      // in matrix _rays
  for(unsigned int i = 0; i < _width; i++)
  {
    for(unsigned int j = 0; j < _height; j++, n++)
    {
      obvious::Matrix calcRay(3, 1);
      // to follow definition of theta (inclination angle) in spherical
      // coordinate system
      double       thetaSphere = 0.0;
      const double piHalf      = deg2rad(90.0);

      if(inclMin < 0)
      {
        thetaSphere = piHalf + (inclMin) * (-1);
      }
      else
      {
        thetaSphere = piHalf - inclMin;
      }

      calcRay(0, 0) = sin(thetaSphere) * cos(azimAngle);
      calcRay(1, 0) = sin(thetaSphere) * sin(azimAngle);
      calcRay(2, 0) = cos(thetaSphere);
      // normalize rays
      const double length    = sqrt(calcRay(0, 0) * calcRay(0, 0) + calcRay(1, 0) * calcRay(1, 0) + calcRay(2, 0) * calcRay(2, 0));
      const double lengthInv = 1.0 / length;
      calcRay                = R * calcRay;

      // store end points of current ray in matrix* _rays inherited from class Sensor
      (*_rays)(0, n) = calcRay(0, 0) * lengthInv;
      (*_rays)(1, n) = calcRay(1, 0) * lengthInv;
      (*_rays)(2, n) = calcRay(2, 0) * lengthInv;

      inclMin += inclRes;
    }
    inclMin = resetInclMin; // reset inclination angle to minimum
    azimAngle += azimRes;
  }

  _raysLocal  = new obvious::Matrix(3, _size);
  *_raysLocal = *_rays;
}

SensorVelodyne3D::~SensorVelodyne3D()
{
  delete _rays;
  delete _raysLocal;
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
}

// return by reference - inclAngle u azimuth in DEGREEs -- changed to RAD
void SensorVelodyne3D::returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle)
{
  // Inclination
  double theta = 0.0; // careful love, this is the angle between z-axis and x-y-plane as
                      // defined in polar coordinates --> no distinction of
                      // cases for acos necessary, bec. only values from 75° -
                      // 105° for VLP16
  double length    = sqrt(xCoord * xCoord + yCoord * yCoord + zCoord * zCoord);
  double lengthInv = 1.0 / length;

  theta = acos(zCoord * lengthInv);

  if(theta > deg2rad(90.0)) // translate theta into inclination angle "aperture angle"
                            // from -15° to +15°
  {
    *inclAngle = -(theta - deg2rad(90.0)); // -15° -> 0°
  }
  else
  {
    *inclAngle = deg2rad(90.0) - theta; // 0° -> +15°
  }

  // Azimuth
  *azimAngle = atan2(yCoord, xCoord);
  if(*azimAngle < 0)
  {
    *azimAngle += 2.0 * M_PI; // express angles positively in 3rd and 4th quadrant
  }
}

void SensorVelodyne3D::returnRayIndex(double azimAngle, double inclAngle, unsigned int* azimIndex, unsigned int* inclIndex)
{
  *azimIndex = round(azimAngle / _azimRes);

  // assignment will always be the same for VLP16 - inclination resolution is
  // fixed 2° and nbr of rays is 16
  double mapInclination = inclAngle + deg2rad(15.0); // map inclination angles (-15° -> +15°) up to positive
  // range 0° - 30° --> TO DO change so this also works for E32
  *inclIndex = round(mapInclination / _inclRes);
}

void SensorVelodyne3D::setIndexMap(unsigned int width, unsigned int height)
{
  unsigned int column = 0;
  for(unsigned int row = 0; row < width; row++)
  {
    for(column = 0; column < height; column++)
    {
      _indexMap[row][column] = row * (height) + column;
    }
    column = 0; // iterate over 16 vertical rays for each azimuth ray
  }
}

// todo - adapt this for E32
unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
{
  unsigned int indexVelodyneROS = 0;
  switch(indexSensormodel)
  {
  case 0:
    indexVelodyneROS = 0;
    break;
  case 1:
    indexVelodyneROS = 2;
    break;
  case 2:
    indexVelodyneROS = 4;
    break;
  case 3:
    indexVelodyneROS = 6;
    break;
  case 4:
    indexVelodyneROS = 8;
    break;
  case 5:
    indexVelodyneROS = 10;
    break;
  case 6:
    indexVelodyneROS = 12;
    break;
  case 7:
    indexVelodyneROS = 14;
    break;
  case 8:
    indexVelodyneROS = 1;
    break;
  case 9:
    indexVelodyneROS = 3;
    break;
  case 10:
    indexVelodyneROS = 5;
    break;
  case 11:
    indexVelodyneROS = 7;
    break;
  case 12:
    indexVelodyneROS = 9;
    break;
  case 13:
    indexVelodyneROS = 11;
    break;
  case 14:
    indexVelodyneROS = 13;
    break;
  case 15:
    indexVelodyneROS = 15;
    break;
  }
  return indexVelodyneROS;
}

// M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorVelodyne3D::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{
  obvious::Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  // multiply PoseInv with M where poseInv is not transposed but M is transposed (true)
  obvious::Matrix coords3D = obvious::Matrix::multiply(PoseInv, *M, false, true);

  double       inclAngle    = 0.0;
  double       azimAngle    = 0.0;
  unsigned int row          = 0;
  unsigned int column       = 0;
  unsigned int columnMapped = 0;
  unsigned int idxCheck     = 0;

  for(unsigned int i = 0; i < M->getRows(); i++)
  {
    double x = coords3D(0, i);
    double y = coords3D(1, i);
    double z = coords3D(2, i);

    returnAngles(x, y, z, &inclAngle, &azimAngle);

    // leave current loop if incl angle out of measurement area -15° --> +15.0°
    if((inclAngle < deg2rad(-15.0)) || (inclAngle > deg2rad(15.0)))
    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // 1: calculate incoming azimuth = ROW index of indexMap
      // 2: calculate incoming inclination = COLUMN of indexMap
      returnRayIndex(azimAngle, inclAngle, &row, &column);

      // ROW CORRECTED weil row= azimindex max zb 359,9 / 0.2 = 1799 == 1800 --> index 1799 weil 0 anfängt? ist das richtig?
      // 0 / 0.2 = 0
      // 0.2 / 0.2 = 1
      // muss nur beim letzten wert passieren gell? was versteh ich hier grad nicht
      // warum passiert das dann bei col nicht?
      // ich brauch einen index mehr gell? in allocate

      // map column from sensor model to Velodyne firing sequence (order of
      // vertical rays differs between sensormodel and velodyne ros input)
      columnMapped = lookupIndex(column);

      // probe: index ausrechnen
      idxCheck = columnMapped + 16 * row;

      // push current value of current indexMap[row][column] into int* indices (returned by backProject to push())
      indices[i] = _indexMap[row][columnMapped];
    }
  }
}
} // namespace obvious