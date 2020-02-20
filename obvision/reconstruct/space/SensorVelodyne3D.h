#ifndef OBVISION_RECONSTRUCT_SPACE_SENSORVELODYNE3D_H_
#define OBVISION_RECONSTRUCT_SPACE_SENSORVELODYNE3D_H_

#include "obcore/math/linalg/eigen/Matrix.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class SensorVelodyne3D
 * @brief class for velodyne 3D laser scanners (VLP16 PUCK, E32)
 * @author Jasmin Ziegler
 */
class SensorVelodyne3D : public Sensor
{
public:
  /**
   * Standard constructor
   * @param[in] raysIncl number of inclination rays of scanning device (vertical)
   * @param[in] inclMin lowest inclination angle in RAD
   * @param[in] inclRes resolution of inclination rays in RAD, i.e. angle between two vertical rays
   * @param[in] azimRes resolution of azimuth rays in RAD, angle between two horizontal rays in 360° plane
   */
  SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange = INFINITY, double minRange = 0.0,
                   double lowReflectivityRange = INFINITY);

  /**
   * Destructor
   */
  virtual ~SensorVelodyne3D();

  /**
   * returns azimuth angle and inclination angle
   * @param[in] xCoord x coordinate of a 3D point
   * @param[in] yCoord y coordinate of a 3D point
   * @param[in] zCoord z coordinate of a 3D point
   * @param[out] inclAngle inclination angle in x-z-plane, 16 layers of rays from -15° to +15°, 2° resolution (VLP16 PUCK) in RAD
   * @param[out] azimAngle azimuth angle in x-y-plane, 0° to 360° in RAD
   */
  void returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle);

  /**
   * returns ray index
   * @param[in] azimAngle azimuth angle calculated in returnAngles()
   * @param[in] inclAngle inclination angle calculated in returnAngles()
   * @param[out] azimIndex index of azimuth ray number which is closest to 3D point to assign laser data to 3D point
   * @param[out] inclIndex index of inclination ray number which is closest to 3D point to assign laser data to 3D point
   * @todo remove adaptation to VLP16 PUCK - formulate generally
   */
  void returnRayIndex(double azimAngle, double inclAngle, unsigned int* azimIndex, unsigned int* inclIndex);

  /**
   * sets up an index map in the form of an 2D array; iterates over all azimuth values from 0° - 360° (=ROWS) and all inclination values from -15° to +15°
   * (=COLUMNS)
   * @param[in] width width of sensor, same as _width inherited from Sensor; in this case all azimuth rays
   * @param[in] height height of sensor, same as _height inherited from Sensor; in this case all inclination rays
   */
  void setIndexMap(unsigned int width, unsigned int height);

  /**
   *
   * @todo make this function abstract in parent class so each velodyne sensor inheriting from it has to implement it since it may differ from sensor to
   * sensor
   */
  unsigned int lookupIndex(int indexSensormodel);

  /**
   * Project all coordinates (center of each voxel in tsd space) back to sensor index: which sensor ray comes closest to the coordinate?
   * @param[in] M matrix of 3D coordinates of all voxel center points in tsd space (homogeneous)
   * @param[out] indices vector of projection results (must be allocated outside)
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T = NULL);

private:
  double _azimRes;
  double _inclRes;
  int**  _indexMap;
  /////////////////////////////////////////// DAS HIER  RAUS wird im push initialisiert
  // int* _indices;
  //////////////////////////////
};

} /* namespace obvious */

#endif /* OBVISION_RECONSTRUCT_SPACE_SENSORVELODYNE3D_H_ */
