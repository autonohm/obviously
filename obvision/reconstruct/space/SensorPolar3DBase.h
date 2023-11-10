#ifndef OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_
#define OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_

#include "obcore/math/linalg/eigen/Matrix.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{
/**
 * @class SensorPolar3DBase
 * @brief generic sensor model class for polar sensing units (to be tested for: Velodyne VLP16 PUCK, Velodyne HDL-32E, ohm_tilt_scanner_3d, SICK LDMRS8, ohm_4d_scanner, Ouster OS0-128, & many more)
 * @brief all polar sensor model classes inherit from this class & implement lookupIndex
 * @author Jasmin Ziegler
 */
class SensorPolar3DBase : public Sensor
{
public:

/**
 * Standard Constructor
 * @param[in] inclMin lowest inclination angle [RAD] (vertical)
 * @param[in] inclMax highest inclination angle in [RAD] (vertical)
 * @param[in] inclRes vertical angular resolution [RAD]
 * @param[in] azimMin lowest azimuth angle [RAD] (horizontal)
 * @param[in] azimMax highest azimuth angle in [RAD] (horizontal)
 * @param[in] azimRes horizontal angular resolution [RAD]
 */
SensorPolar3DBase(double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes, double maxRange = INFINITY, double minRange = 0.0, double lowReflectivityRange = INFINITY);
 
/**
 * Destructor
 */
virtual ~SensorPolar3DBase();



} //namespacec obvious

#endif /* OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_ */