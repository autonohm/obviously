#include "SensorPolar3DBase.h"

namespace obvious
{

SensorPolar3DBase::SensorPolar3DBase(double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes, double maxRange = INFINITY, double minRange = 0.0, double lowReflectivityRange = INFINITY)
: Sensor(3, maxRange, minRange, lowReflectivityRange)
{
    unsigned int raysIncl = round(static_cast<unsigned>((abs(inclMin) + abs(inclMax)) / inclRes));

    
    //inherited from class sensor
    
}






















} // namespace obvious