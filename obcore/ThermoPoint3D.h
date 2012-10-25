/*********************************************************************************/
/*                                                                               */
/* class ThermoPoint3D                                                           */
/*                                                                               */
/*      Author: Ch. Merkel, R. Koch                                              */
/*  Created on: 10.09.2012                                                       */
/* Description:	contains x,y,z data and thermo data of one point                 */
/*                                                                               */
/*********************************************************************************/
#ifndef __THERMO_POINT_3D__
#define __THERMO_POINT_3D__

namespace obvious {

class ThermoPoint3D : public virtual Point3D
{
public:
    ThermoPoint3D(const double x = 0.0, const double y = 0.0, const double z = 0.0, const unsigned short temperature = 0)
        : Point3D(x, y, z), m_temperature(temperature) { }

    ThermoPoint3D(const ThermoPoint3D& point) : Point3D(point.m_x, point.m_y, point.m_z), m_temperature(point.m_temperature) { }

    ThermoPoint3D& operator=(const ThermoPoint3D& point)
    {
        m_temperature = point.m_temperature;
        Point3D::operator=(point);
        return *this;
    }

    unsigned short temperature(void) const { return m_temperature; }
    void setTemperature(const unsigned short temperature) { m_temperature = temperature; }

protected:
    unsigned short m_temperature;
};

}

#endif
