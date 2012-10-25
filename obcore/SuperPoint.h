/*********************************************************************************/
/*                                                                               */
/* class SuperPoint3D                                                            */
/*                                                                               */
/*      Author: Ch. Merkl                                                        */
/*  Created on: 25.10.2012                                                       */
/*                                                                               */
/*********************************************************************************/

#ifndef __SUPER_POINT_3D__
#define __SUPER_POINT_3D__

#include "RgbPoint3D.h"
#include "ThermoPoint.h"

namespace obvious {

class SuperPoint3D : public RgbPoint3D, public ThermoPoint3D
{
public:
    //! Constructor
    SuperPoint3D(const double x = 0.0, const double y = 0.0, const double z = 0.0)
        : SuperPoint3D(x, y, z), RgbPoint3D(), ThermoPoint3D() { }

    //! Copy - Constructor
    SuperPoint3D(const SuperPoint3D& point) : Point3D(point.m_x, point.m_y, point.m_z), RgbPoint3D(), ThermoPoint(),
                                              m_temperature(point.m_temperature), m_rgb(point.m_rgb) { }

    //! Operators
    SuperPoint3D& operator=(const SuperPoint3D& point)
    {
        Point3D::operator=(point);
        m_temperature = point.m_temperature;
        m_rgb = point.m_rgb;

        return *this;
    }
};

}

#endif
