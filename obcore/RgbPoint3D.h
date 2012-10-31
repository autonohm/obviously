/*********************************************************************************/
/*                                                                               */
/* class RgbPoint3D                                                              */
/*                                                                               */
/*      Author: Ch. Merkl                                                        */
/*  Created on: 25.10.2012                                                       */
/* Description:	rgb point, child of Point3D                                      */
/*                                                                               */
/*********************************************************************************/

#ifndef __RGB_POINT_3D__
#define __RGB_POINT_3D__

#include "Point3D.h"
#include "rgbColor.h"

namespace obvious {

class RgbPoint3D : public virtual Point3D
{
public:
    //! Constructor
    RgbPoint3D(const double x = 0.0, const double y = 0.0, const double z = 0.0, const RGBColor& rgb = RGBColor())
        : Point3D(x, y, z), m_rgb(rgb) { }

    //! CopyConstructor
    RgbPoint3D(const RgbPoint3D& point) : Point3D(point.m_x, point.m_y, point.m_z), m_rgb(point.m_rgb) { }

    //! Operators
    RgbPoint3D& operator=(const RgbPoint3D& point)
    {
        m_rgb = point.m_rgb;
        Point3D::operator=(point);
        return *this;
    }

    void setRGB(const RGBColor& rgb) { m_rgb = rgb; }
    const RGBColor& rgb(void) const { return m_rgb; }

protected:
    RGBColor m_rgb;
};

}

#endif
