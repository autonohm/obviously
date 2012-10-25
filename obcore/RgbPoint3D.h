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
    RgbPoint3D(const double x = 0.0, const double y = 0.0, const double z = 0.0, const rgbColor& rgb = RGB(Color::Black))
        : Point3D(x, y, z), m_rgb(rgb) { }

    //! CopyConstructor
    RgbPoint3D(const RgbPoint3D& point) : RgbPoint3D(point.m_x, point.m_y, point.m_z), m_rgb(point.m_rgb) { }

    //! Operators
    RgbPoint3D& operator=(const RgbPoint3D& point)
    {
        m_rgb = point.m_rgb;
        Point3D::opertor=(point);
        return *this;
    }

    void setRGB(const rgbColor& rgb) { m_rgb = rgb; }
    const rgbColor& rgb(void) const { return m_rgb; }

protected:
    rgbColor m_rgb;
};

}

#endif
