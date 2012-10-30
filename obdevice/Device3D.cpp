#include "Device3D.h"
#include "Point3D.h"

namespace obvious {

Device3D::~Device3D(void)
{
    for (unsigned int i = 0; i < m_points.size(); i++)
        delete m_points[i];
}

void Device3D::deletePoints(void)
{
    std::vector<Point3D*>::iterator it = m_points.begin();

    while (!m_points.empty())
    {
        delete m_points.back();
        m_points.pop_back();
    }
}

} // end namespace obvious
