#include "Device3D.h"
#include "Point3D.h"

namespace obvious {

Device3D::~Device3D(void)
{
    this->deletePoints();
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

MatD Device3D::getMatZ(void) const
{
    const double* z = _z;
    MatD mat(_rows, _cols);

    for (unsigned int row = 0; row < _rows; row++)
        for (unsigned int col = 0; col < _cols; col++)
            mat.at(row, col) = *z++;

    return mat;
}

void Device3D::startRecording(char* filename)
{
  _recfile.open(filename, ios::out);
  _recfile << _cols << " " << _rows << endl;
  _record = true;
}

void Device3D::stopRecording()
{
  _recfile.close();
  _record = false;
}

} // end namespace obvious
