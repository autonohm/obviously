/**
* @file   ParentDevice3D.cpp
* @author Christian Pfitzner
* @date   07.01.2013
*
*
*/

#include "obdevice/ParentDevice3D.h"

using namespace obvious;

ParentDevice3D::~ParentDevice3D()
{
  delete _coords;
  delete _mask;
}

MatD ParentDevice3D::getMatZ(void) const
{
  const double* z = _z;
  MatD mat(_rows, _cols);

  for (unsigned int row = 0; row < _rows; row++)
      for (unsigned int col = 0; col < _cols; col++)
          mat.at(row, col) = *z++;

  return mat;
}

void ParentDevice3D::startRecording(char* filename)
{
  _recfile.open(filename, std::ios::out);
  _recfile << _cols << " " << _rows << std::endl;
  _record = true;
}

void ParentDevice3D::stopRecording()
{
  _recfile.close();
  _record = false;
}



