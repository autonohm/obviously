/**
* @file   ParentDevice3D.cpp
* @author Christian Pfitzner
* @date   07.01.2013
*
*
*/

#include "obdevice/ParentDevice3D.h"

using namespace obvious;

ParentDevice3D::ParentDevice3D(unsigned int cols, unsigned int rows)
  : _coords(NULL), _mask(NULL), _z(NULL),
   _rows(rows), _cols(cols),
   _frameRate(0.0f),
   _record(false)
{
  _cols      = cols;
  _rows      = rows;
  _coords    = new double[_rows*_cols*3];
  _mask      = new bool[_rows*_cols];
  _z         = new double[_rows*_cols];
  _rgb       = new unsigned char[_rows*_cols*3];
}


ParentDevice3D::~ParentDevice3D()
{
  delete [] _coords;
  delete [] _z;
  delete [] _mask;
  delete [] _rgb;
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

MatRGB ParentDevice3D::getMatRGB(void) const
{
    const unsigned char* rgb = _rgb;
    MatRGB mat(_rows, _cols);

    for (unsigned int row = 0; row < _rows; row++)
    {
        for (unsigned int col = 0; col < _cols; col++)
        {
            mat.at(row, col, MatRGB::Red)   = *rgb++;
            mat.at(row, col, MatRGB::Green) = *rgb++;
            mat.at(row, col, MatRGB::Blue)  = *rgb++;
        }
    }

    return mat;
}

/*
 * Function to estimate frame rate
 */
void ParentDevice3D::estimateFrameRate(void)
{
  static long double oldTime;
  long double newTime = _time.getTime();

  _frameRate = 1 / (newTime - oldTime) * 1000;
  oldTime = newTime;
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



