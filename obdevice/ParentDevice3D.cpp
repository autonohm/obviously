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

  _timer.start();
}


ParentDevice3D::~ParentDevice3D()
{
  delete [] _coords;
  delete [] _z;
  delete [] _mask;
  delete [] _rgb;
}

/*
 * Function to estimate frame rate
 */
void ParentDevice3D::estimateFrameRate(void)
{
    _frameRate = 1.0f / _timer.reset();
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



