#include "KinectPlayback.h"
#include <cstring>
#include <stdlib.h>
#include <obcore/math/Matrix.h>
#include <math.h>

namespace obvious
{

KinectPlayback::KinectPlayback(const char* filename)
{
  _recfile.open(filename, ios::in);
  _recfile >> _cols >> _rows;

  _coords  = new double[_rows*_cols*3];
  _z       = new double[_rows*_cols];
  _rgb     = new unsigned char[_rows*_cols*3];
  _mask    = new unsigned char[_rows*_cols];
  memset(_mask, 0, _rows*_cols*sizeof(*_mask));

  // Determine number of frames
  unsigned int size = 0;
  string line;
  while (getline(_recfile, line))
    size++;
  _recfile.clear();
  _recfile.seekg(0, ios::beg);
  _recfile >> _cols >> _rows;

  _frames = (size-1) / (_rows*_cols);
  _frame  = 0;

  _eof    = false;
}

KinectPlayback::~KinectPlayback()
{
  delete [] _coords;
  delete [] _z;
  delete [] _rgb;
  delete [] _mask;
  _recfile.close();
}

bool KinectPlayback::grab()
{
  if(_eof) return false;

  double px, py, pz;
  double x, y, z;
  unsigned int r, g, b;

  memset(_mask, 0, _rows*_cols*sizeof(*_mask));

  for(unsigned int row=0; row<_rows; row++)
  {
    for(unsigned int col=0; col<_cols; col++)
    {
      _recfile >> px >> py >> pz >> x >> y >> z >> r >> g >> b;
      unsigned int i = row*_cols+col;
      _coords[3*i]   = x;
      _coords[3*i+1] = y;
      _coords[3*i+2] = z;
      _z[i]          = pz;
      _rgb[3*i]      = r;
      _rgb[3*i+1]    = g;
      _rgb[3*i+2]    = b;
      if(!isnan(_z[i]) && _coords[3*i+2]>10e-6) _mask[i] = 1;
    }
  }

  _frame++;

  if(_frame == _frames)
  {
    _eof   = true;
  }

  return true;
}

void KinectPlayback::reset()
{
  _recfile.clear();
  _recfile.seekg(0, ios::beg);
  _recfile >> _cols >> _rows;
  _frame = 0;
  _eof   = false;
}

bool KinectPlayback::eof()
{
  return _eof;
}

void KinectPlayback::skip(unsigned int frames)
{
  if(_frame+frames >= _frames)
  {
    _recfile.clear();
    _recfile.seekg(0, ios::beg);
    _recfile >> _cols >> _rows;
    _frame = 0;
  }
  else
  {
    string line;
    unsigned int size = _rows*_cols;
    for(unsigned int i=0; i<frames; i++)
    {
      for(unsigned int j=0; j<size; j++)
      {
        getline(_recfile, line);
      }
    }
  }
}

unsigned int KinectPlayback::getRows()
{
  return _rows;
}

unsigned int KinectPlayback::getCols()
{
  return _cols;
}

double* KinectPlayback::getCoords()
{
  return _coords;
}

unsigned char* KinectPlayback::getMask()
{
  return _mask;
}

double* KinectPlayback::getZ()
{
  return _z;
}

unsigned char* KinectPlayback::getRGB()
{
  return _rgb;
}

}
