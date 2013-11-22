#include "SensorProjective3D.h"

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

SensorProjective3D::SensorProjective3D(unsigned int cols, unsigned int rows, double PData[12]) : Sensor(3)
{
  init(cols, rows, PData);
}

SensorProjective3D::SensorProjective3D(SensorProjective3D* sensor) : Sensor(3)
{
  double PData[12];
  sensor->_P->getData(PData);

  init(sensor->_width, sensor->_height, PData);
}

void SensorProjective3D::init(unsigned int cols, unsigned int rows, double PData[12])
{
  _P = new Matrix(3,4);
  _P->setData(PData);

  _width = cols;
  _height = rows;
  _size = _width*_height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  System<Matrix*>::allocate(_width, _height, _rays);
  for(unsigned int col=0; col<_width; col++)
    for(unsigned int row=0; row<_height; row++)
    {
      _rays[col][row] = new Matrix(4, 1);
      project2Space(col, row, 1.0, _rays[col][row]);

      // Normalize ray to 1.0
      Matrix* M = _rays[col][row];
      double len = sqrt((*M)[0][0]*(*M)[0][0] + (*M)[1][0]*(*M)[1][0] + (*M)[2][0]*(*M)[2][0]);
      (*M)[0][0] /= len;
      (*M)[1][0] /= len;
      (*M)[2][0] /= len;
      (*M)[3][0] = 0.0;
    }
}

SensorProjective3D::~SensorProjective3D()
{
  delete _P;
  delete[] _data; _data = NULL;
  delete[] _mask; _mask = NULL;

  for(unsigned int col=0; col<_width; col++)
    for(unsigned int row=0; row<_height; row++)
      delete _rays[col][row];
  System<Matrix*>::deallocate(_rays);
}

void SensorProjective3D::calcRayFromCurrentPose(const unsigned int row, const unsigned int col, double ray[3])
{
  Matrix r(4, 1);
  Matrix* T = getPose();

  // bring peakpoint in map coordinate system
  r = *_rays[col][row];
  r = *T * r;

  ray[0] = r[0][0];
  ray[1] = r[1][0];
  ray[2] = r[2][0];
}

void SensorProjective3D::project2Space(const unsigned int col, const unsigned int row, const double depth, Matrix* coord)
{
  double fx = (*_P)[0][0];
  double fy = (*_P)[1][1];
  double tx = (*_P)[0][2];
  double ty = (*_P)[1][2];

  double x = (depth/fx)*(col-tx);
  double y = (depth/fy)*(row-ty);
  //double lambda_inv = 1./sqrt(x * x + y * y + 1.);
  //double z = depth * lambda_inv;

  (*coord)[0][0]=x;
  (*coord)[1][0]=y;
  (*coord)[2][0]=depth;
  (*coord)[3][0]=1.0;
}

void SensorProjective3D::backProject(Matrix* M, int* indices)
{
  Matrix PoseInv = (*_Pose);
  PoseInv.invert();

  Matrix Pgen(3, 4);
  Pgen = (*_P) * PoseInv;

  // coords2D = P * Tinv * voxelCoords
  Matrix coords2D = Matrix::multiply(Pgen, *M, false, true);

  const double* du = coords2D[0];
  const double* dv = coords2D[1];
  const double* dw = coords2D[2];

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    indices[i] = -1;
    if(dw[i] > 0.0)
    {
      const double inv_dw = 1.0 / dw[i];
      const unsigned int u = static_cast<unsigned int>(du[i]*inv_dw + 0.5);
      const unsigned int v = static_cast<unsigned int>(dv[i]*inv_dw + 0.5);

      if(u < _width && v < _height && _mask[((_height - 1) - v) * _width + u])
      {
        unsigned int idx = ((_height - 1) - v) * _width + u;
        indices[i] = idx;
      }
    }
  }
}

}
