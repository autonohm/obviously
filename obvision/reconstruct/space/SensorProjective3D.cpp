#include "SensorProjective3D.h"

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

SensorProjective3D::SensorProjective3D(unsigned int cols, unsigned int rows, double PData[12], double maxRange, double minRange) : Sensor(3, maxRange, minRange)
{
  init(cols, rows, PData);
}

SensorProjective3D::SensorProjective3D(SensorProjective3D* sensor) : Sensor(3, sensor->getMaximumRange(), sensor->getMinimumRange())
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

  _rays= new Matrix(3, _size);
  unsigned int i=0;
  for(unsigned int row=0; row<_height; row++)
    for(unsigned int col=0; col<_width; col++, i++)
    {
      Matrix ray(3, 1);
      project2Space(col, row, 1.0, &ray);
      // Normalize ray to 1.0
      double len = sqrt(ray(0,0)*ray(0,0) + ray(1,0)*ray(1,0) + ray(2,0)*ray(2,0));
      (*_rays)(0, i) = ray(0, 0) / len;
      (*_rays)(1, i) = ray(1, 0) / len;
      (*_rays)(2, i) = ray(2, 0) / len;
    }
}

SensorProjective3D::~SensorProjective3D()
{
  delete _P;
  delete[] _data; _data = NULL;
  delete[] _mask; _mask = NULL;
  delete _rays;
}

void SensorProjective3D::project2Space(const unsigned int col, const unsigned int row, const double depth, Matrix* coord)
{
  double fx = (*_P)(0,0);
  double fy = (*_P)(1,1);
  double tx = (*_P)(0,2);
  double ty = (*_P)(1,2);

  double x = (depth/fx)*(col-tx);
  double y = (depth/fy)*(row-ty);
  //double lambda_inv = 1./sqrt(x * x + y * y + 1.);
  //double z = depth * lambda_inv;

  (*coord)(0, 0) = x;
  (*coord)(1, 0) = y;
  (*coord)(2, 0) = depth;
}

void SensorProjective3D::backProject(Matrix* M, int* indices, Matrix* T)
{
  //Matrix PoseInv = (*_Pose);
  Matrix PoseInv = getTransformation();
  PoseInv.invert();

  // Provide temporary transformation of voxelCoords, i.e. shift of coordinate system (partitioning)
  if(T)
    PoseInv *= *T;

  Matrix Pgen(3, 4);
  Pgen = (*_P) * PoseInv;

  // coords2D = P * Tinv * Ttmp * voxelCoords
  Matrix coords2D = Matrix::multiply(Pgen, *M, false, true);

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    indices[i] = -1;
    if(coords2D(2,i) > 0.0)
    {
      const double inv_dw = 1.0 / coords2D(2,i);
      const unsigned int u = static_cast<unsigned int>(coords2D(0,i)*inv_dw + 0.5);
      const unsigned int v = static_cast<unsigned int>(coords2D(1,i)*inv_dw + 0.5);

      if(u < _width && v < _height && _mask[((_height - 1) - v) * _width + u])
      {
        unsigned int idx = ((_height - 1) - v) * _width + u;
        indices[i] = idx;
      }
    }
  }
}

}
