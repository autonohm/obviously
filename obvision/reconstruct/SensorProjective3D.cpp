#include "SensorProjective3D.h"

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

#define _PARALLEL_VERSION 1

SensorProjective3D::SensorProjective3D(unsigned int cols, unsigned int rows, double PData[12], double voxelSize) : Sensor(3)
{
  _P = new Matrix(3,4);
  _P->setData(PData);

  _cols = cols;
  _rows = rows;
  _size = _cols*_rows;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  System<Matrix*>::allocate(_cols, _rows, _rays);
  for(unsigned int col=0; col<_cols; col++)
    for(unsigned int row=0; row<_rows; row++)
    {
      _rays[col][row] = new Matrix(4, 1);
      project2Space(col, row, 1.0, _rays[col][row]);

      // Normalize ray to size of voxel
      Matrix* M = _rays[col][row];
      double len = sqrt((*M)[0][0]*(*M)[0][0] + (*M)[1][0]*(*M)[1][0] + (*M)[2][0]*(*M)[2][0]);
      len /= voxelSize;
      (*M)[0][0] /= len;
      (*M)[1][0] /= len;
      (*M)[2][0] /= len;
      (*M)[3][0] = 0.0;
    }
}

SensorProjective3D::~SensorProjective3D()
{
  delete _P;
  delete[] _data;
  delete[] _mask;

  for(unsigned int col=0; col<_cols; col++)
    for(unsigned int row=0; row<_rows; row++)
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
  double Tx = (*_P)[0][2];
  double Ty = (*_P)[1][2];

  double x = (depth/fx)*(col-Tx);
  double y = (depth/fy)*(row-Ty);
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

  double tr[3];
  getPosition(tr);

  gsl_matrix* Pgen = gsl_matrix_alloc(3, 4);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _P->getBuffer(), PoseInv.getBuffer(), 0.0, Pgen);

  // coords2D = P * Tinv * voxelCoords
  gsl_matrix* coords2D = gsl_matrix_alloc(3, M->getRows());
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Pgen, M->getBuffer(), 0.0, coords2D);

  const double* du = gsl_matrix_ptr(coords2D, 0, 0);
  const double* dv = gsl_matrix_ptr(coords2D, 1, 0);
  const double* dw = gsl_matrix_ptr(coords2D, 2, 0);

  for(unsigned int i=0; i<M->getRows(); i++)
  {
    indices[i] = -1;
    if(dw[i] > 0.0)
    {
      const double inv_dw = 1.0 / dw[i];
      const unsigned int u = static_cast<unsigned int>(du[i]*inv_dw + 0.5);
      const unsigned int v = static_cast<unsigned int>(dv[i]*inv_dw + 0.5);

      if(u < _cols && v < _rows && _mask[((_rows - 1) - v) * _cols + u])
      {
        unsigned int idx = ((_rows - 1) - v) * _cols + u;
        indices[i] = idx;
      }
    }
  }

  gsl_matrix_free(coords2D);
}

}
