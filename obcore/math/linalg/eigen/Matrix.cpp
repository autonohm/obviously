#include "Matrix.h"
#include "obcore/base/Logger.h"

#include <math.h>

namespace obvious
{

using namespace Eigen;

Matrix::Matrix(unsigned int rows, unsigned int cols, double* data)
{
  _M.resize(rows, cols);
  if(data)
  {
    unsigned int i=0;
    for(unsigned int r=0; r<rows; r++)
      for(unsigned int c=0; c<cols; c++, i++)
        _M(r,c) = data[i];
  }
}

Matrix::Matrix(const Matrix &M)
{
  _M = M._M;
}

Matrix::Matrix(Matrix M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{
  _M.resize(rows, cols);
  _M = M._M.block(i, j, rows, cols);
}

VectorView Matrix::getColumnView(unsigned int index)
{

}

VectorView Matrix::getRowView(unsigned int index)
{

}

Matrix::~Matrix()
{

}

Matrix&  Matrix::operator =  (const Matrix &M)
{
  _M = M._M;
  return *this;
}

Matrix&  Matrix::operator *=  (const Matrix &M)
{
  _M *= M._M;
  return *this;
}

Matrix& Matrix::operator -= (const Matrix &M)
{
  _M -= M._M;
  return *this;
}

double& Matrix::operator () (unsigned int row, unsigned int col)
{
  return _M(row, col);
}

Matrix operator * (const Matrix &M1, const Matrix &M2)
{
  Matrix M(M1._M.rows(), M2._M.cols());
  M._M = M1._M * M2._M;
  return M;
}

void Matrix::multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2)
{
  MatrixXd Mleft = _M;
  MatrixXd Mright = M._M;
  if(transposeArg1 && transposeArg2)
    _M = Mleft.transpose() * Mright.transpose();
  else if(transposeArg1)
    _M = Mleft.transpose() * Mright;
  else if(transposeArg2)
    _M = Mleft * Mright.transpose();
  else
    _M = Mleft * Mright;
}

ostream& operator <<(ostream &os, Matrix &M)
{
  os << M;
  return os;
}

void Matrix::getData(double* array)
{
  unsigned int i=0;
  for(unsigned int r=0; r<_M.rows(); r++)
    for(unsigned int c=0; c<_M.cols(); c++, i++)
      array[i] = _M(r,c);
}

void Matrix::setData(const double* array)
{
  unsigned int i=0;
  for(unsigned r=0; r<_M.rows(); r++)
    for(unsigned c=0; c<_M.cols(); c++, i++)
      _M(r,c) = array[i];
}

unsigned int Matrix::getRows()
{
  return _M.rows();
}

unsigned int Matrix::getCols()
{
  return _M.cols();
}

void Matrix::setIdentity()
{
  _M.setIdentity();
}

void Matrix::setZero()
{
  _M.setZero(_M.rows(), _M.cols());
}

Matrix Matrix::getInverse()
{
  Matrix Mi = *this;
  Mi.invert();
  return Mi;
}

void Matrix::invert()
{
  _M = _M.inverse();
}

void Matrix::transpose()
{
  _M.transposeInPlace();
}

Matrix Matrix::getTranspose()
{
  Matrix M = *this;
  M.transpose();
  return M;
}

double Matrix::trace()
{
  return _M.trace();
}

Matrix* Matrix::pcaAnalysis()
{

}

void Matrix::svd(Matrix* U, double* s, Matrix* V)
{

}

void Matrix::solve(double* b, double* x)
{
  Map<MatrixXd> vb(b, _M.rows(), 1);
  Map<MatrixXd> vx(x, _M.rows(), 1);
  vx = _M.lu().solve(vb);
}

Matrix Matrix::createTransform(Matrix T)
{
  unsigned int dim = getCols();

  if(dim!=2 && dim!=3)
  {
    LOGMSG(DBG_ERROR, "Matrix dimension invalid");
    abort();
  }

  // Apply rotation
  Matrix M(getRows(), dim);
  Matrix Rt(dim, dim);
  for(unsigned int r=0; r<dim; r++)
    for(unsigned int c=0; c<dim; c++)
      Rt(c, r) = T(r, c);

  M = *this * Rt;

  for(unsigned int r=0; r<getRows(); r++)
  {
    M(r, 0) += T(0, dim);
    M(r, 1) += T(1, dim);
  }

  if(dim==3)
  for(unsigned int r=0; r<getRows(); r++)
    M(r, 2) += T(2, dim);

  return M;
}

void Matrix::transform(Matrix T)
{

}

void Matrix::print()
{
  cout << _M;
}

Matrix Matrix::multiply(const Matrix &M1, const Matrix &M2, bool transposeArg1, bool transposeArg2)
{
  MatrixXd Mleft = M1._M;
  MatrixXd Mright = M2._M;
  MatrixXd X;
  if(transposeArg1 && transposeArg2)
    X = Mleft.transpose() * Mright.transpose();
  else if(transposeArg1)
    X = Mleft.transpose() * Mright;
  else if(transposeArg2)
    X = Mleft * Mright.transpose();
  else
    X = Mleft * Mright;
  Matrix M(X.rows(), X.cols());
  M._M = X;
  return M;
}

Vector Matrix::multiply(const Matrix &M, const Vector &V, bool transpose)
{

}

}
