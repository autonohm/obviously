#include "Matrix.h"
#include "obcore/base/Logger.h"

#include <math.h>

namespace obvious
{

using namespace Eigen;

Matrix::Matrix(unsigned int rows, unsigned int cols, double* data)
{
  if(data)
  {
    Map< Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> > M(data, rows, cols);
    _M = M;
  }
  else
  {
    _M.resize(rows, cols);
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

Matrix& Matrix::operator += (const Matrix &M)
{
  _M += M._M;
  return *this;
}

Matrix& Matrix::operator += (const double scalar)
{
  _M.array() += scalar;
  return *this;
}

void Matrix::addToColumn(unsigned int column, const double scalar)
{
  _M.col(column).array() += scalar;
}

void Matrix::addToRow(unsigned int row, const double scalar)
{
  _M.row(row).array() += scalar;
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
    _M.noalias() = Mleft.transpose() * Mright.transpose();
  else if(transposeArg1)
    _M.noalias() = Mleft.transpose() * Mright;
  else if(transposeArg2)
    _M.noalias() = Mleft * Mright.transpose();
  else
    _M.noalias() = Mleft * Mright;
}

ostream& operator <<(ostream &os, Matrix &M)
{
  os << M;
  return os;
}

void Matrix::getData(double* array)
{
  int i=0;
  for(int r=0; r<_M.rows(); r++)
    for(int c=0; c<_M.cols(); c++, i++)
      array[i] = _M(r,c);
}

void Matrix::setData(double* array)
{
  Map< Eigen::Matrix<double, Dynamic, Dynamic, RowMajor > > M(array, _M.rows(), _M.cols());
  _M = M;
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
  cout << "WARNING: Matrix::pcaAnalysis Not implemented yet" << endl;
  abort();
}

void Matrix::svd(Matrix* U, double* s, Matrix* V)
{
  if(U->getCols() != getCols() || U->getRows() != getRows())
  {
    LOGMSG(DBG_ERROR, "Matrix U must have same dimension");
    return;
  }

  JacobiSVD<MatrixXd> svdOfM = _M.jacobiSvd(ComputeFullU | ComputeFullV);
  U->_M = svdOfM.matrixU();
  V->_M = svdOfM.matrixV();
  VectorXd singular = svdOfM.singularValues();
  for(int i=0; i<singular.count(); i++)
    s[i] = singular(i);
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

  Matrix M = *this;
  M.transform(T);
  return M;
}

void Matrix::transform(Matrix T)
{
  unsigned int dim = getCols();

  if(dim!=2 && dim!=3)
  {
    LOGMSG(DBG_ERROR, "Matrix dimension invalid");
    abort();
  }

  // Apply rotation
  Matrix Rt(dim, dim);
  for(unsigned int r=0; r<dim; r++)
    for(unsigned int c=0; c<dim; c++)
      Rt(c, r) = T(r, c);

  _M *= Rt._M;
  addToColumn(0, T(0, dim));
  addToColumn(1, T(1, dim));
  if(dim==3)
    addToColumn(2, T(2, dim));
}

void Matrix::print()
{
  cout << _M;
  cout << endl;
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
  Vector V2(M._M.rows());
  if(transpose)
    V2._V = M._M.transpose() * V._V;
  else
    V2._V = M._M * V._V;
  return V2;
}

typedef Map<Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> > MapType;
void Matrix::multiply(const Matrix &M1, double* array, unsigned int rows, unsigned int cols)
{
  MapType map(array, rows, cols);
  map = map * M1._M.transpose();
}

}
