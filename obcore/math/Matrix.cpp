#include "Matrix.h"
#include "obcore/base/Logger.h"

#include <gsl/gsl_permutation.h>
#include <gsl/gsl_statistics_double.h>
#include <math.h>

namespace obvious
{

Matrix::Matrix(unsigned int rows, unsigned int cols, double* data)
{
  _M = gsl_matrix_alloc(rows, cols);
  if(data != NULL) setData(data);
}

Matrix::Matrix(const Matrix &M)
{
  int r = M._M->size1;
  int c = M._M->size2;
  _M = gsl_matrix_alloc(r, c);
  gsl_matrix_memcpy(_M, M._M);
}

Matrix::Matrix(Matrix M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{
  gsl_matrix_view V = gsl_matrix_submatrix(M._M, i, j, i+rows, j+cols);
  _M = gsl_matrix_alloc(rows, cols);
  gsl_matrix_memcpy(_M, &(V.matrix));
}

Matrix::~Matrix()
{
  gsl_matrix_free(_M);
}

void Matrix::copy(const Matrix &M)
{
  gsl_matrix_memcpy(_M, M._M);
}

Matrix&  Matrix::operator =  (const Matrix &M)
{
  gsl_matrix_memcpy(_M, M._M);
  return *this;
}

Matrix&  Matrix::operator *=  (const Matrix &M)
{
  gsl_matrix* work = gsl_matrix_alloc(_M->size1, _M->size2);
  gsl_matrix_memcpy(work, _M);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, work, M._M, 0.0, _M);
  gsl_matrix_free(work);
  return *this;
}

Matrix& Matrix::operator -= (const Matrix &M)
{
  gsl_matrix_sub (_M, M._M);
  return *this;
}

double* Matrix::operator [] (unsigned int i)
{
  return gsl_matrix_ptr(_M, i, 0);
}

Matrix operator * (const Matrix &M1, const Matrix &M2)
{
  Matrix Mnew(M1._M->size1, M2._M->size2);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, M1._M, M2._M, 0.0, Mnew._M);
  return Mnew;
}

Matrix Matrix::multiply(const Matrix &M1, const Matrix &M2, bool transposeArg1, bool transposeArg2)
{
  Matrix Mnew(M1._M->size1, M2._M->size2);
  enum CBLAS_TRANSPOSE t1 = CblasNoTrans;
  enum CBLAS_TRANSPOSE t2 = CblasNoTrans;
  if(transposeArg1) t1 = CblasTrans;
  if(transposeArg2) t2 = CblasTrans;
  gsl_blas_dgemm(t1, t2, 1.0, M1._M, M2._M, 0.0, Mnew._M);
  return Mnew;
}

void Matrix::multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2)
{
  gsl_matrix* work = gsl_matrix_alloc(_M->size1, _M->size2);
  enum CBLAS_TRANSPOSE t1 = CblasNoTrans;
  enum CBLAS_TRANSPOSE t2 = CblasNoTrans;
  if(transposeArg1) t1 = CblasTrans;
  if(transposeArg2) t2 = CblasTrans;
  gsl_blas_dgemm(t1, t2, 1.0, _M, M._M, 0.0, work);
  gsl_matrix_memcpy(_M, work);
  gsl_matrix_free(work);
}

ostream& operator <<(ostream &os, Matrix &M)
{
  unsigned int rows = M._M->size1;
  unsigned int cols = M._M->size2;
  for(unsigned int r=0; r<rows; r++)
  {
    os << M[r][0];
    for(unsigned int c=1; c<cols; c++)
    {
      os << " " << M[r][c];
    }
    os << endl;
  }
  return os;
}

gsl_matrix* Matrix::getBuffer()
{
  return _M;
}

void Matrix::getData(double* array)
{
  unsigned int rows = _M->size1;
  unsigned int cols = _M->size2;
  for(unsigned int r=0; r<rows; r++)
  {
    for(unsigned int c=0; c<cols; c++)
    {
      array[r*cols+c] = gsl_matrix_get(_M, r, c);
    }
  }
}

void Matrix::setData(const double* array)
{
  unsigned int rows = _M->size1;
  unsigned int cols = _M->size2;
  for(unsigned int r=0; r<rows; r++)
  {
    for(unsigned int c=0; c<cols; c++)
    {
      gsl_matrix_set(_M, r, c, array[r*cols+c]);
    }
  }
}

unsigned int Matrix::getRows()
{
  return _M->size1;
}

unsigned int Matrix::getCols()
{
  return _M->size2;
}

void Matrix::setIdentity()
{
  gsl_matrix_set_identity(_M);
}

void Matrix::setZero()
{
  gsl_matrix_set_zero(_M);
}

Matrix Matrix::getInverse()
{
  Matrix Mi = *this;
  Mi.invert();
  return Mi;
}

void Matrix::invert()
{
  int r = _M->size1;
  int sig;
  gsl_permutation* perm = gsl_permutation_alloc(r);
  gsl_matrix* work = gsl_matrix_alloc(_M->size1, _M->size2);
  gsl_matrix_memcpy(work, _M);
  gsl_linalg_LU_decomp (work, perm ,&sig);
  gsl_linalg_LU_invert (work, perm, _M);
  gsl_matrix_free(work);
  gsl_permutation_free(perm);
}

void Matrix::transpose()
{
  if(_M->size1 != _M->size2)
  {
    LOGMSG(DBG_ERROR, "Matrix must be square to be transposed");
    return;
  }
  gsl_matrix_transpose(_M);
}

Matrix Matrix::getTranspose()
{
  if(_M->size1 != _M->size2)
  {
    Matrix Mt(_M->size2, _M->size1);
    gsl_matrix_transpose_memcpy(Mt.getBuffer(), _M);
    return Mt;
  }
  else
  {
    Matrix Mt = *this;
    Mt.transpose();
    return Mt;
  }
}

double Matrix::trace()
{
  int rows = _M->size1;
  int cols = _M->size2;

  if(rows != cols)
  {
    printf("WARNING Matrix::trace: trace is defined only for nxn-matrices\n");
    return 0.0;
  }

  double trace = 0.0;

  for(int i=0; i<rows; i++)
  {
    trace += gsl_matrix_get(this->_M, i, i);
  }
  return trace;
}

Matrix* Matrix::pcaAnalysis()
{

  gsl_matrix* M = gsl_matrix_alloc(_M->size1, _M->size2);
  gsl_matrix_memcpy(M, _M);

  size_t i;
  // number of points
  size_t rows = M->size1;
  // dimensionality
  size_t dim  = M->size2;

  Matrix* axes = new Matrix(dim, 2*dim);

  gsl_matrix* V = gsl_matrix_alloc(dim, dim);

  // calculate centroid
  gsl_vector* vcent = gsl_vector_alloc(_M->size2);
  for(size_t i=0; i<_M->size2; i++)
  {
    gsl_vector_view col = gsl_matrix_column(_M, i);
    double m = gsl_stats_mean(col.vector.data,_M->size2,_M->size1);
    gsl_vector_set(vcent, i, m);
  }

  double* cent = vcent->data;

  for(i=0; i<dim; i++)
  {
    gsl_vector_view c = gsl_matrix_column(M, i);
    gsl_vector_add_constant(&c.vector, -cent[i]);
  }

  gsl_matrix* MtM = gsl_matrix_alloc(dim,dim);
  gsl_blas_dgemm(CblasTrans, CblasNoTrans,
                 1.0, M, M,
                 0.0, MtM);

  gsl_vector* s = gsl_vector_alloc(dim);

  gsl_linalg_SV_decomp_jacobi(MtM, V, s);

  gsl_matrix* P = gsl_matrix_alloc(dim, rows);
  gsl_blas_dgemm (CblasTrans, CblasTrans,
                  1.0, V, M,
                  0.0, P);


  for(i=0; i<dim; i++)
  {
    // coordinates in coordinate system of eigenvectors
    gsl_vector_view coord = gsl_matrix_row(P, i);

    // corresponding eigenvector in original coordinate system
    double max = gsl_vector_max(&coord.vector);
    double min = gsl_vector_min(&coord.vector);
    double ext = max - min;
    double align = 0.0;;
    if(ext > 1e-6)
    {
      align = (max + min)/2.0;
    }

    gsl_vector_view eigen = gsl_matrix_column(V, i);

    for(size_t j=0; j<dim; j++)
    {
      double e = gsl_vector_get(&eigen.vector,j)*align;
      cent[j] += e;
    }
  }

  for(i=0; i<dim; i++)
  {
    gsl_vector_view coord = gsl_matrix_row(P, i);

    // extends of axis in orientation of eigenvector i
    double ext = gsl_vector_max(&coord.vector) - gsl_vector_min(&coord.vector);

    // corresponding eigenvector in original coordinate system
    gsl_vector_view eigen = gsl_matrix_column(V, i);

    // coordinates of axis j
    for(size_t j=0; j<dim; j++)
    {
      double e = gsl_vector_get(&eigen.vector,j)*ext/2.0;
      // axis coordinates with respect to center of original coordinate system
      gsl_matrix_set(axes->getBuffer(), i, 2*j,   cent[j] - e);
      gsl_matrix_set(axes->getBuffer(), i, 2*j+1, cent[j] + e);
    }
  }

  gsl_matrix_free(P);
  gsl_vector_free(s);
  gsl_matrix_free(MtM);
  gsl_vector_free(vcent);
  gsl_matrix_free(V);
  gsl_matrix_free(M);

  return axes;
}

void Matrix::svd(Matrix* U, double* s, Matrix* V)
{
  if(U->getCols() != getCols() || U->getRows() != getRows())
  {
    LOGMSG(DBG_ERROR, "Matrix U must have same dimension");
    return;
  }
  U->copy(*this);
  gsl_vector* work = gsl_vector_alloc(getRows());
  gsl_vector_view vs = gsl_vector_view_array(s, getRows());
  gsl_linalg_SV_decomp(U->getBuffer(), V->getBuffer(), &vs.vector, work);
  gsl_vector_free(work);
}

void Matrix::solve(double* b, double* x)
{
  unsigned int dim = getRows();
  gsl_vector_view vx = gsl_vector_view_array(x, dim);
  gsl_vector_view vb = gsl_vector_view_array(b, dim);

  gsl_permutation* perm = gsl_permutation_alloc(getRows());
  int sig;
  gsl_linalg_LU_decomp(_M, perm, &sig);

  gsl_linalg_LU_solve(_M, perm, &vb.vector, &vx.vector);
  gsl_permutation_free(perm);
}

Matrix* Matrix::TranslationMatrix44(double tx, double ty, double tz)
{
  Matrix* M = new Matrix(4, 4);
  M->setIdentity();
  gsl_matrix* buf = M->getBuffer();
  gsl_matrix_set(buf, 0, 3, tx);
  gsl_matrix_set(buf, 1, 3, ty);
  gsl_matrix_set(buf, 2, 3, tz);
  return M;
}

void Matrix::transform(Matrix T)
{
  unsigned int dim = getCols();

  if(dim!=2 && dim!=3)
  {
    LOGMSG(DBG_ERROR, "Matrix dimension invalid");
    return;
  }

  // Apply rotation
  gsl_matrix* points_tmp = gsl_matrix_alloc(getRows(), dim);
  gsl_matrix_view R = gsl_matrix_submatrix(T.getBuffer(), 0, 0, dim, dim);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, _M, &R.matrix, 0.0, points_tmp);
  gsl_matrix_memcpy(_M, points_tmp);
  gsl_matrix_free(points_tmp);

  // Add translation
  gsl_vector_view x  = gsl_matrix_column(_M, 0);
  gsl_vector_view y  = gsl_matrix_column(_M, 1);
  gsl_vector_view tr = gsl_matrix_column(T.getBuffer(), dim);
  gsl_vector_add_constant(&x.vector, gsl_vector_get(&tr.vector,0));
  gsl_vector_add_constant(&y.vector, gsl_vector_get(&tr.vector,1));

  if(dim==3)
  {
    gsl_vector_view z  = gsl_matrix_column(_M, 2);
    gsl_vector_add_constant(&z.vector, gsl_vector_get(&tr.vector,2));
  }
}

void Matrix::print()
{
  for(size_t r=0; r<_M->size1; r++)
  {
    for(size_t c=0; c<_M->size2; c++)
      cout << (*this)[r][c] << " ";
    cout << endl;
  }
}

}
