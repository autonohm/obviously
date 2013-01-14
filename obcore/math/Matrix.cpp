#include "Matrix.h"

#include <gsl/gsl_permutation.h>
#include <gsl/gsl_statistics_double.h>
#include <math.h>

namespace obvious
{

Matrix::Matrix(unsigned int rows, unsigned int cols)
{
  _M = gsl_matrix_alloc(rows, cols);
  _work = gsl_matrix_alloc(rows, cols);
}

Matrix::Matrix(const Matrix &M)
{
  int r = M._M->size1;
  int c = M._M->size2;
  _M = gsl_matrix_alloc(r, c);
  _work = gsl_matrix_alloc(r, c);
  gsl_matrix_memcpy(_M, M._M);
}

Matrix::~Matrix()
{
  gsl_matrix_free(_M);
  gsl_matrix_free(_work);
}

Matrix&  Matrix::operator =  (const Matrix &M)
{
  gsl_matrix_memcpy(_M, M._M);
  return *this;
}

Matrix&  Matrix::operator *=  (const Matrix &M)
{
  gsl_matrix_memcpy(_work, _M);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _work, M._M, 0.0, _M);
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

void Matrix::setData(double* array)
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
  gsl_matrix_memcpy(_work, _M);
  gsl_linalg_LU_decomp (_work, perm ,&sig);
  gsl_linalg_LU_invert (_work, perm, _M);
  gsl_permutation_free(perm);
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

gsl_vector* Matrix::centroid()
{
  gsl_vector* c = gsl_vector_alloc(_M->size2);

  for(size_t i=0; i<_M->size2; i++)
  {
    gsl_vector_view col = gsl_matrix_column(_M, i);
    double m = gsl_stats_mean(col.vector.data,_M->size2,_M->size1);
    gsl_vector_set(c, i, m);
  }

  return c;
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
  gsl_vector* vcent = this->centroid();
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
