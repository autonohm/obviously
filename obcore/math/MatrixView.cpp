#include "MatrixView.h"
#include "obcore/base/Logger.h"
#include <math.h>

namespace obvious
{

MatrixView::MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{
  _V = gsl_matrix_submatrix(M->getBuffer(), i, j, i+rows, j+cols);
}

MatrixView::MatrixView(double* data, unsigned int rows, unsigned int cols)
{
  _V = gsl_matrix_view_array(data, rows, cols);
}

MatrixView::~MatrixView()
{

}

double* MatrixView::operator [] (unsigned int i)
{
  return gsl_matrix_ptr(&(_V.matrix), i, 0);
}

MatrixView&  MatrixView::operator *=  (const Matrix &M)
{
  gsl_matrix* work = gsl_matrix_alloc(_V.matrix.size1, _V.matrix.size2);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &(_V.matrix), M._M, 0.0, work);
  gsl_matrix_memcpy(&_V.matrix, work);
  gsl_matrix_free(work);
  return *this;
}

void MatrixView::multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2)
{
  gsl_matrix* work = gsl_matrix_alloc(_V.matrix.size1, _V.matrix.size2);
  enum CBLAS_TRANSPOSE t1 = CblasNoTrans;
  enum CBLAS_TRANSPOSE t2 = CblasNoTrans;
  if(transposeArg1) t1 = CblasTrans;
  if(transposeArg2) t2 = CblasTrans;
  gsl_blas_dgemm(t1, t2, 1.0, &(_V.matrix), M._M, 0.0, work);
  gsl_matrix_memcpy(&_V.matrix, work);
  gsl_matrix_free(work);
}

void MatrixView::print()
{
  for(size_t r=0; r<_V.matrix.size1; r++)
  {
    for(size_t c=0; c<_V.matrix.size2; c++)
      cout << (*this)[r][c] << " ";
    cout << endl;
  }
}
}
