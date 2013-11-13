#include "MatrixView.h"
#include "obcore/base/Logger.h"
#include <math.h>

namespace obvious
{

MatrixView::MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{
  _V = gsl_matrix_submatrix(M->getBuffer(), i, j, i+rows, j+cols);
  _M = &(_V.matrix);
  _work = gsl_matrix_alloc(rows, cols);
}

Matrix::~Matrix()
{

}

}
