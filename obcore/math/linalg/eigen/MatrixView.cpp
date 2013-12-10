#include "MatrixView.h"
#include "obcore/base/Logger.h"
#include <math.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_statistics_double.h>

namespace obvious
{

MatrixView::MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{

}

MatrixView::MatrixView(double* data, unsigned int rows, unsigned int cols)
{

}

MatrixView::~MatrixView()
{

}

double* MatrixView::operator [] (unsigned int i)
{

}

MatrixView&  MatrixView::operator *=  (const Matrix &M)
{

}

void MatrixView::multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2)
{

}

void MatrixView::print()
{

}

}
