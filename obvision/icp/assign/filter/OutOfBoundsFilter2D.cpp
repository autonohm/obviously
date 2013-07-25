#include "OutOfBoundsFilter2D.h"
#include "obcore/base/System.h"
#include <string.h>

namespace obvious
{

OutOfBoundsFilter2D::OutOfBoundsFilter2D(double xMin, double xMax, double yMin, double yMax)
{
  _xMin = xMin;
  _xMax = xMax;
  _yMin = yMin;
  _yMax = yMax;
  _T = new Matrix(3, 3);;
}

OutOfBoundsFilter2D::~OutOfBoundsFilter2D()
{
  delete _T;
}

void OutOfBoundsFilter2D::setPose(Matrix* T)
{
  _T->copy(*T);
}

void OutOfBoundsFilter2D::filter(double** scene, unsigned int size, bool* mask)
{
  double** sceneAligned;
  System<double>::allocate(size, 2, sceneAligned);
  memcpy(*sceneAligned, *scene, size*2);

  // Apply rotation
  gsl_matrix_view points = gsl_matrix_view_array(*sceneAligned, size, 2);
  gsl_matrix* points_tmp = gsl_matrix_alloc(size, 2);
  gsl_matrix_view R = gsl_matrix_submatrix(_T->getBuffer(), 0, 0, 2, 2);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &points.matrix, &R.matrix, 0.0, points_tmp);
  gsl_matrix_memcpy(&points.matrix, points_tmp);
  gsl_matrix_free(points_tmp);

  // Add translation
  gsl_vector_view x  = gsl_matrix_column(&points.matrix, 0);
  gsl_vector_view y  = gsl_matrix_column(&points.matrix, 1);
  gsl_vector_view tr = gsl_matrix_column(_T->getBuffer(), 2);
  gsl_vector_add_constant(&x.vector, gsl_vector_get(&tr.vector,0));
  gsl_vector_add_constant(&y.vector, gsl_vector_get(&tr.vector,1));

  for(int i=0; i<size; i++)
  {
    double* pt = gsl_matrix_ptr(&points.matrix, i, 0);
    if(pt[0]<_xMin || pt[0]>_xMax || pt[1]<_yMin || pt[1]>_yMax)
      mask[i] = false;
  }
  System<double>::deallocate(sceneAligned);
}

}

