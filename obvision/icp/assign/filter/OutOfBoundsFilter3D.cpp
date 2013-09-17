#include "OutOfBoundsFilter3D.h"
#include "obcore/base/System.h"
#include <string.h>

namespace obvious
{

OutOfBoundsFilter3D::OutOfBoundsFilter3D(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
{
  _xMin = xMin;
  _xMax = xMax;
  _yMin = yMin;
  _yMax = yMax;
  _zMin = zMin;
  _zMax = zMax;
  _T = new Matrix(4, 4);
}

OutOfBoundsFilter3D::~OutOfBoundsFilter3D()
{
  delete _T;
}

void OutOfBoundsFilter3D::setPose(Matrix* T)
{
  _T->copy(*T);
}

void OutOfBoundsFilter3D::filter(double** scene, unsigned int size, bool* mask)
{
  double** sceneAligned;
  System<double>::allocate(size, 3, sceneAligned);
  memcpy(*sceneAligned, *scene, size*3*sizeof(double));

  // Apply rotation
  gsl_matrix_view points = gsl_matrix_view_array(*sceneAligned, size, 3);
  gsl_matrix* points_tmp = gsl_matrix_alloc(size, 3);
  gsl_matrix_view R = gsl_matrix_submatrix(_T->getBuffer(), 0, 0, 3, 3);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &points.matrix, &R.matrix, 0.0, points_tmp);
  gsl_matrix_memcpy(&points.matrix, points_tmp);
  gsl_matrix_free(points_tmp);

  // Add translation
  gsl_vector_view x  = gsl_matrix_column(&points.matrix, 0);
  gsl_vector_view y  = gsl_matrix_column(&points.matrix, 1);
  gsl_vector_view z  = gsl_matrix_column(&points.matrix, 2);
  gsl_vector_view tr = gsl_matrix_column(_T->getBuffer(), 3);
  gsl_vector_add_constant(&x.vector, gsl_vector_get(&tr.vector,0));
  gsl_vector_add_constant(&y.vector, gsl_vector_get(&tr.vector,1));
  gsl_vector_add_constant(&z.vector, gsl_vector_get(&tr.vector,2));

  for(unsigned int i=0; i<size; i++)
  {
    //double* pt = gsl_matrix_ptr(&points.matrix, i, 0);
    //double* pt2 = &((*scene)[3*i]);
    double* pt = sceneAligned[i];
    //if(mask[i]) cout << pt[0] << " " << pt[1] << " " << pt[2] << " " << pt2[0] << " " << pt2[1] << " " << pt2[2] << endl;
    if(pt[0]<_xMin || pt[0]>_xMax || pt[1]<_yMin || pt[1]>_yMax || pt[2]<_zMin || pt[2]>_zMax)
      mask[i] = false;
  }
  System<double>::deallocate(sceneAligned);
}

}

