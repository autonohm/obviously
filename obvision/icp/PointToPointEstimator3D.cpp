#include "PointToPointEstimator3D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

using namespace obvious;

namespace obvious
{

PointToPointEstimator3D::PointToPointEstimator3D()
{
  _model  = NULL;
  _scene  = NULL;
  _rms    = 0.0;
  _cm[0]  = 0.0;
  _cm[1]  = 0.0;
  _cm[2]  = 0.0;
  _cs[0]  = 0.0;
  _cs[1]  = 0.0;
  _cs[2]  = 0.0;
  _pairs = NULL;
  _iterations = 0;
}

PointToPointEstimator3D::~PointToPointEstimator3D()
{
	
}

void PointToPointEstimator3D::setModel(double**model, unsigned int size, double** normals)  // normals are ignored in this class
{
  _model = model;
}

void PointToPointEstimator3D::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
  _scene = scene;
}

void PointToPointEstimator3D::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;

  _rms = 0.0;

  // Compute centroids
  _cm[0] = 0.0;         _cs[0] = 0.0;
  _cm[1] = 0.0;         _cs[1] = 0.0;
  _cm[2] = 0.0;         _cs[2] = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel = _model[pair.indexFirst];
    double* pointScene = _scene[pair.indexSecond];

    _cm[0] += pointModel[0];        _cs[0] += pointScene[0];
    _cm[1] += pointModel[1];        _cs[1] += pointScene[1];
    _cm[2] += pointModel[2];        _cs[2] += pointScene[2];

    _rms += distSqr3D(pointModel,pointScene);
  }
  double dSize = (double)size;
  _rms /= dSize;
  _cm[0] /= dSize;        _cs[0] /= dSize;
  _cm[1] /= dSize;        _cs[1] /= dSize;
  _cm[2] /= dSize;        _cs[2] /= dSize;

  _rms = sqrt(_rms);
}

double PointToPointEstimator3D::getRMS()
{
  return _rms;
}

unsigned int PointToPointEstimator3D::getIterations(void)
{
  return _iterations;
}

void PointToPointEstimator3D::estimateTransformation(gsl_matrix* T)
{
  unsigned int i;
  int          r, c;

  double** pm;
  double** ps;

  int size = _pairs->size();
  _iterations++;

  System<double>::allocate(size,3, pm);
  System<double>::allocate(size,3, ps);

  //Calculate centered point pairs
  for(i = 0; i < size; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];
    double* pointModel         = _model[pair.indexFirst];
    double* pointScene         = _scene[pair.indexSecond];
    pm[i][0]                   = pointModel[0] - _cm[0];
    pm[i][1]                   = pointModel[1] - _cm[1];
    pm[i][2]                   = pointModel[2] - _cm[2];
    ps[i][0]                   = pointScene[0] - _cs[0];
    ps[i][1]                   = pointScene[1] - _cs[1];
    ps[i][2]                   = pointScene[2] - _cs[2];
  }

  //Fill H matrix
  gsl_matrix* H = gsl_matrix_alloc(3,3);
  gsl_matrix_set_zero(H);

  for(r = 0; r < 3; r++) {
    for(c = 0; c < 3; c++) {
      double val = 0.0;
      for(i = 0; i < size; i++) {
        val += ps[i][r] * pm[i][c];
      }
      gsl_matrix_set(H, r, c, val);
    }
  }

  // compute singular value decomposition
  gsl_vector* s    = gsl_vector_alloc(3);
  gsl_matrix* V    = gsl_matrix_alloc(3,3);
  gsl_vector* work = gsl_vector_alloc(3);
  gsl_linalg_SV_decomp(H, V, s, work);
  gsl_vector_free(work);

  // R = V * U'
  gsl_matrix_set_identity(T);
  gsl_matrix_view R = gsl_matrix_submatrix(T, 0, 0, 3, 3);
  gsl_vector_view t = gsl_matrix_subcolumn(T, 3, 0, 3);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, V, H, 0.0, &R.matrix);

  // Calculate translation
  gsl_vector* col = gsl_vector_alloc(3);
  for(i = 0; i < 3; i++)
    gsl_vector_set(col, i, _cs[i]);
  gsl_vector* tmp = gsl_vector_alloc(3);
  gsl_blas_dgemv(CblasNoTrans, 1.0, &R.matrix, col, 0.0, tmp);

  gsl_vector_set(&t.vector, 0, _cm[0]);
  gsl_vector_set(&t.vector, 1, _cm[1]);
  gsl_vector_set(&t.vector, 2, _cm[2]);
  gsl_vector_sub(&t.vector, tmp);

  gsl_vector_free(tmp);
  gsl_vector_free(col);

  gsl_matrix_free(V);
  gsl_vector_free(s);
  gsl_matrix_free(H);

  System<double>::deallocate(pm);
  System<double>::deallocate(ps);
}

}
