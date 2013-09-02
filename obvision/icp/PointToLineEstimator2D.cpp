#include "PointToLineEstimator2D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

using namespace std;
using namespace obvious;

enum Coords
{
  x = 0,
  y = 1,
  z = 2
};

namespace obvious
{

PointToLine2DEstimator ::PointToLine2DEstimator ()
{
  _model        = NULL;
  _scene        = NULL;
  _normals      = NULL;
  _pairs        = NULL;
  _rms          = 10000.0;
  _iterations   = 0;

  for (unsigned int i=0 ; i<=2 ; i++)
  {
    _cs[i] = 0.0;
    _cm[i] = 0.0;
  }
}

PointToLine2DEstimator::~PointToLine2DEstimator (void)
{

}

void PointToLine2DEstimator::setModel(double** model, unsigned int size, double** normals)
{
  _model   = model;
  _normals = normals;
}

void PointToLine2DEstimator::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
  _scene = scene;
}

void PointToLine2DEstimator::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;
  _rms = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++) {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel         = _model[pair.indexFirst];
    double* pointScene         = _scene[pair.indexSecond];
    double* pointNorm          = _normals[pair.indexFirst];

    _rms += distSqr2D(pointModel,pointScene)*abs2D(pointNorm);
  }
  _rms /= (double)size;
  _rms = sqrt(_rms);
}

double PointToLine2DEstimator::getRMS()
{
  return _rms;
}

/**
 * Estimation based on paper: Sickel, Konrad; Bubnik, Vojtech, Iterative Closest Point Algorithm for Rigid Registration of Ear Impressions,
 * In: Bauman Moscow State Technical University (Eds.) Proceedings of the 6-th Russian-Bavarian Conference on Bio-Medical Engineering
 * (6th Russian Bavarian Conference on Bio-Medical Engineering Moscow, Russia 8-12.11.2010) 2010, pp. 142-145
 */
void PointToLine2DEstimator::estimateTransformation(gsl_matrix* T)
{
  if(_normals==NULL) {
    cout << "WARNING (" << __PRETTY_FUNCTION__ << "): Normals not set." << endl;
    return;
  }
  _iterations++;
  unsigned int size = _pairs->size();

  unsigned int A_cols = 3, A_rows =3;
  double A_buf[A_cols*A_rows];
  gsl_matrix_view A     = gsl_matrix_view_array(A_buf, A_cols, A_rows);
  gsl_matrix_set_zero(&A.matrix);

  double b_buf[A_cols];
  gsl_vector_view b     = gsl_vector_view_array(b_buf, A_cols);
  gsl_vector_set_zero(&b.vector);

  for(unsigned int i=0 ; i< size ; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];

    double* q  = _model[  pair.indexFirst];
    double* p  = _scene[  pair.indexSecond];
    double* n  = _normals[pair.indexFirst];

    double a[3];
    a[x] = 0;
    a[y] = 0;
    a[z] = p[x]*n[y] - p[y]*n[x];

    A_buf[0] += a[z]*a[z];             A_buf[1] += a[z]*n[x];             A_buf[2] += a[z]*n[y];
    A_buf[3] += a[z]*n[x];             A_buf[4] += n[x]*n[x];             A_buf[5] += n[x]*n[y];
    A_buf[6] += a[z]*n[y];             A_buf[7] += n[y]*n[x];             A_buf[8] += n[y]*n[y];

    double pmq[3];
    pmq[x] = p[x]-q[x];
    pmq[y] = p[y]-q[y];
    pmq[z] = 0;

    double tmp = pmq[0]*n[0] + pmq[1]*n[1];

    b_buf[0]-= a[z]*tmp;
    b_buf[1]-= n[x]*tmp;
    b_buf[2]-= n[y]*tmp;
  }

  gsl_vector* x         = gsl_vector_alloc(A_cols);
  gsl_permutation* perm = gsl_permutation_alloc(A_cols);

  int sig;
  gsl_linalg_LU_decomp(&A.matrix, perm, &sig);
  gsl_linalg_LU_solve(&A.matrix,  perm, &b.vector, x);
  gsl_permutation_free(perm);

  const double psi   = gsl_vector_get(x, 0);
  const double theta = 0.0;
  const double phi   = 0.0;
  gsl_matrix_view R = gsl_matrix_submatrix(T, 0, 0, 3, 3);


  double cph = cos(phi);
  double cth = cos(theta);
  double cps = cos(psi);
  double sph = sin(phi);
  double sth = sin(theta);
  double sps = sin(psi);

  gsl_matrix_set(&R.matrix, 0, 0, cth*cps);
  gsl_matrix_set(&R.matrix, 0, 1, -cph*sps+sph*sth*cps);
  gsl_matrix_set(&R.matrix, 0, 2, sph*sth+cph*sth*cps);

  gsl_matrix_set(&R.matrix, 1, 0, cth*sps);
  gsl_matrix_set(&R.matrix, 1, 1, cph*cps+sph*sth*sps);
  gsl_matrix_set(&R.matrix, 1, 2, -sph*cps+cph*sth*sps);

  gsl_matrix_set(&R.matrix, 2, 0, -sth);
  gsl_matrix_set(&R.matrix, 2, 1, sph*cth);
  gsl_matrix_set(&R.matrix, 2, 2, cph*cth);

  gsl_vector_view t = gsl_matrix_subcolumn(T, 3, 0, 3);
  gsl_vector_set(&t.vector, 0, gsl_vector_get(x, 1));
  gsl_vector_set(&t.vector, 1, gsl_vector_get(x, 2));

  gsl_vector_free(x);
}

unsigned int PointToLine2DEstimator::getIterations(void)
{
  return(_iterations);
}

}
