#include "PointToPlaneEstimator3D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

using namespace std;
using namespace obvious;

namespace obvious
{

PointToPlaneEstimator3D::PointToPlaneEstimator3D()
{
  _model   = NULL;
  _scene   = NULL;
  _normals = NULL;
  _pairs   = NULL;
}

PointToPlaneEstimator3D::~PointToPlaneEstimator3D()
{
	
}

void PointToPlaneEstimator3D::setModel(double** model, unsigned int size, double** normals)
{
	_model   = model;
	_normals = normals;
}

void PointToPlaneEstimator3D::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
	_scene = scene;
}

void PointToPlaneEstimator3D::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;

  _rms = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel = _model[pair.indexFirst];
    double* pointScene = _scene[pair.indexSecond];
    _rms += distSqr3D(pointModel,pointScene);
  }
  _rms /= (double)size;

  _rms = sqrt(_rms);
}

double PointToPlaneEstimator3D::getRMS()
{
	return _rms;
}

/**
 * Estimation based on paper: Sickel, Konrad; Bubnik, Vojtech, Iterative Closest Point Algorithm for Rigid Registration of Ear Impressions,
 * In: Bauman Moscow State Technical University (Eds.) Proceedings of the 6-th Russian-Bavarian Conference on Bio-Medical Engineering
 * (6th Russian Bavarian Conference on Bio-Medical Engineering Moscow, Russia 8-12.11.2010) 2010, pp. 142-145
 */
void PointToPlaneEstimator3D::estimateTransformation(gsl_matrix* T)
{
  if(_normals==NULL)
  {
    cout << "WARNING (PointToPlaneEstimator3D::estimateTransformation): Normals not set." << endl;
    return;
  }
  unsigned int i;

  int size = _pairs->size();

  double A_buf[36];
  gsl_matrix_view A     = gsl_matrix_view_array(A_buf, 6, 6);
  gsl_matrix_set_zero(&A.matrix);

  double b_buf[6];
  gsl_vector_view b     = gsl_vector_view_array(b_buf, 6);
  gsl_vector_set_zero(&b.vector);

  for(i = 0; i < size; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];

    double* q         = _model[pair.indexFirst];
    double* p         = _scene[pair.indexSecond];
    double* n         = _normals[pair.indexFirst];

    double pxn[3];
    pxn[0] = p[1]*n[2] - p[2]*n[1];
    pxn[1] = p[2]*n[0] - p[0]*n[2];
    pxn[2] = p[0]*n[1] - p[1]*n[0];

    double a12 = pxn[0]*pxn[1];
    double a13 = pxn[0]*pxn[2];
    double a14 = pxn[0]*n[0];
    double a15 = pxn[0]*n[1];
    double a16 = pxn[0]*n[2];
    double a23 = pxn[1]*pxn[2];
    double a24 = pxn[1]*n[0];
    double a25 = pxn[1]*n[1];
    double a26 = pxn[1]*n[2];
    double a34 = pxn[2]*n[0];
    double a35 = pxn[2]*n[1];
    double a36 = pxn[2]*n[2];
    double a45 = n[0]*n[1];
    double a46 = n[0]*n[2];
    double a56 = n[1]*n[2];
    A_buf[0]+=pxn[0]*pxn[0];   A_buf[1]+=a12;             A_buf[2]+=a13;             A_buf[3]+=a14;           A_buf[4]+=a15;           A_buf[5]+=a16;
    A_buf[6]+=a12;             A_buf[7]+=pxn[1]*pxn[1];   A_buf[8]+=a23;             A_buf[9]+=a24;           A_buf[10]+=a25;          A_buf[11]+=a26;
    A_buf[12]+=a13;            A_buf[13]+=a23;            A_buf[14]+=pxn[2]*pxn[2];  A_buf[15]+=a34;          A_buf[16]+=a35;          A_buf[17]+=a36;
    A_buf[18]+=a14;            A_buf[19]+=a24;            A_buf[20]+=a34;            A_buf[21]+=n[0]*n[0];    A_buf[22]+=a45;          A_buf[23]+=a46;
    A_buf[24]+=a15;            A_buf[25]+=a25;            A_buf[26]+=a35;            A_buf[27]+=a45;          A_buf[28]+=n[1]*n[1];    A_buf[29]+=a56;
    A_buf[30]+=a16;            A_buf[31]+=a26;            A_buf[32]+=a36;            A_buf[33]+=a46;          A_buf[34]+=a56;          A_buf[35]+=n[2]*n[2];

    double pmq[3];
    pmq[0] = p[0]-q[0];
    pmq[1] = p[1]-q[1];
    pmq[2] = p[2]-q[2];
    double tmp = (pmq[0]*n[0]+pmq[1]*n[1]+pmq[2]*n[2]);
    b_buf[0]-= pxn[0]*tmp;
    b_buf[1]-= pxn[1]*tmp;
    b_buf[2]-= pxn[2]*tmp;
    b_buf[3]-= n[0]*tmp;
    b_buf[4]-= n[1]*tmp;
    b_buf[5]-= n[2]*tmp;
  }
  gsl_vector* x         = gsl_vector_alloc(6);
  gsl_permutation* perm = gsl_permutation_alloc(6);
  int sig;
  gsl_linalg_LU_decomp(&A.matrix, perm, &sig);
  gsl_linalg_LU_solve(&A.matrix, perm, &b.vector, x);
  gsl_permutation_free(perm);

  gsl_vector_view t = gsl_matrix_subcolumn(T, 3, 0, 3);
  gsl_vector_set(&t.vector, 0, gsl_vector_get(x, 3));
  gsl_vector_set(&t.vector, 1, gsl_vector_get(x, 4));
  gsl_vector_set(&t.vector, 2, gsl_vector_get(x, 5));

  double phi   = gsl_vector_get(x, 0);
  double theta = gsl_vector_get(x, 1);
  double psi   = gsl_vector_get(x, 2);

  gsl_matrix_view R = gsl_matrix_submatrix(T, 0, 0, 3, 3);
  /**
   * This approximation originates from the paper, but the precise calculation of R (see below) provides better results
   */
  /*gsl_matrix_set(&R.matrix, 0, 0, 1);
  gsl_matrix_set(&R.matrix, 0, 1, -psi);
  gsl_matrix_set(&R.matrix, 0, 2, theta);
  gsl_matrix_set(&R.matrix, 1, 0, psi);
  gsl_matrix_set(&R.matrix, 1, 1, 1);
  gsl_matrix_set(&R.matrix, 1, 2, -phi);
  gsl_matrix_set(&R.matrix, 2, 0, -theta);
  gsl_matrix_set(&R.matrix, 2, 1, phi);
  gsl_matrix_set(&R.matrix, 2, 2, 1);*/

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

  gsl_vector_free(x);
}

}
