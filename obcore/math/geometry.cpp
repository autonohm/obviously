#include "geometry.h"
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <iostream>

using namespace std;

namespace obvious
{

void calculatePerspective(gsl_matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample)
{

  int points = cloud->size() / subsample + 1;

  gsl_matrix* A = gsl_matrix_alloc(2 * points, 11);
  gsl_vector* b = gsl_vector_alloc(2 * points);

  int k = 0;
  int* indices = cloud->getIndices();

  for (unsigned int i = 0; i < cloud->size(); i += subsample)
  {
    int idx = indices[i];
    double* point = (*cloud)[i];
    double ik = (double) (idx % nW);
    double jk = (double) (idx / nW);
    double xk = point[0];
    double yk = point[1];
    double zk = point[2];

    gsl_matrix_set(A, k, 0, xk);
    gsl_matrix_set(A, k, 1, yk);
    gsl_matrix_set(A, k, 2, zk);
    gsl_matrix_set(A, k, 3, 1.0);
    gsl_matrix_set(A, k, 4, 0.0);
    gsl_matrix_set(A, k, 5, 0.0);
    gsl_matrix_set(A, k, 6, 0.0);
    gsl_matrix_set(A, k, 7, 0.0);
    gsl_matrix_set(A, k, 8, -ik * xk);
    gsl_matrix_set(A, k, 9, -ik * yk);
    gsl_matrix_set(A, k, 10, -ik * zk);

    gsl_matrix_set(A, k + 1, 0, 0.0);
    gsl_matrix_set(A, k + 1, 1, 0.0);
    gsl_matrix_set(A, k + 1, 2, 0.0);
    gsl_matrix_set(A, k + 1, 3, 0.0);
    gsl_matrix_set(A, k + 1, 4, xk);
    gsl_matrix_set(A, k + 1, 5, yk);
    gsl_matrix_set(A, k + 1, 6, zk);
    gsl_matrix_set(A, k + 1, 7, 1.0);
    gsl_matrix_set(A, k + 1, 8, -jk * xk);
    gsl_matrix_set(A, k + 1, 9, -jk * yk);
    gsl_matrix_set(A, k + 1, 10, -jk * zk);

    gsl_vector_set(b, k, ik);
    gsl_vector_set(b, k + 1, jk);
    k += 2;

  }

  gsl_vector* x = gsl_vector_alloc(11);

  // QR decomposition
  /*
   gsl_vector* res = gsl_vector_alloc (2*points);
   gsl_vector* tau = gsl_vector_alloc(11);
   gsl_linalg_QR_decomp (A, tau);
   gsl_linalg_QR_lssolve (A, tau, b, x, res);
   gsl_vector_free(tau);
   gsl_vector_free(res);
   */

  // SVD
  gsl_vector* s = gsl_vector_alloc(11);
  gsl_matrix* V = gsl_matrix_alloc(11, 11);
  gsl_vector* work = gsl_vector_alloc(11);
  //gsl_linalg_SV_decomp(A, V, s, work);
  gsl_linalg_SV_decomp_jacobi(A, V, s);
  gsl_linalg_SV_solve(A, V, s, b, x);
  gsl_vector_free(work);
  gsl_matrix_free(V);
  gsl_vector_free(s);

  for (int i = 0; i < 11; i++)
    gsl_matrix_set(P, i / 4, i % 4, gsl_vector_get(x, i));
  gsl_matrix_set(P, 2, 3, 1.0);

  gsl_matrix_free(A);
  gsl_vector_free(b);
  gsl_vector_free(x);

}

void calculatePerspective_cblas(gsl_matrix* P, CartesianCloud3D* cloud, int nW, int nH, int subsample)
{

  int points = cloud->size() / subsample + 1;

  gsl_matrix* A = gsl_matrix_alloc(2 * points, 11);
  gsl_vector* b = gsl_vector_alloc(2 * points);

  int* indices = cloud->getIndices();

  int k = 0;
  for (unsigned int i = 0; i < cloud->size(); i += subsample)
  {
    int idx = indices[i];

    double* point = (*cloud)[i];
    double ik = (double) (idx % nW);
    double jk = (double) (idx / nW);
    double xk = point[0];
    double yk = point[1];
    double zk = point[2];

    gsl_matrix_set(A, k, 0, xk);
    gsl_matrix_set(A, k, 1, yk);
    gsl_matrix_set(A, k, 2, zk);
    gsl_matrix_set(A, k, 3, 1.0);
    gsl_matrix_set(A, k, 4, 0.0);
    gsl_matrix_set(A, k, 5, 0.0);
    gsl_matrix_set(A, k, 6, 0.0);
    gsl_matrix_set(A, k, 7, 0.0);
    gsl_matrix_set(A, k, 8, -ik * xk);
    gsl_matrix_set(A, k, 9, -ik * yk);
    gsl_matrix_set(A, k, 10, -ik * zk);

    gsl_matrix_set(A, k + 1, 0, 0.0);
    gsl_matrix_set(A, k + 1, 1, 0.0);
    gsl_matrix_set(A, k + 1, 2, 0.0);
    gsl_matrix_set(A, k + 1, 3, 0.0);
    gsl_matrix_set(A, k + 1, 4, xk);
    gsl_matrix_set(A, k + 1, 5, yk);
    gsl_matrix_set(A, k + 1, 6, zk);
    gsl_matrix_set(A, k + 1, 7, 1.0);
    gsl_matrix_set(A, k + 1, 8, -jk * xk);
    gsl_matrix_set(A, k + 1, 9, -jk * yk);
    gsl_matrix_set(A, k + 1, 10, -jk * zk);

    gsl_vector_set(b, k, ik);
    gsl_vector_set(b, k + 1, jk);
    k += 2;
  }

  /* Calculate x  = (At * A)^-1 * At * b
   *           M  = (At * A)
   *           Mi = M^-1
   *           N  = Mi * At
   *           x  = N * b
   */

  gsl_vector* x = gsl_vector_alloc(11);

  gsl_matrix* M = gsl_matrix_alloc(11, 11);
  gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, A, A, 0.0, M);

  int s;
  gsl_permutation * p = gsl_permutation_alloc(11);

  gsl_linalg_LU_decomp(M, p, &s);
  gsl_matrix* Mi = gsl_matrix_alloc(11, 11);
  gsl_linalg_LU_invert(M, p, Mi);

  gsl_matrix* N = gsl_matrix_alloc(11, 2 * points);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Mi, A, 0.0, N);
  gsl_blas_dgemv(CblasNoTrans, 1.0, N, b, 0.0, x);

  for (int i = 0; i < 11; i++)
    gsl_matrix_set(P, i / 4, i % 4, gsl_vector_get(x, i));
  gsl_matrix_set(P, 2, 3, 1.0);

  gsl_matrix_free(N);
  gsl_matrix_free(Mi);
  gsl_matrix_free(M);
  gsl_matrix_free(A);
  gsl_vector_free(b);
  gsl_vector_free(x);

}

bool axisAngle(Matrix M, gsl_vector* axis, double* angle)
{
  int rows = M.getRows();
  int cols = M.getCols();

  if((rows != 3 && cols != 3) && (rows != 4 && cols != 4))
  {
    printf("WARNING Matrix::axisAngle: axis angle representation only valid for 3x3 or 4x4 matrices\n");
    return false;
  }

  double trace = M.trace();
  if(rows==4) trace -= M[3][3];

  /**
   * Fix for imprecise rotation matrices
   * ToDo: proof correctness
   */
  if(trace>3.0) trace = 3.0;
  if(trace<-1.0) trace = -1.0;

  *angle = acos((trace - 1.0)/2.0);

  double w = 1.0 / (2.0 * sin(*angle));
  double a1 = w * (M[2][1] - M[1][2]);
  double a2 = w * (M[0][2] - M[2][0]);
  double a3 = w * (M[1][0] - M[0][1]);
  gsl_vector_set(axis, 0, a1);
  gsl_vector_set(axis, 1, a2);
  gsl_vector_set(axis, 2, a3);

  return true;
}

}
