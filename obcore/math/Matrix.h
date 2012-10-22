#ifndef MATRIX_H__
#define MATRIX_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include <iostream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Matrix
 * @brief
 * @author Stefan May
 */
class Matrix
{
public:
  Matrix(unsigned int rows, unsigned int cols);

  Matrix(const Matrix &M);

  ~Matrix();

  Matrix  &operator =  (const Matrix &M);

  Matrix  &operator *= (const Matrix &M);

  double* operator [] (unsigned int i);

  friend Matrix operator * (const Matrix &M1, const Matrix &M2);

  friend ostream& operator <<(ostream &os, Matrix &M);

  gsl_matrix* getBuffer();

  void getData(double* array);

  void setData(double* array);

  unsigned int getRows();

  unsigned int getCols();

  void setIdentity();

  void setZero();

  Matrix getInverse();

  void invert();

  double trace();

  gsl_vector* centroid();

  /**
   * perform principle component analysis
   * @return matrix in layout [x1_from x1_to y1_from y1_to z1_from z1_to; x2...]
   */
  Matrix* pcaAnalysis();

  static Matrix* TranslationMatrix44(double tx, double ty, double tz);

  void print();

private:
  gsl_matrix* _M;
  gsl_matrix* _work;
};

}

#endif //MATRIX_H
