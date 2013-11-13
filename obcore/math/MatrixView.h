#ifndef MATRIXVIEW_H__
#define MATRIXVIEW_H__

#include "Matrix.h"

#include <iostream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Matrix
 * @brief Matrix abstraction layer of GSL
 * @author Stefan May
 */
class MatrixView
{
public:

  MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols);

  MatrixView(double* data, unsigned int rows, unsigned int cols);

	/**
	 * Destructor
	 */
	~MatrixView();

	double* operator [] (unsigned int i);

  /**
   * Multiplication operator
   * @param M matrix multiplied with this matrix view
   * @return this matrix view instance
   */
  MatrixView  &operator *= (const Matrix &M);

  void multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2);

  void print();

private:

	gsl_matrix_view _V;

};

}

#endif //MATRIXVIEW_H
