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
 * @class MatrixView
 * @brief Matrix view abstraction layer of GSL. There is no data buffer instantiated, but referenced.
 * @author Stefan May
 */
class MatrixView
{
public:

  /**
   * Constructor
   * @param M source matrix
   * @param i start row
   * @param j start column
   * @param rows number of rows
   * @param cols number of columns
   */
  MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols);

  /**
   * Constructor
   * @param data one-dimensional buffer
   * @param rows number of rows of MatrixView
   * @param cols number of columns of MatrixView
   */
  MatrixView(double* data, unsigned int rows, unsigned int cols);

	/**
	 * Destructor
	 */
	~MatrixView();

	/**
	 * Row accessor
	 * @param i row index
	 */
	double* operator [] (unsigned int i);

  /**
   * Multiplication operator
   * @param M matrix multiplied with this matrix view
   * @return this matrix view instance
   */
  MatrixView  &operator *= (const Matrix &M);

  /**
   * Multiply matrix to this one
   * @param M matrix multiplied from right
   * @param transposeArg1 transpose current matrix
   * @param transposeArg2 transpose matrix M
   */
  void multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2);

  /**
   * Print matrix to stdout
   */
  void print();

private:

};

}

#endif //MATRIXVIEW_H
