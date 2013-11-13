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
 * @brief Matrix abstraction layer of GSL
 * @author Stefan May
 */
class Matrix
{
public:
	/**
	 * Constructor
	 * @param rows number of matrix rows
	 * @param cols number of matrix columns
	 * @param data data buffer to be copied ore linked to matrix representation
	 */
	Matrix(unsigned int rows, unsigned int cols, double* data = NULL);

	/**
	 * Copy constructor
	 * @param M matrix to be copied
	 */
	Matrix(const Matrix &M);

	Matrix(Matrix M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols);

	Matrix(){};

	/**
	 * Destructor
	 */
	~Matrix();

	/**
	 * Copy matrix elements
	 * @param M source matrix (size must be equal of this instance)
	 */
	void copy(const Matrix &M);

	/**
	 * Assignment operator
	 * @param M matrix assigned to this one
	 * @return this matrix instance
	 */
	Matrix  &operator =  (const Matrix &M);

	/**
	 * Multiplication operator
	 * @param M matrix multiplied with this one
	 * @return this matrix instance
	 */
	Matrix  &operator *= (const Matrix &M);

	/**
	 * Subtraction operator
	 * @param M matrix subtracted from this one
	 * @return this matrix instance
	 */
	Matrix &operator -= (const Matrix &M);

	/**
	 * Row accessor
	 * @param i row index
	 * @return row elements as array
	 */
	double* operator [] (unsigned int i);

	/**
	 * Multiplication operator
	 * @param M1 1st matrix of product
	 * @param M2 2nd matrix of product
	 * @return matrix product
	 */
	friend Matrix operator * (const Matrix &M1, const Matrix &M2);

	/**
	 * Stream operator
	 * @param os output stream
	 * @param M matrix to be streamed, e.g. printed out
	 */
	friend ostream& operator <<(ostream &os, Matrix &M);

	/**
	 * GSL matrix accessor
	 * @brief get access to internal matrix representation
	 * @return GSL matrix
	 */
	gsl_matrix* getBuffer();

	/**
	 * Data accessor
	 * @param array array to copy data into (must be instanciated outside)
	 */
	void getData(double* array);

	/**
	 * Data mutator
	 * @param array array to copy data from
	 */
	void setData(const double* array);

	/**
	 * Property accessor
	 * @return number of matrix rows
	 */
	unsigned int getRows();

	/**
	 * Property accessor
	 * @return number of matrix columns
	 */
	unsigned int getCols();

	/**
	 * Set matrix to identity
	 */
	void setIdentity();

	/**
	 * Set all matrix elements to zero
	 */
	void setZero();

	/**
	 * Instantiate an inverse of the present matrix
	 * @return inverse matrix as new instance
	 */
	Matrix getInverse();

	/**
	 * Invert present matrix
	 */
	void invert();

	/**
	 * Instantiate the transpose matrix of present matrix
	 * @return transposed matrix as new instance
	 */
	Matrix getTranspose();

	/**
	 * Transpose current matrix
	 */
	void transpose();

	/**
	 * Calculate trace of matrix
	 * @return trace
	 */
	double trace();

	/**
	 * perform principle component analysis
	 * @return matrix in layout [x1_from x1_to y1_from y1_to z1_from z1_to; x2...]
	 */
	Matrix* pcaAnalysis();

	/**
	 * perform singular value decomposition A = U S V'
	 * @param U orthogonal matrix U
	 * @param s singular values
	 * @param V orthogonal square matrix
	 */
	void svd(Matrix* U, double* s, Matrix* V);

	/**
	 * Solve A x = b
	 * @param[in] b b must have elements = number of rows of current matrix
	 * @param[out] x x must have elements = number of rows of current matrix
	 */
	void solve(double* b, double* x);

	/**
	 * Instantiate a 4x4 translation matrix, i.e. identity with last column set to translational input
	 * @param tx x-component of translation
	 * @param ty y-component of translation
	 * @param tz z-component of translation
	 */
	static Matrix* TranslationMatrix44(double tx, double ty, double tz);

	/**
	 * Transform current matrix, i.e. homogeneous coordinates with transformation matrix
	 * @param[in] T transformation matrix
	 */
	void transform(Matrix T);

	/**
	 * Print matrix to output stream
	 */
	void print();

protected:

	/**
	 * Internal GSL representation
	 */
	gsl_matrix* _M;

	/**
	 * Internal GSL work buffer
	 */
	gsl_matrix* _work;

};

}

#endif //MATRIX_H
