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
class MatrixView : public Matrix
{
public:

  MatrixView(Matrix* M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols);


	/**
	 * Destructor
	 */
	~MatrixView();

private:

	/**
	 * Internal GSL representation
	 */
	gsl_matrix_view _V;

};

}

#endif //MATRIXVIEW_H
