#ifndef VECTORVIEW_H__
#define VECTORVIEW_H__

#include "Matrix.h"

#include <iostream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class VectorView
 * @brief Vector view abstraction layer of GSL
 * @author Stefan May
 */
class VectorView
{
public:

  VectorView(Matrix* M, unsigned int index, bool column = false);

  VectorView(double* data, unsigned int size);
  VectorView(double* data, unsigned int size, unsigned int stride);

	/**
	 * Destructor
	 */
	~VectorView();

	double operator [] (unsigned int i);

	double* ptr();

	void addConstant(const double val);

  void print();

private:

	gsl_vector_view _V;

};

}

#endif //VECTORVIEW_H
