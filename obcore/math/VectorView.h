#ifndef VECTORVIEW_H__
#define VECTORVIEW_H__

#include <iostream>

#include "Vector.h"

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
  friend class Matrix;
  friend class Vector;

public:

  /**
   * Standard constructor
   */
  VectorView();

  /**
   * Constructor
   * @param data external buffer to be referenced
   * @param size size of external buffer
   */
  VectorView(double* data, unsigned int size);

  /**
   * Constructor
   * @param data external buffer to be referenced
   * @param size size of external buffer
   * @param stride memory alignment, i.e. number of values per row
   */
  VectorView(double* data, unsigned int size, unsigned int stride);

	/**
	 * Destructor
	 */
	~VectorView();

	/**
	 * Row accessor
	 * @param i index
	 * @param return value
	 */
	double& operator [] (unsigned int i);

	/**
	 * Accessor to referenced buffer
	 * @return buffer reference pointer
	 */
	double* ptr();

	/**
	 * Add constant to each element of vector
	 * @param val constant to be added
	 */
	void addConstant(const double val);

	/**
	 * Print data buffer to stdout
	 */
  void print();

private:

	gsl_vector_view _V;

};

}

#endif //VECTORVIEW_H
