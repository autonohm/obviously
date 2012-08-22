#ifndef NORMALSESTIMATOR_H_
#define NORMALSESTIMATOR_H_

#include <iostream>
using namespace std;

#include "obcore/base/CartesianCloud.h"
#include "obcore/base/System.h"

#include <gsl/gsl_matrix.h>
#include "obcore/math/Matrix.h"

using namespace obvious;

namespace obvious
{

/**
 * @class NormalsEstimator
 * @brief Estimation of normals of a point cloud
 * @author Stefan May
 **/
class NormalsEstimator
{
public:
	/**
	 * Standard constructor
	 */
  NormalsEstimator();
		 
	/**
	 * Destructor
	 */
	~NormalsEstimator();
	
	/**
   *
   */
	void estimateNormals3DGrid(unsigned int cols, unsigned int rows, double* coords, bool* mask, double* normals);

	/**
   *
   */
	void estimateNormalsReverseMapping(gsl_matrix* coords, gsl_matrix* P, int w, int h, gsl_matrix* normals);

	/**
	 *
	 */
	void estimateNormalsFLANN(gsl_matrix* coords, gsl_matrix* normals);
	void estimateNormalsANN(gsl_matrix* coords, gsl_matrix* normals);

private:

};

}

#endif /*NORMALSESTIMATOR_H_*/
