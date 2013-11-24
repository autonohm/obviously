#ifndef NORMALSESTIMATOR_H_
#define NORMALSESTIMATOR_H_

#include <iostream>
using namespace std;

#include "obcore/base/CartesianCloud.h"
#include "obcore/base/System.h"

#include "obcore/math/linalg/linalg.h"

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
	void estimateNormalsReverseMapping(Matrix* coords, Matrix* P, int w, int h, Matrix* normals);

	/**
	 *
	 */
	void estimateNormalsFLANN(Matrix* coords, Matrix* normals);
	void estimateNormalsANN(Matrix* coords, Matrix* normals);

private:

};

}

#endif /*NORMALSESTIMATOR_H_*/
