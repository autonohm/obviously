#ifndef ICP_H_
#define ICP_H_

#include <iostream>
using namespace std;

#include "obvision/icp/assign/PairAssignment.h"
#include "obvision/icp/assign/filter/IPostAssignmentFilter.h"
#include "obvision/icp/IRigidEstimator.h"
#include "obcore/base/CartesianCloud.h"
#include "obcore/base/System.h"

#include <gsl/gsl_matrix.h>
#include "obcore/math/Matrix.h"

using namespace obvious;

namespace obvious
{

/**
 * ICP return states
 */
enum EnumIcpState { ICP_IDLE 			= 0,
					ICP_PROCESSING 		= 1,
					ICP_NOTMATCHABLE 	= 2,
					ICP_MAXITERATIONS 	= 3,
					ICP_TIMEELAPSED 	= 4,
					ICP_SUCCESS 		= 5,
					ICP_CONVERGED   = 6,
					ICP_ERROR			= 7 };

/**
 * @class Icp
 * @brief Represents the iterative closest point algorithm
 * @author Stefan May and Dirk Holz
 **/
class Icp
{
public:
	/**
	 * Standard constructor
	 * @param assigner pair assignment strategy for point clouds
	 * @param estimator transformation estimator
	 * @param pDistanceThreshold thresholding strategy
	 */
	Icp(PairAssignment* assigner,
		 IRigidEstimator* estimator);
		 
	/**
	 * Destructor
	 */
	~Icp();
	
	/**
	 * convert enumeration to char*
	 * @param eState state enumeration
	 * @return state string
	 */
	const char* state2char(EnumIcpState eState);
	
	/**
	 * Accessor to pair assigner
	 * @return assigner
	 */
	PairAssignment* getPairAssigner();
	
	/**
	 * Accessor to rigid estimator
	 * @return estimator
	 */
	IRigidEstimator* getRigidEstimator();
	
	/**
	 * Convenience method extracting all points from cloud to double array
	 * @param cloud model cloud
	 */
	void setModel(gsl_matrix* coords, gsl_matrix* normals = NULL);
	
	/**
	 * Convenience method extracting data from cloud to double array
	 * @param cloud scene cloud 
	 */
	void setScene(gsl_matrix* coords, gsl_matrix* normals = NULL);
	
	/**
	 * Set maximal RMS error interrupting iteration
	 * @param rms RMS error
	 */
	void setMaxRMS(double rms);
	
	/**
	 * Reset state of ICP algorithm (resets iteration count, estimated transformation and point pairs)
	 */
	 void reset();
	
	/**
	 * Get maximal RMS error interrupting iteration
	 * @return RMS error
	 */
	double getMaxRMS();
	
	/**
	 * Set maximal iteration steps
	 * @param nIterations maximum number of iteration steps
	 */
	void setMaxIterations(unsigned int iterations);
	
	unsigned int getMaxIterations();

	void setConvergenceCounter(unsigned int convCnt);

	unsigned int getConvergenceCounter();

	EnumIcpState step(double* rms);

	/**
	 * Start iteration.
	 * @param rms return value of RMS error
	 * @return state of finished and successful processing
	 */
	EnumIcpState iterate(double* rms, unsigned int* iterations);
	
	/**
	 * get final rotation matrix determined through iteration
	 * @return final rotation matrix
	 */
	Matrix* getFinalTransformation();
	Matrix* getLastTransformation();
	
private:
	
	void applyTransformation(double** data, unsigned int size, unsigned int dim, Matrix* T);

	/**
	 * Internel memory check routine
	 * @param rows row size of needed memory
	 * @param memsize row size of target memory
	 * @param mem target memory
	 */
	void checkMemory(unsigned int rows, unsigned int cols, unsigned int &memsize, double** &mem);
	
	/**
	 * maximal RMS error interrupting iteration
	 */
	double _maxRMS;
	
	/**
	 * maximum number of iterations
	 */
	unsigned int _maxIterations;

	/**
	 * the model
	 */
	double** _model;

	double** _normalsM;
  double** _normalsS;

	/** 
	 * size of model 
	 */
	unsigned int _sizeModel;
	
	/**
	 * size of internal model buffer 
	 */
	unsigned int _sizeModelBuf;

	/**
	 * size of internal scene buffer 
	 */
	unsigned int _sizeSceneBuf;
		
	/** 
	 * the scene
	 */
	double** _scene;

	/**
	 * size of scene
	 */
	unsigned int _sizeScene;

	/**
	 * point assigner
	 */
	PairAssignment* _assigner;
	
	/**
	 * transformation estimator
	 */
	IRigidEstimator* _estimator;
	
	/**
	 * final matrix, found after iteration
	 */
	
	//gsl_matrix* _finalMatrix;
	Matrix* _Tfinal;
	Matrix* _Tlast;

	/**
	 * Dimension of space
	 */
	int _dim;

	unsigned int _convCnt;
};

}

#endif /*ICP_H_*/
