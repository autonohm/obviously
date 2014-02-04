#ifndef ICP_H_
#define ICP_H_

#include <iostream>
using namespace std;

#include "obvision/icp/assign/PairAssignment.h"
#include "obvision/icp/assign/filter/IPostAssignmentFilter.h"
#include "obvision/icp/IRigidEstimator.h"
#include "obcore/base/CartesianCloud.h"
#include "obcore/base/System.h"

#include "obcore/math/linalg/linalg.h"

using namespace obvious;

namespace obvious
{

typedef void (*fptrAssignmentCallback)(double**, double**, unsigned int);

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
	Icp(PairAssignment* assigner, IRigidEstimator* estimator);
		 
	/**
	 * Destructor
	 */
	~Icp();
	
	void setAssignmentCallback(fptrAssignmentCallback fptr);

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
   * Copy model to internal buffer
   * @param coords model coordinates, as tuples or triples
   * @param normals model normals, as tuples or triples, may be NULL
   * @param size number of points, i.e. coordinate triples
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
   */
	void setModel(double* coords, double* normals, const unsigned int size, double probability=1.0);

	/**
	 * Convenience method extracting all points from cloud to double array
   * @param coords model coordinates
   * @param normals model normals, may be NULL
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
	 */
	void setModel(Matrix* coords, Matrix* normals = NULL, double probability=1.0);
	
  /**
   * Copy scene to internal buffer
   * @param coords scene coordinates, as tuples or triples
   * @param normals scene normals, as tuples or triples, may be NULL
   * @param size number of points, i.e. coordinate triples
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
   */
  void setScene(double* coords, double* normals, const unsigned int size, double probability=1.0);

	/**
	 * Convenience method extracting data from cloud to double array
   * @param coords scene coordinates
   * @param normals scene normals, may be NULL
   * @param probability probability of coordinates of being sampled (range [0.0 1.0])
	 */
	void setScene(Matrix* coords, Matrix* normals = NULL, double probability=1.0);
	
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
	 * Set maximum number of iteration steps
	 * @param nIterations maximum number of iteration steps
	 */
	void setMaxIterations(unsigned int iterations);
	
	/**
	 * Get maximum number of iteration steps
	 * @return maximum number of iteration steps
	 */
	unsigned int getMaxIterations();

	/**
	 * If the RMS error is not reducing within this number, the matching process is considered as successful.
	 * @param convCnt convergence counter
	 */
	void setConvergenceCounter(unsigned int convCnt);

	/**
	 * Access convergence counter
	 * @param convCnt convergence counter
	 */
	unsigned int getConvergenceCounter();

	/**
	 * Perform one iteration
	 * @param rms return value of RMS error
	 * @param pairs return value of pair assignments, i.e. number of pairs
	 * @return processing state
	 */
	EnumIcpState step(double* rms, unsigned int* pairs);

	/**
	 * Start iteration
	 * @param rms return value of RMS error
	 * @param pairs return value of pair assignments, i.e. number of pairs
	 * @param iterations return value of performed iterations
	 * @return  processing state
	 */
	EnumIcpState iterate(double* rms, unsigned int* pairs, unsigned int* iterations);
	
	/**
	 * Get final 4x4 rotation matrix determined through iteration
	 * @return final transformation matrix
	 */
	Matrix* getFinalTransformation4x4();

	/**
    * Get final rotation matrix determined through iteration
    * @return final transformation matrix
    */
   Matrix* getFinalTransformation();

	/**
	 * Get last rotation matrix determined within the last iteration step
	 * @return last transformation matrix
	 */
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
	Matrix* _Tfinal4x4;
	Matrix* _Tfinal;
	Matrix* _Tlast;

	/**
	 * Dimension of space
	 */
	int _dim;

	unsigned int _convCnt;

	fptrAssignmentCallback _fptrCallbackPairs;

	bool _reset;
};

}

#endif /*ICP_H_*/
