#ifndef ICPTRACE_H_
#define ICPTRACE_H_

#include <iostream>
#include <vector>
#include "obvision/icp/assign/assignbase.h"
#include "obcore/math/linalg/linalg.h"

using namespace std;

namespace obvious
{

/**
 * @class IcpTrace
 * @brief Represents a trace module for the iterative closest point algorithm
 * @author Stefan May
 **/
class IcpTrace
{
public:
	/**
	 * Default constructor
	 */
	IcpTrace(unsigned int dim);
		 
	/**
	 * Destructor
	 */
	~IcpTrace();
	
	/**
	 * Reset trace record
	 */
	void reset();

	/**
	 * Add model and scene assignment to trace record
	 * @param model model data
	 * @param sizeM size of model
	 * @param scene scene data
	 * @param sizeS size of scene
	 * @param pairs tuple of assigned indices
	 */
	void addAssignment(double** model, unsigned int sizeM, double** scene, unsigned int sizeS, vector<StrCartesianIndexPair> pairs);
	
	void serialize(char* folder);

private:
	
	unsigned int _dim;

	vector<Matrix*> _models;
	
	vector<Matrix*> _scenes;
	
	vector< vector<StrCartesianIndexPair> > _pairs;
	
};

}

#endif /*ICPTRACE_H_*/
