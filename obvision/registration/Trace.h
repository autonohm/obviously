#ifndef TRACE_H_
#define TRACE_H_

#include <iostream>
#include <vector>
#include "obvision/registration/icp/assign/assignbase.h"
#include "obcore/math/linalg/linalg.h"

using namespace std;

namespace obvious
{

/**
 * @class Trace
 * @brief Represents a trace module for the point cloud registration algorithms
 * @author Stefan May
 **/
class Trace
{
public:
	/**
	 * Default constructor
	 */
	Trace(unsigned int dim);
		 
	/**
	 * Destructor
	 */
	~Trace();
	
	/**
	 * Reset trace record
	 */
	void reset();

	/**
   * Set model of trace record
   * @param model model data
   * @param sizeM size of model
   */
	void setModel(double** model, unsigned int sizeM);

	/**
	 * Add scene assignment to trace record
	 * @param scene scene data
	 * @param sizeS size of scene
	 * @param pairs tuple of assigned indices
	 */
	void addAssignment(double** scene, unsigned int sizeS, vector<StrCartesianIndexPair> pairs);
	
	/**
	 * Serialize assignment to trace folder
	 * @param folder trace folder (must not be existent)
	 * @param delay animation delay (specified in delay*1/100s)
	 */
	void serialize(char* folder, unsigned int delay);

private:
	
	unsigned int _dim;

	Matrix* _M;
	
	vector<Matrix*> _scenes;
	
	vector< vector<StrCartesianIndexPair> > _pairs;
	
};

}

#endif /*TRACE_H_*/
