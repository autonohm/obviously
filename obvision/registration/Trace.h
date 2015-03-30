#ifndef TRACE_H_
#define TRACE_H_

#include "obvision/registration/icp/assign/assignbase.h"
#include "obcore/math/linalg/linalg.h"

#include <iostream>
#include <vector>

using namespace std;

namespace obvious
{

/**
 * @struct pair coordinates for 2D or 3D data. Last dimension is ignored for 2D case.
 * @author Stefan May
 */
struct StrTraceCartesianPair
{
  double first[3];
  double second[3];
};

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
	 * @param dim dimensionality
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
   * Set scene of trace record
   * @param scene scene data
   * @param sizeS size of scene
   */
	void setScene(double** scene, unsigned int sizeS);

	/**
	 * Add scene assignment to trace record
	 * @param scene scene data
	 * @param sizeS size of scene
	 * @param pairs assigned pairs coordinates
	 * @param score score
	 * @param id of assignment, e.g., the iteration
	 */
	void addAssignment(double** scene, unsigned int sizeS, vector<StrTraceCartesianPair> pairs, const double score, vector<unsigned int> id = vector<unsigned int>());
	
	/**
	 * Serialize assignment to trace folder
	 * @param folder trace folder (must not be existent)
	 */
	void serialize(const char* folder);

private:
	
	unsigned int _dim;

	obvious::Matrix* _M;

	obvious::Matrix* _S;

	vector<obvious::Matrix*> _scenes;
	
	vector< vector<StrTraceCartesianPair> > _pairs;

	vector< vector<unsigned int> > _ids;

	vector<double> _scores;
};

}

#endif /*TRACE_H_*/
