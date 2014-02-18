#ifndef NABOPAIRASSIGNMENT_H
#define NABOPAIRASSIGNMENT_H

#include "nabo/nabo.h"
using namespace Nabo;
using namespace Eigen;

#include "obcore/math/mathbase.h"
#include "obvision/icp/assign/PairAssignment.h"

using std::vector;

namespace obvious
{

/**
 * @class NaboPairAssignment
 * @brief Encapsulates neighbor searching with libnabo
 * @author Stefan May
 **/
class NaboPairAssignment : public PairAssignment
{
public:
  NaboPairAssignment(){init();};
	
	/**
	 * Standard constructor
	 **/
  NaboPairAssignment(int dimension) : PairAssignment(dimension) {init();};
	
	/**
	 * Standard destructor
	 **/
	~NaboPairAssignment();
		
	/**
	 * Set model as matching base
	 * @param model array of xy values
	 * @param size number of points
	 **/
	void setModel(double** model, int size);
	
	/**
	 * Determine point pairs (nearest neighbors)
	 * @param scene scene to be compared
	 * @param size nr of points in scene
	 * @param pairs return value of neighbors
	 * @param nonPairs return value of points with no neighbors
	 */
	void determinePairs(double** scene, bool* msk, int size);
	
private:

	/**
	 * Private initialization routine called by constructors
	 */
	void init();

	/**
	 * search tree
	 */
	NNSearchF* _nns;

};

}

#endif