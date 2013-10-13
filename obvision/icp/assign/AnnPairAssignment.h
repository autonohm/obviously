#ifndef ANNPAIRASSIGNMENT_H
#define ANNPAIRASSIGNMENT_H

#include <ANN/ANN.h>
#include <ANN/ANNx.h>
#include <ANN/ANNperf.h>
#include "obcore/math/mathbase.h"
#include "obvision/icp/assign/PairAssignment.h"


using std::vector;

namespace obvious
{

/**
 * @class AnnPairAssignment
 * @brief Encapsulates neighbor searching with approximate nearest neighbor algorithm (ANN)
 * @author Stefan May
 **/
class AnnPairAssignment : public PairAssignment
{
public:
	AnnPairAssignment(){init();};
	
	/**
	 * Standard constructor
	 **/
	AnnPairAssignment(int dimension) : PairAssignment(dimension) {init();};
	
	/**
	 * Standard destructor
	 **/
	~AnnPairAssignment();

	/**
	 * Set maximum number of point to be visited until interruption.
	 * This option enhances the performance, but can raise non-optimal matches.
	 * @param visitPoints number of points to be visited
	 */
	void setMaxVisitPoints(unsigned int visitPoints);
		
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
	
	/**
	 * Get the constructed kd-tree
	 * @return Pointer to the internally used kd-tree-structure constructed by libANN
	 */
	ANNkd_tree* getTree();
	
private:

	/**
	 * Private initialization routine called by constructors
	 */
	void init();

	/**
	 * KD search tree
	 */
	ANNkd_tree* _tree;

};

}

#endif
