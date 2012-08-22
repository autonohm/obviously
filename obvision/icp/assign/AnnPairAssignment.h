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
	AnnPairAssignment(int nDimension) : PairAssignment(nDimension) {init();};
	
	/**
	 * Standard destructor
	 **/
	~AnnPairAssignment();

	/**
	 * Set maximum number of point to be visited until interruption.
	 * This option enhances the performance, but can raise non-optimal matches.
	 * @param unVisitPoints number of points to be visited
	 */
	void setMaxVisitPoints(unsigned int unVisitPoints);
		
	/**
	 * Set model as matching base
	 * @param ppdModel array of xy values
	 * @param nSize number of points
	 **/
	void setModel(double** ppdModel, int nSize);
	
	/**
	 * Determine point pairs (nearest neighbors)
	 * @param ppdScene scene to be compared
	 * @param nSize nr of points in scene
	 * @param pairs return value of neighbors
	 * @param nonPairs return value of points with no neighbors
	 */
	void determinePairs(double** ppdScene, bool* msk, int nSize);
	
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
