#ifndef FLANNPAIRASSIGNMENT_H
#define FLANNPAIRASSIGNMENT_H

#include <flann/flann.hpp>
#include "obcore/math/mathbase.h"
#include "obvision/icp/assign/PairAssignment.h"


using std::vector;

namespace obvious
{

/**
 * @class FlannPairAssignment
 * @brief Encapsulates neighbor searching with approximate nearest neighbor algorithm (FLANN)
 * @author Stefan May
 **/
class FlannPairAssignment : public PairAssignment
{
public:
	FlannPairAssignment(){init(0.0);};
	
	/**
	 * Standard constructor
	 * @param dimension dimensionality of dataset
	 * @param eps used for searching eps-approximate neighbors
	 **/
	FlannPairAssignment(int dimension, double eps = 0.0) : PairAssignment(dimension) {init(eps);};
	
	/**
	 * Standard destructor
	 **/
	~FlannPairAssignment();

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
	void init(double eps);

	flann::Matrix<double>* _dataset;
	flann::Index<flann::L2<double> >* _index;

	double _eps;
};

}

#endif
