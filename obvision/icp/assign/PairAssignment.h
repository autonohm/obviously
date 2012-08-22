#ifndef PAIRASSIGNMENT_H
#define PAIRASSIGNMENT_H

#include <vector>
#include <map>
#include "obvision/icp/assign/assignbase.h"
#include "obvision/icp/assign/filter/IPreAssignmentFilter.h"
#include "obvision/icp/assign/filter/IPostAssignmentFilter.h"
#include <iostream>

using namespace std;

#define DEFAULTDIMENSION 2

namespace obvious
{

/**
 * @class PairAssignment
 * @brief Encapsulates neighbor searching
 * @author Stefan May
 **/
class PairAssignment
{
public:
  /**
   * Standard constructor
   */
  PairAssignment();

  /**
   * Standard constructor
   **/
  PairAssignment(int nDimension);

  /**
   * Standard destructor
   **/
  virtual ~PairAssignment();

  void addPreFilter(IPreAssignmentFilter* filter);
  void addPostFilter(IPostAssignmentFilter* filter);

  /**
   * Access internal container of assigned pair indices
   * @return pointer to pair index vector
   */
  virtual vector<StrCartesianIndexPair>* getPairs();

  /**
   * Access squared distances between index pairs
   * @return point to pair distance vector
   */
  virtual vector<double>* getDistancesSqr();

  /**
   * Access internal container of non-assigned point indices
   * @return pointer to non-pair index vector
   */
  virtual vector<unsigned int>* getNonPairs();

  /**
   * Dimension of space
   * @return dimension of space
   */
  virtual int getDimension();

  /**
   * Set model as matching base
   * @param ppdModel array of xy values
   * @param nSize number of points
   **/
  virtual void setModel(double** ppdModel, int nSize) = 0;

  /**
   * Determine point pairs (nearest neighbors)
   * @param ppdScene scene to be compared
   * @param nSize nr of points in scene
   */
  void determinePairs(double** ppdScene, int nSize);
  virtual void determinePairs(double** ppdScene, bool* mask, int nSize) = 0;

  /**
   * clear vector of Cartesian point pairs and reset post assignment filters
   */
  void reset();

protected:
  /**
   * add assigned point to internal vector
   * @param unIndexModel index of model point
   * @param unIndexScene index of scene point
   * @param dDistanceSqr squared distance between model and scene point
   */
  virtual void addPair(unsigned int unIndexModel, unsigned int unIndexScene, double dDistanceSqr);

  /**
   * add non-assigned point to internal vector
   * @param unIndexScene index of scene point
   */
  virtual void addNonPair(unsigned int unIndexScene);

  /**
   * Dimension of space
   */
  int _nDimension;

  /**
   * the 2D or 3D model
   */
  double** _ppdModel;

  /**
   * model size / number of points */
  int _nSizeModel;


  vector<IPreAssignmentFilter*> _vPrefilter;
  vector<IPostAssignmentFilter*> _vPostfilter;

  void clearPairs();

private:
  /**
   * Vector of Cartesian pairs
   */
  vector<StrCartesianIndexPair> _initPairs;
  vector<StrCartesianIndexPair> _filteredPairs;
  vector<StrCartesianIndexPair>* _pairs;

  /**
   * Vector of squared distances of pairs
   */
  vector<double> _initDistancesSqr;
  vector<double> _filteredDistancesSqr;
  vector<double>* _distancesSqr;

  /**
   * Vector of Cartesian points (scene points, that could not be assigned to model points)
   */
  vector<unsigned int> _nonPairs;

};

}

#endif
