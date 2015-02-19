#ifndef RANSACMATCHING_H_
#define RANSACMATCHING_H_

#include <flann/flann.hpp>
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class RansacMatching
 * @brief Matching algorithm with RANSAC scheme
 * @author Stefan May
 **/
class RansacMatching
{
public:
  /**
   * Constructor
   * @param trials number of trials / matching guesses
   * @param epsThresh threshold for rating good matches
   * @param phiMax maximum rotation
   * @param sizeControlSet approximate set of control set
   */
  RansacMatching(unsigned int trials = 50, double epsThresh = 0.15, unsigned int sizeControlSet = 180);

  /**
   * Destructor
   */
  virtual ~RansacMatching();

  /**
   * Matching method
   * @param M model
   * @param S scene
   * @return 3x3 registration matrix
   */
  obvious::Matrix match(const obvious::Matrix* M, const bool* maskM, const obvious::Matrix* S, const bool* maskS, double phiMax = M_PI / 4.0, double resolution = 0.0);

private:

  // extract valid indices from matrix giving a validity mask
  vector<unsigned int> extractValidIndices(const obvious::Matrix* M, const bool* mask);

  // init kd-tree for fast NN search in model
  void initKDTree(const obvious::Matrix* M, vector<unsigned int> valid);

  // pick control set for RANSAC in-/outlier detection
  obvious::Matrix* pickControlSet(const obvious::Matrix* M, vector<unsigned int> idxValid, vector<unsigned int> &idxControl);

  // create look-up table for point to point intra-distance of a point set
  double** createLutIntraDistance(const obvious::Matrix* M, const bool* mask, int maxDist);

  // opening angle of laser scanner (absolute value)
  double _fov;

  // number of measurements included in a single scan
  int _samples;

  // squared distance threshold
  double _epsSqr;

  // number of trials
  unsigned int _trials;

  // approximate control set
  unsigned int _sizeControlSet;

  // tree for accelerating NN search
  flann::Index<flann::L2<double> >* _index;
  flann::Matrix<double>* _model;

};

}

#endif /* RANSACMATCHING_H_ */
