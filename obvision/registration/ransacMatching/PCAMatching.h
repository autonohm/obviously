#ifndef PCAMATCHING_H_
#define PCAMATCHING_H_

#include <flann/flann.hpp>
#include "obcore/math/linalg/linalg.h"
#include "obvision/registration/Trace.h"
#include "obvision/registration/icp/PointToLineEstimator2D.h"
#include "omp.h"

namespace obvious
{

/**
 * @class PCAMatching
 * @brief Matching algorithm with PCA alignment and RANSAC scheme
 * @author Stefan May
 **/
class PCAMatching
{
public:
  /**
   * Constructor
   * @param trials number of trials / matching guesses
   * @param epsThresh threshold for rating good matches
   * @param phiMax maximum rotation
   * @param sizeControlSet approximate set of control set
   */
  PCAMatching(unsigned int trials = 50, double epsThresh = 0.15, unsigned int sizeControlSet = 180);

  /**
   * Destructor
   */
  virtual ~PCAMatching();

  /**
   * Activate internal trace writing. While the method iterate is executed, all states are traced.
   */
  void activateTrace();

  /**
   * Deactivate internal trace writing.
   */
  void deactivateTrace();

  /**
   * Matching method
   * @param M Matrix for model points. Points are accessed by rows. e.g. x = M(p, 0) y= M(p,1)
   * @param maskM Mask for matrix M. Valid points are identified with true-value. It has the same size as M.getCols()
   * @param S Matrix for scene points
   * @param maskS Mask for matrix S.
   * @param phiMax Maximum allowed rotation as output
   * @param transMax Maximum allowed translation
   * @param resolution Angular resolution of the laser scan
   * @return 3x3 registration matrix
   */
  obvious::Matrix match(const obvious::Matrix* M, const bool* maskM, const obvious::Matrix* S, const bool* maskS, double phiMax = M_PI / 4.0, const double transMax = 1.5, const double resolution = 0.0);

  /**
   * Serialize assignment to trace folder
   * @param folder trace folder (must not be existent)
   */
  void serializeTrace(const char* folder);

private:

  // extract valid sample indices from matrix giving a validity mask
  vector<unsigned int> extractSamples(const obvious::Matrix* M, const bool* mask);

  // init kd-tree for fast NN search in model
  void initKDTree(const obvious::Matrix* M, vector<unsigned int> valid);

  // pick control set for RANSAC in-/outlier detection
  obvious::Matrix* pickControlSet(const obvious::Matrix* M, vector<unsigned int> idxValid, vector<unsigned int> &idxControl);

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

  // Trace module
  Trace* _trace;

  // Number of samples used for PCA in local neighborhood
  int _pcaCnt;
};

}

#endif /* PCAMATCHING_H_ */
