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
  RansacMatching(unsigned int trials = 50, double epsThresh = 0.03, unsigned int sizeControlSet = 180, bool clipPeripheralArea = false);

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
  obvious::Matrix match(obvious::Matrix* M, obvious::Matrix* S, double phiMax = M_PI / 4.0);

private:

  // distance threshold
  double _epsDist;

  // number of trials
  unsigned int _trials;

  // approximate control set
  unsigned int _sizeControlSet;

  // clip peripheral area of laser (non-overlapping area)
  bool _clipPeripheralArea;

  // tree for accelerating NN search
  flann::Index<flann::L2<double> >* _index;
  flann::Matrix<double>* _model;

};

}

#endif /* RANSACMATCHING_H_ */
