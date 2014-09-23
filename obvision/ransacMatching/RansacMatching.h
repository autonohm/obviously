/*
 * RansacMatching.h
 *
 *  Created on: 23.09.2014
 *      Author: mayst
 */

#ifndef RANSACMATCHING_H_
#define RANSACMATCHING_H_

#include <flann/flann.hpp>
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

class RansacMatching
{
public:
  RansacMatching();
  virtual ~RansacMatching();

  obvious::Matrix match(obvious::Matrix* M, obvious::Matrix* S);

private:

  // distance threshold
  double _epsDist;

  // rotational threshold
  double _phiMax;

  // number of trials
  unsigned int _trials;

  flann::Matrix<double>* _model;
  flann::Index<flann::L2<double> >* _index;

};

}

#endif /* RANSACMATCHING_H_ */
