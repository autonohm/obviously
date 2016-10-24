#ifndef RANSACPRIMITIVES_H_
#define RANSACPRIMITIVES_H_

#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class RansacPrimitives
 * @brief Ransac parameter estimator for geometric primitives
 * @author Stefan May
 * @date 22.10.2015
 */
class RansacPrimitives
{
public:
  /**
   * Constructor
   */
  RansacPrimitives(unsigned int trials);

  /**
   * Destructor
   */
  ~RansacPrimitives();

  /**
   * Estimate parameters for circular primitives
   * @param D data matrix of size nx2, i.e. n tuples of xy-coordinates
   * @param params output vector of found circular parameters (centroid x,y and radius)
   * @param mask assignment mask (true=point belongs to circular structure)
   */
  bool findCircle(obvious::Matrix* D, double params[3], bool* mask);

private:

  unsigned int _trials;
};

}// namespace

#endif
