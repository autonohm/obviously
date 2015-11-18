#ifndef _RANDOMMATCHING_H_
#define _RANDOMMATCHING_H_

#include <vector>
#include "obcore/math/linalg/linalg.h"
#include "obvision/registration/Trace.h"

namespace obvious
{

class RandomMatching
{
public:
  RandomMatching(unsigned int sizeControlSet);

  virtual ~RandomMatching();

  void activateTrace();

  void deactivateTrace();

  void serializeTrace(const char* folder);

protected:

  // extract valid indices from matrix giving a validity mask
  std::vector<unsigned int> extractSamples(const obvious::Matrix* M, const bool* mask, unsigned int searchRange);

  // pick control set for RANSAC in-/outlier detection
  obvious::Matrix* pickControlSet(const obvious::Matrix* M, std::vector<unsigned int> idxValid, std::vector<unsigned int> &idxControl);

  unsigned int _sizeControlSet;

  // Trace module
  Trace* _trace;

};

} /* namespace obvious */

#endif /* _RANDOMMATCHING_H_ */
