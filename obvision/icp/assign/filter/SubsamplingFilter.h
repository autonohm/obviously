#ifndef SUBSAMPLINGFILTER_H
#define SUBSAMPLINGFILTER_H

#include "obvision/icp/assign/filter/IPreAssignmentFilter.h"
#include <vector>

namespace obvious
{

/**
 * @class SubsamplingFilter
 * @brief
 * @author Stefan May
 */
class SubsamplingFilter : public IPreAssignmentFilter
{
public:
  /**
   * Constructor
   */
  SubsamplingFilter(unsigned int step);

  /**
   * Destructor
   */
  ~SubsamplingFilter();

  virtual void filter(double** scene, unsigned int size, bool* mask);

private:
  unsigned int _step;
};

}

#endif /*SUBSAMPLINGFILTER_H*/
