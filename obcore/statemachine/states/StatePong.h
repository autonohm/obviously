#ifndef STATEPONG_H_
#define STATEPONG_H_

#include "obcore/statemachine/states/StateBase.h"

namespace obvious
{

/**
 * @class StatePong
 * @brief Example state, transition to ping state
 * @author Stefan May
 */
class StatePong: public StateBase
{
public:

  /**
   * Constructor
   */
  StatePong(Agent* agent);

  /**
   * Destructor
   */
  virtual ~StatePong(void);

  /**
   * Process method (step-wise, never block this method)
   */
  void process(void);

private:

};

} /* namespace obvious */

#endif /* STATEPONG_H_ */
