#ifndef STATEPING_H_
#define STATEPING_H_

#include "obcore/statemachine/states/StateBase.h"

namespace obvious {

/**
 * @class StatePing
 * @brief Example state, transition to pong state
 * @author Stefan May
 */
class StatePing : public StateBase
{
public:

  /**
   * Constructor
   */
  StatePing(Agent* agent);

  /**
   * Destructor
   */
  virtual ~StatePing(void);

  /**
   * Process method (step-wise, never block this method)
   */
  void process(void);

private:

};

} /* namespace obvious */

#endif /* STATEPING_H_ */
