#ifndef STATEPING_H_
#define STATEPING_H_

#include "obcore/statemachine/IState.h"

namespace obvious {

/**
 * @class StatePing
 * @brief Example state, transition to pong state
 * @author Stefan May
 */
class StatePing: public IState
{
public:

  /**
   * Constructor
   */
  StatePing(void);

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
