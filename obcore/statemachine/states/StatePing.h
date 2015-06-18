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
  StatePing();

  /**
   * Destructor
   */
  virtual ~StatePing();

  /**
   * Called once when activated
   */
  void onEntry();

  /**
   * Process method (step-wise, never block this method)
   */
  StateBase* onActive();

  /**
   * Called once when left
   */
  void onExit();

private:

};

} /* namespace obvious */

#endif /* STATEPING_H_ */
