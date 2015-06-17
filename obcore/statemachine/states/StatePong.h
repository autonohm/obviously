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
  StatePong();

  /**
   * Destructor
   */
  virtual ~StatePong();

  /**
   * Called once when activated
   */
  void doEntry();

  /**
   * Process method (step-wise, never block this method)
   */
  StateBase* doActive();

  /**
   * Called once when left
   */
  void doExit();

private:

};

} /* namespace obvious */

#endif /* STATEPONG_H_ */
