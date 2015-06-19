#ifndef STATEPING_H_
#define STATEPING_H_

#include <obcore/statemachine/states/StateBaseModel.h>

namespace obvious {

/**
 * @class StatePing
 * @brief Example state, transition to pong state
 * @author Stefan May
 */
class StatePing : public StateBaseModel
{
public:

  /**
   * Constructor
   */
  StatePing(AgentModel* model);

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
  void onActive();

  /**
   * Called once when left
   */
  void onExit();

private:

};

} /* namespace obvious */

#endif /* STATEPING_H_ */
