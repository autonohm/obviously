#ifndef STATEPONG_H_
#define STATEPONG_H_

#include <obcore/statemachine/states/StateBaseModel.h>

namespace obvious
{

/**
 * @class StatePong
 * @brief Example state, transition to ping state
 * @author Stefan May
 */
class StatePong: public StateBaseModel
{
public:

  /**
   * Constructor
   */
  StatePong(AgentModel* model);

  /**
   * Destructor
   */
  virtual ~StatePong();

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

#endif /* STATEPONG_H_ */
