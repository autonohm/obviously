#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "obcore/statemachine/states/StateBase.h"
#include "obcore/statemachine/Agent.h"

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class   StateMachine
 * @author  Stefan May
 * @date    23.10.2014
 * @brief   StateMachine to manage state machine. Designed as singleton pattern.
 */
class StateMachine
{
public:

  /**
   * Constructor
   */
  StateMachine(Agent* agent);

  /**
   * Copy constructor
   * @param machine state machine
   */
  StateMachine(StateMachine &machine);

  /**
   * Default destructor
   */
  virtual ~StateMachine();

  /**
   * Function to set state
   * @param state
   */
  void setState(StateBase* state);

  /**
   * Function to get current state
   * @return
   */
  StateBase* getState() const;

  /**
   * Access agent
   * @return model
   */
  Agent* getAgent() const;

  /**
   * Function for processing
   */
  void process(void);

private:

  StateBase* _currentState;

  Agent* _agent;

};

} /* namespace obvious */

#endif /* STATEMACHINE_H_ */
