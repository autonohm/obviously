#ifndef STATEBASE_H_
#define STATEBASE_H_

#include <iostream>
#include <map>

/**
 * @namespace  obvious
 */
namespace obvious
{

class Agent;

/**
 * @class   StateBase
 * @author  Stefan May
 * @date    23.10.2014
 */
class StateBase
{

public:

  /**
   * Constructor
   */
  StateBase();

  /**
   * Default destructor
   */
  virtual ~StateBase();

  /**
   * Set agent of state. This is called by the agent at state transitions.
   * @param agent Agent instance
   */
  void setAgent(Agent* agent);

  /**
   * Called once when activated
   */
  virtual void onEntry() {};

  /**
   * Called while active
   */
  virtual void onActive() = 0;

  /**
   * Called once when left
   */
  virtual void onExit() { };

protected:

  Agent* _agent;

};

} /* namespace obvious */

#endif /* STATEBASE_H_ */
