#ifndef STATEBASE_H_
#define STATEBASE_H_

#include <iostream>

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
  StateBase(bool autoCleanup=true);

  /**
   * Default destructor
   */
  virtual ~StateBase(){};

  /**
   * Set agent (needs to be set before call of process method)
   * @param agent agent instance
   */
  void setAgent(Agent* agent);

  /**
   * Called once when activated
   */
  virtual void doEntry() {};

  /**
   * Called while active
   */
  virtual StateBase* doActive() = 0;

  /**
   * Called once when left
   */
  virtual void doExit() { };

  /**
   * Called once when left
   */
  void doCleanup() { if(_autoCleanup) delete this; };

protected:

  Agent* _agent;

  bool _autoCleanup;

};

} /* namespace obvious */

#endif /* STATEBASE_H_ */
