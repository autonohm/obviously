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
  StateBase(bool persistant=false);

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
  virtual void onEntry() {};

  /**
   * Called while active
   */
  virtual StateBase* onActive() = 0;

  /**
   * Called once when left
   */
  virtual void onExit() { };

  /**
   * Called once when left
   */
  void onCleanup() { if(!_persistant) delete this; };

protected:

  Agent* _agent;

private:

  bool _persistant;

};

} /* namespace obvious */

#endif /* STATEBASE_H_ */
