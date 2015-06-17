#ifndef AGENT_H_
#define AGENT_H_

#include "obcore/statemachine/states/StateBase.h"
#include "obcore/base/Point.h"
#include "obcore/base/Timer.h"

#include <vector>
/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   Agent
 * @brief   Generic class for statemachine agents
 * @author  Stefan May
 * @date    23.10.2014
 */
class Agent
{

public:

  /**
   * Constructor
   * @param initState Initial state
   */
  Agent(StateBase* initState);

  /**
   * Destructor
   */
  virtual ~Agent();

  /**
   * Awake agent to do his job
   */
  void awake();

  /**
   * Get unique ID of agent, each instance is assigned a sequential ID
   * @return ID
   */
  unsigned int getID();

private:

  unsigned int _ID;

  StateBase* _currentState;

  static unsigned int _AgentID;

  bool _initialized;
};

} // end namespace

#endif
