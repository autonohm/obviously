#ifndef AGENT_H_
#define AGENT_H_

#include "obcore/statemachine/states/StateBase.h"

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
   */
  Agent();

  /**
   * Destructor
   */
  virtual ~Agent();

  /**
   * Get Agent instance by passing unique ID. The Agent must be instantiated before, otherwise NULL is returned.
   * @param id unique agent ID (assigned when Agent is instantiated).
   * @return agent instance
   */
  static Agent* getAgentByID(const unsigned int id);

  /**
   * Get unique ID of agent, each instance is assigned a sequential ID
   * @return ID
   */
  unsigned int getID();

  /**
   * Awake agent to do his job
   */
  void awake();

  /**
   * Register persistant state with give ID
   * @param stateID state ID
   * @param state state instance
   */
  void registerPersistantState(const int stateID, StateBase* state);

  /**
   * Get state by ID
   * @param stateID
   * @return state instance
   */
  StateBase* getPersistantState(const int stateID);

  /**
   * Activate previously registered state by ID
   * @param stateID state ID
   */
  void transitionToPersistantState(const int stateID);

  /**
   * Activate new state instance. Instance is deleted with next transition.
   * @param nextState
   */
  void transitionToVolatileState(StateBase* nextState);

  /**
   * Clean up persistant state map
   */
  void deletePersistantStates();

private:

  unsigned int _ID;

  StateBase* _currentState;

  StateBase* _nextState;

  static unsigned int _AgentID;

  bool _volatile;

  std::map<const int, StateBase*> _persistantStateMap;

  static std::map<const unsigned int, Agent*> _agents;
};

} // end namespace

#endif
