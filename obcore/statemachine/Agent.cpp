#include "Agent.h"
#include <string.h>
#include <iostream>

#include "obcore/base/Logger.h"

namespace obvious
{

unsigned int Agent::_AgentID = 0;

std::map<const unsigned int, Agent*> Agent::_agents;

Agent::Agent()
{
  _ID = _AgentID++;
  Agent::_agents[_ID] = this;
  _currentState = NULL;
  _nextState = NULL;
}

Agent::~Agent()
{
  if(_currentState)
  {
    _currentState->onExit();
    if(_volatile) delete _currentState;
  }
  Agent::_agents[_ID] = NULL;
}

Agent* Agent::getAgentByID(const unsigned int id)
{
  return Agent::_agents[id];
}

unsigned int Agent::getID()
{
  return _ID;
}

void Agent::awake()
{
  // check 1st time awake
  if(!_currentState && _nextState)
  {
    _currentState = _nextState;
    _currentState->onEntry();
    _volatile = _volatileNext;
    _nextState = NULL;
  }

  if(_currentState)
  {
    _currentState->onActive();

    if(_nextState) // do transition
    {
      // do de-initialization of current state
      _currentState->onExit();
      if(_volatile) delete _currentState;
    }
  }

  if(_nextState) // do transition
  {
    // do transition to next state
    _currentState = _nextState;
    _volatile = _volatileNext;
    _nextState = NULL;

    _currentState->onEntry();
  }
}

void Agent::registerPersistantState(const int stateID, StateBase* state)
{
  if(state->getAgent()==NULL)
  {
    _persistantStateMap[stateID] = state;
    state->setAgent(this);
    state->onSetup();
  }
  else
  {
    LOGMSG(DBG_ERROR, "State already registered at another Agent.");
  }
}

StateBase* Agent::getPersistantState(const int stateID)
{
  return _persistantStateMap[stateID];
}

void Agent::transitionToPersistantState(const int stateID)
{
  StateBase* nextState = _persistantStateMap[stateID];
  if(_currentState != nextState)
  {
    _nextState = nextState;
    _volatileNext = false;
  }
}

void Agent::transitionToVolatileState(StateBase* nextState)
{
  if(_currentState != nextState)
  {
    if(nextState!=NULL && nextState->getAgent()==NULL)
    {
      _nextState = nextState;
      _nextState->setAgent(this);
      _volatileNext = true;
      _nextState->onSetup();
    }
    else
    {
      LOGMSG(DBG_ERROR, "Next state instance invalid. Either NULL or already assigned state passed.");
    }
  }
}

void Agent::deletePersistantStates()
{
  for(std::map<const int, StateBase*>::iterator it=_persistantStateMap.begin(); it!=_persistantStateMap.end(); ++it)
  {
    delete it->second;
  }
  _persistantStateMap.clear();
}

} // end namespace

