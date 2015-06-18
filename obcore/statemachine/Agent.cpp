#include "Agent.h"
#include <string.h>
#include <iostream>

namespace obvious
{

unsigned int Agent::_AgentID = 0;

Agent::Agent()
{
  _ID = _AgentID++;
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
}

void Agent::awake()
{
  if(_currentState)
  {
    _currentState->onActive();
  }

  if(_nextState)
  {
    if(_currentState)
    {
      _currentState->onExit();
      if(_volatile) delete _currentState;
    }
    _currentState = _nextState;
    _currentState->onEntry();
    _nextState = NULL;
  }
}

void Agent::registerPersistantState(const int stateID, StateBase* state)
{
  _persistantStateMap[stateID] = state;
}

StateBase* Agent::getPersistantState(const int stateID)
{
  return _persistantStateMap[stateID];
}

void Agent::transitionToPersistantState(const int stateID)
{
  _nextState = _persistantStateMap[stateID];
  if(_nextState) _nextState->setAgent(this);
  _volatile = false;
}

void Agent::transitionToVolatileState(StateBase* nextState)
{
  if(_currentState != nextState)
  {
    _nextState = nextState;
    if(_nextState) _nextState->setAgent(this);
    _volatile = true;
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

unsigned int Agent::getID()
{
  return _ID;
}

} // end namespace

