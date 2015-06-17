#include "Agent.h"
#include <string.h>
#include <iostream>

namespace obvious
{

unsigned int Agent::_AgentID = 0;

Agent::Agent(StateBase* initState)
{
  _ID = _AgentID++;
  initState->setAgent(this);
  _currentState = initState;
  _initialized = false;
}

Agent::~Agent()
{
  if(_currentState) delete _currentState;
}

void Agent::awake()
{
  if(!_initialized)
  {
    // doEntry for the initial state cannot be called in constructor, since constructor of child class has not been passed at that time.
    // Variables set in constructor are not initialized.
    _currentState->doEntry();
    _initialized = true;
  }

  StateBase* nextState = _currentState->doActive();

  if(nextState)
  {
    _currentState->doExit();
    _currentState->doCleanup();
    _currentState = nextState;
    _currentState->doEntry();
  }
}

} // end namespace

