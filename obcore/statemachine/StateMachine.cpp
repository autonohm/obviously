#include "StateMachine.h"
#include <cstddef>

namespace obvious
{

static StateMachine* _machine;

StateMachine::StateMachine(Agent* agent)
{
  _currentState = NULL;
  _agent = agent;
}

StateMachine::StateMachine(StateMachine& c)
{
  _currentState = c.getState();
  _agent = c.getAgent();
}

StateMachine::~StateMachine()
{
  if(_machine) delete _machine;
}

void StateMachine::setState(StateBase* state)
{
  _currentState = state;
}

StateBase* StateMachine::getState(void) const
{
   return _currentState;
}

Agent* StateMachine::getAgent() const
{
  return _agent;
}

void StateMachine::process(void)
{
  if(_currentState) _currentState->process();
}

} /* namespace obvious */
