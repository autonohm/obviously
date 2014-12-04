#include "StateMachine.h"
#include <cstddef>

namespace obvious
{

static StateMachine* _machine;

StateMachine::StateMachine()
{
  _currentState = NULL;
}

StateMachine::StateMachine(StateMachine& c)
{
  _currentState = c.getState();
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

void StateMachine::process(void)
{
  if(_currentState) _currentState->process();
}

} /* namespace obvious */
