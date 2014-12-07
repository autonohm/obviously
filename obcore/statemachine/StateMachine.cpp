#include "StateMachine.h"

#include <cstddef>
#include <unistd.h>

#include "obcore/base/Logger.h"

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
  else
  {
    LOGMSG(DBG_ERROR, "No state selected");
    usleep(1000000);
  }
}

} /* namespace obvious */
