/*
 * StateMachine.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include "StateMachine.h"
#include <cstddef>

namespace obvious
{

static StateMachine* _machine;

StateMachine::StateMachine(void)
{
  _currentState = NULL;
}

StateMachine::StateMachine(StateMachine& c)
{
  _currentState = NULL;
}

StateMachine* StateMachine::getInstance(void)
{
  if(_machine==NULL) _machine = new StateMachine();

  return _machine;
}

StateMachine::~StateMachine(void)
{
  if(_machine) delete _machine;
}

IState* StateMachine::getState(void) const
{
   return _currentState;
}

void StateMachine::setState(IState* state)
{
  _currentState = state;
}

void StateMachine::process(void)
{
  if(_currentState) _currentState->process();
}

} /* namespace obvious */
