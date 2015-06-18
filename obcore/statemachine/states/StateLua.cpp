#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StateLua.h"

namespace obvious
{

StateLua::StateLua(const char* filepath) : StateBase(true)
{
  _manager = new LuaScriptManager(filepath);
  _nextState = NULL;
}

StateLua::~StateLua()
{
  delete _manager;
}

void StateLua::setNextState(StateBase* state)
{
  _nextState = state;
}

void StateLua::onEntry()
{
  _manager->callFunction("doEntry", "");
}

StateBase* StateLua::onActive()
{
  int retVal;

  _manager->reload();
  _manager->callFunction("doActive", ">i", &retVal);

  if(retVal)
    return _nextState;

  return NULL;
}

void StateLua::onExit()
{
  _manager->callFunction("doExit", "");
}

} /* namespace obvious */
