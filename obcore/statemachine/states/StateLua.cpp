#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/AgentModel.h"
#include "StateLua.h"

namespace obvious
{

StateLua::StateLua(AgentModel* model, const char* filepath) : StateBaseModel(model)
{
  _manager = new LuaScriptManager(filepath);
}

StateLua::~StateLua()
{
  delete _manager;
}

void StateLua::onEntry()
{
  _manager->callFunction("onEntry", "");
}

void StateLua::onActive()
{
  _manager->reload();
  int id = 0;
  _manager->callFunction("onActive", ">i", &id);
  if(id)
    _agent->transitionToPersistantState(id);
}

void StateLua::onExit()
{
  _manager->callFunction("onExit", "");
}

} /* namespace obvious */
