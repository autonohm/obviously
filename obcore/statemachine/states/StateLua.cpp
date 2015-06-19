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
  _manager->callFunction("doEntry", "");
}

void StateLua::onActive()
{
  _manager->reload();
  int id = 0;
  _manager->callFunction("doActive", ">i", &id);
  if(id)
    _agent->transitionToPersistantState(id);
}

void StateLua::onExit()
{
  _manager->callFunction("doExit", "");
}

} /* namespace obvious */
