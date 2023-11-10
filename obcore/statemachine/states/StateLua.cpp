#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/AgentModel.h"
#include "obcore/base/Logger.h"
#include "StateLua.h"

namespace obvious
{

/**
 * Callback function for Lua script
 */
static int luaTransistionToPersistantState(lua_State* L)
{
  unsigned int agentID = luaL_checknumber(L, 1);
  unsigned int stateID = luaL_checknumber(L, 2);

  Agent* agent = Agent::getAgentByID(agentID);
  if(agent)
    agent->transitionToPersistantState(stateID);
  else
    LOGMSG(DBG_ERROR, "Agent ID not valid");

  return 0;
}

StateLua::StateLua(AgentModel* model, const char* filepath) : StateBaseModel(model)
{
  _manager = new LuaScriptManager(filepath);
  _manager->registerCallback(luaTransistionToPersistantState, "transitionToPersistantState");
  _autoReload = true;
}

StateLua::~StateLua()
{
  delete _manager;
}

void StateLua::setAutoReload(bool autoReload)
{
  _autoReload = autoReload;
}

bool StateLua::getAutoReload()
{
  return _autoReload;
}

void StateLua::onSetup()
{
  unsigned int agentID = _agent->getID();
  _manager->callFunction("onLoad", "i", agentID);
  _loadInThisCycle = false;
}

void StateLua::onEntry()
{
  if(_loadInThisCycle) reload();

  _manager->callFunction("onEntry", "");
}

void StateLua::onActive()
{
  if(_loadInThisCycle) reload();

  int retval = 0;
  _manager->callFunction("onActive", ">i", &retval);

  _loadInThisCycle = _autoReload;
}

void StateLua::onExit()
{
  _manager->callFunction("onExit", "");
}

void StateLua::reload()
{
  _manager->reload();
  unsigned int agentID = _agent->getID();
  _manager->callFunction("onLoad", "i", agentID);
  _loadInThisCycle = false;
}

} /* namespace obvious */
