#include <obcore/scripting/LuaScriptManager.h>
#include <obcore/base/Logger.h>

namespace obvious
{

LuaScriptManager::LuaScriptManager()
{
  _L = luaL_newstate();
  luaL_openlibs(_L);
}

LuaScriptManager::~LuaScriptManager()
{
  lua_close(_L);
}

LuaTable* LuaScriptManager::readTable(const char* filepath, const char* tablename)
{
  if(luaL_loadfile(_L, filepath) || lua_pcall(_L, 0, 0, 0))
  {
    luaL_error(_L, "Cannot read table file: %s", lua_tostring(_L, -1));
    return NULL;
  }
  lua_getglobal(_L, tablename);
  if(!lua_istable(_L, -1))
  {
    luaL_error(_L, "not a valid table");
    return NULL;
  }

  LuaTable* table = new LuaTable(_L);

  return table;
}

LuaMethod* LuaScriptManager::readMethod(const char* filepath, const char* methodname)
{
  if(luaL_loadfile(_L, filepath) || lua_pcall(_L, 0, 0, 0))
  {
    LOGMSG(DBG_ERROR, "Error calling LUA script");
    return NULL;
  }
  else
  {
    return new LuaMethod(_L, methodname);
  }
}

} /* namespace obvious */
