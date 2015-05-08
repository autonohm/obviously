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

void LuaScriptManager::execute(const char* filepath, const char* methodname)
{
  if(luaL_loadfile(_L, filepath) || lua_pcall(_L, 0, 0, 0))
  {
    LOGMSG(DBG_ERROR, "Error calling LUA script");
  }
  else
  {
    lua_getglobal(_L, methodname);
    //lua_pushnumber(_L, height);
    //lua_pushnumber(_L, surface);
    //lua_pushnumber(_L, volume);

    if (!lua_pcall(_L, 3, 1, 0))
    {
      //if(lua_isnumber(_L,-1))
      //  weight = (double)lua_tonumber(_L, -1);
      //else
      //  std::cout << "WeightEstimator::estimate: Return value is not a number." << std::endl;
    }
    else
    {
      LOGMSG(DBG_ERROR, "Number of arguments or return values are not valid.");
    }
  }
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

} /* namespace obvious */
