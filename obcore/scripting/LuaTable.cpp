#include <obcore/scripting/LuaTable.h>

namespace obvious
{

LuaTable::LuaTable(lua_State* L)
{
  _L = L;
}

LuaTable::~LuaTable()
{

}

bool LuaTable::getBool(const char* key)
{
  bool result;
  lua_pushstring(_L, key);
  lua_gettable(_L, -2);
  if(!lua_isboolean(_L, -1))
    luaL_error(_L, "invalid component in table: %s", lua_tostring(_L, -1));

  result = lua_toboolean(_L, -1);
  lua_pop(_L, 1);
  return result;
}

double LuaTable::getDouble(const char* key)
{
  double result;
  lua_pushstring(_L, key);
  lua_gettable(_L, -2);
  if(!lua_isnumber(_L, -1))
    luaL_error(_L, "invalid component in table: %s", lua_tostring(_L, -1));

  result = lua_tonumber(_L, -1);
  lua_pop(_L, 1);
  return result;
}

} /* namespace obvious */
