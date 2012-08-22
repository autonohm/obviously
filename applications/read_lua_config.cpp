#include <iostream>
#include "lua.hpp"

using namespace std;

double getNumber(lua_State* L, const char* key)
{
  double result;
  lua_pushstring(L, key);
  lua_gettable(L, -2);
  if(!lua_isnumber(L, -1))
    luaL_error(L, "invalid component in table: %s", lua_tostring(L, -1));

  result = lua_tonumber(L, -1);
  lua_pop(L, 1);
  return result;
}


int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " config.lua" << endl;
    return -1;
  }

  lua_State *L = luaL_newstate();
  luaL_openlibs(L);

  if(luaL_loadfile(L, argv[1]) || lua_pcall(L, 0, 0, 0))
    luaL_error(L, "Cannot run configuration file: %s", lua_tostring(L, -1));

  lua_getglobal(L, "group");
  if(!lua_istable(L, -1))
    luaL_error(L, "`group` is not a table");

  double val = getNumber(L, "value");

  cout << "Value read: " << val << endl;

  lua_close(L);

  return 0;
}
