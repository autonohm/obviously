#include <iostream>
#include <unistd.h>
#include <cmath>

#include "obcore/scripting/LuaScriptManager.h"

using namespace std;
using namespace obvious;

/**
 * Callback function (called from Lua script)
 */
static int l_sin (lua_State *L)
{
  double d = luaL_checknumber(L, 1);
  lua_pushnumber(L, sin(d));
  return 1;  /* number of results */
}

/**
 * Example showing usage of LuaScriptManager: Calling Lua functions / Callback to C
 * @author Stefan May
 * @date 16.6.2015
 */
int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " config.lua" << endl;
    return -1;
  }

  LuaScriptManager manager(argv[1]);

  // Register name of above implemented callback in the Lua script
  char callbackName[] = "mysin";
  manager.registerCallback(l_sin, callbackName);

  while(1)
  {
    double x = M_PI/3.0;
    double z;

    manager.callFunction("aLuaFunctionWithCallback", "d>d", x, &z);
    cout << "Called with parameters x=" << x << endl;
    cout << " reveived value z=" << z << endl;

    // slow down loop
    usleep(500000);

    // reloading allows to modify Lua function during runtime
    manager.reload();
  }

  return 0;
}
