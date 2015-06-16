#include <iostream>
#include <unistd.h>
#include "obcore/scripting/LuaScriptManager.h"

using namespace std;
using namespace obvious;

/**
 * Example showing usage of LuaScriptManager: Calling Lua functions
 * @author Stefan May
 * @date 16.6.2015
 */
int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " .../lua/functionWithCallback.lua" << endl;
    return -1;
  }

  LuaScriptManager manager(argv[1]);

  while(1)
  {
    double x = 3.0;
    double y = 7.0;
    double z;

    // define input as: double, double
    // define output (>) as: double
    manager.callFunction("aLuaFunction", "dd>d", x, y, &z);
    cout << "Called with parameters x=" << x << ", y=" << y << endl;
    cout << " returned value z=" << z << endl;

    // slow down loop
    usleep(500000);

    // reloading allows to modify Lua function during runtime
    manager.reload();
  }

  return 0;
}
