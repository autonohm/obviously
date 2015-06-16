#include <iostream>
#include <unistd.h>
#include "obcore/scripting/LuaScriptManager.h"

using namespace std;
using namespace obvious;

/**
 * Example showing usage of LuaScriptManager: Reading Lua tables
 * @author Stefan May
 * @date 16.6.2015
 */
int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " lua/config.lua" << endl;
    return -1;
  }

  LuaScriptManager manager(argv[1]);

  double value1, value2;
  int value3;
  bool value4;
  const char * value5;

  vector<string> keys;
  while(1)
  {
    keys.clear();
    keys.push_back("value1");
    keys.push_back("value2");
    if(manager.readTable("group1", keys, "dd", &value1, &value2))
    {
      cout << "--- Table 1 ---" << endl;
      cout << "Value read: " << value1 << endl;
      cout << "Value read: " << value2 << endl;
    }
    else
    {
      cout << "Reading Table 1 failed" << endl;
    }

    keys.clear();
    keys.push_back("value3");
    keys.push_back("value4");
    keys.push_back("value5");
    if(manager.readTable("group2", keys, "ibs", &value3, &value4, &value5))
    {
      cout << "--- Table 2 ---" << endl;
      cout << "Value read: " << value3 << endl;
      cout << "Value read: " << value4 << endl;
      cout << "Value read: " << value5 << endl;
    }
    else
    {
      cout << "Reading Table 2 failed" << endl;
    }

    // slow down loop
    usleep(500000);

    // reloading allows to modify table during runtime
    manager.reload();
  }

  return 0;
}
