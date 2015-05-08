#include <iostream>
#include <unistd.h>
#include "obcore/scripting/LuaScriptManager.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " config.lua" << endl;
    return -1;
  }

  LuaScriptManager manager;

  while(1)
  {
    LuaTable* table = manager.readTable(argv[1], "group1");
    cout << "--- Table 1 ---" << endl;
    cout << "Value read: " << table->getDouble("value1") << endl;
    cout << "Value read: " << table->getDouble("value2") << endl;
    delete table;

    table = manager.readTable(argv[1], "group2");
    cout << "--- Table 2 ---" << endl;
    cout << "Value read: " << table->getDouble("value1") << endl;
    cout << "Value read: " << table->getBool("value2") << endl;
    delete table;

    usleep(500000);
  }

  return 0;
}
