#include <iostream>
#include <unistd.h>

#include "obcore/statemachine/Agent.h"
#include "obcore/statemachine/states/StateLua.h"

using namespace std;
using namespace obvious;

/**
 * Example showing usage of LuaScriptManager: State machine
 * @author Stefan May
 * @date 17.6.2015
 */
int main(int argc, char* argv[])
{
  if(argc<3)
  {
    cout << "usage: " << argv[0] << " .../lua/state1.lua .../lua/state2.lua ..." << endl;
    return -1;
  }

  Agent agent;
  for(int i=1; i<argc; i++)
  {
    StateLua* state = new StateLua(argv[i]);
    agent.registerPersistantState(i, state);
  }
  agent.transitionToPersistantState(1);

  while(1)
  {
    agent.awake();
    // slow down loop
    usleep(500000);
  }

  agent.deletePersistantStates();

  return 0;
}
