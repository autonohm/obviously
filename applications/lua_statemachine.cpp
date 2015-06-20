#include <iostream>
#include <unistd.h>

#include "obcore/statemachine/AgentModel.h"
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

  // The state machine might use a user-defined model. For that purpose use your own model class instances and pass them to your states.
  AgentModel model;

  // Instantiate generic agent. It processes activated states. You can use Lua states as well as pure C++ states (see statemachine_test).
  Agent agent;

  // Instantiate Lua states from passed file path parameters
  for(int i=1; i<argc; i++)
  {
    StateLua* state = new StateLua(&model, argv[i]);

    // Use variable i as ID. For better readability in real applications use an enum type!
    agent.registerPersistantState(i, state);
  }

  // Apply first transition
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
