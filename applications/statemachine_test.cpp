#include <unistd.h>

#include "obcore/statemachine/AgentModel.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

/**
 * Example showing usage of obviously state machine
 * @author Stefan May
 * @date 17.6.2015
 */
int main(int argc, char* argv[])
{
  // The state machine might use a user-defined model. For that purpose use your own model class instances and pass them to your states.
  AgentModel model;

  // Instantiate generic agent. It processes activated states. You can use C++ states as well as Lua states (see lua_statemachine example).
  Agent agent;

  // Pass first state to agent.
  agent.transitionToVolatileState(new StatePing(&model));

  while(true)
  {
    // Process current state and make necessary transitions.
    // If no state has been passed to the agent, awake returns without effect.
    agent.awake();

    // Slow down loop to make monitor console output readable
    usleep(100000);
  }

  return 0;
}
