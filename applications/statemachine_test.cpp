#include <unistd.h>

#include "obcore/statemachine/AgentModel.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  AgentModel model;
  Agent agent;
  agent.transitionToVolatileState(new StatePing(&model));

  while(true)
  {
    usleep(100000);
    agent.awake();
  }

  return 0;
}
