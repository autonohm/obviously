#include <unistd.h>

#include "obcore/statemachine/Agent.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  Agent* agent = new Agent(0, 0);

  agent->setState(new StatePing(agent));

  while(true)
  {
    usleep(100000);
    agent->process();
  }

  delete agent;

  return 0;
}
