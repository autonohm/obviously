#include <unistd.h>

#include "obcore/statemachine/StateMachine.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  Agent* agent = new Agent(0, 0);
  StateMachine* machine = new StateMachine(agent);

  machine->setState(new StatePing(machine));

  while(true)
  {
    usleep(100000);
    machine->process();
  }

  delete machine;
  delete agent;

  return 0;
}
