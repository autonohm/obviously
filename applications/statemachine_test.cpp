#include <unistd.h>

#include "obcore/statemachine/StateMachine.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  StateMachine* machine = StateMachine::getInstance();

  machine->setState(new StatePing());

  while(true)
  {
    usleep(100000);
    machine->process();
  }
  return 0;
}
