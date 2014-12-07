#include <unistd.h>

#include "obcore/statemachine/Agent.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  Point2D pos;
  pos.x = 0;
  pos.y = 0;

  Agent* agent = new Agent(pos);

  agent->setState(new StatePing());

  while(true)
  {
    usleep(100000);
    agent->process();
  }

  delete agent;

  return 0;
}
