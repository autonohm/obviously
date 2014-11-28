#include <unistd.h>

#include "obcore/statemachine/Context.h"
#include "obcore/statemachine/states/StatePing.h"

using namespace obvious;

int main(int argc, char* argv[])
{
  Context* context = Context::getInstance();

  context->setState(new StatePing());

  while(true)
  {
    usleep(100000);
    context->process();
  }
  return 0;
}
