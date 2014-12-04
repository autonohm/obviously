#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StatePong.h"
#include "StatePing.h"

namespace obvious
{

StatePong::StatePong(Agent* agent) : StateBase(agent)
{

}

StatePong::~StatePong(void)
{

}

void StatePong::process(void)
{
  std::cout << "Pong" << std::endl;
  if(rand()%100<30)
  {
    _agent->setState(new StatePing(_agent));
    delete this;
  }
}

} /* namespace obvious */
