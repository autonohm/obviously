#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StatePing.h"
#include "StatePong.h"

namespace obvious
{

StatePing::StatePing(Agent* agent) : StateBase(agent)
{

}

StatePing::~StatePing(void)
{

}

void StatePing::process(void)
{
  std::cout << "Ping" << std::endl;
  if(rand()%100<30)
  {
    _agent->setState(new StatePong(_agent));
    delete this;
  }
}

} /* namespace obvious */
