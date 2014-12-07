#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StatePong.h"
#include "StatePing.h"

namespace obvious
{

StatePong::StatePong()
{

}

StatePong::~StatePong()
{

}

void StatePong::process()
{
  std::cout << "Pong" << std::endl;
  if(rand()%100<30)
  {
    _agent->setState(new StatePing());
    delete this;
  }
}

} /* namespace obvious */
