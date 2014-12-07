#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StatePing.h"
#include "StatePong.h"

namespace obvious
{

StatePing::StatePing()
{

}

StatePing::~StatePing()
{

}

void StatePing::process()
{
  std::cout << "Ping" << std::endl;
  if(rand()%100<30)
  {
    _agent->setState(new StatePong());
    delete this;
  }
}

} /* namespace obvious */
