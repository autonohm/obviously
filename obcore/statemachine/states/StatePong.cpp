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

void StatePong::onEntry()
{
  std::cout << "Enter Pong state:" << std::flush;
}

void StatePong::onActive()
{
  std::cout << " Pong" << std::flush;

  if(rand()%100<30)
    _agent->transitionToVolatileState(new StatePing());
}

void StatePong::onExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* namespace obvious */
