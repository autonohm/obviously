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

void StatePong::doEntry()
{
  std::cout << "Enter Pong state:" << std::flush;
}

StateBase* StatePong::doActive()
{
  std::cout << " Pong" << std::flush;

  if(rand()%100<30)
    return new StatePing();

  return NULL;
}

void StatePong::doExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* namespace obvious */
