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

void StatePing::doEntry()
{
  std::cout << "Enter Ping state:";
}

StateBase* StatePing::doActive()
{
  std::cout << " Ping" << std::flush;

  if(rand()%100<30)
    return new StatePong();

  return NULL;
}

void StatePing::doExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* namespace obvious */
