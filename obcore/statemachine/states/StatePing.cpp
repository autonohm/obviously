#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/Agent.h"
#include "StatePing.h"
#include "StatePong.h"

namespace obvious
{

StatePing::StatePing(AgentModel* model) : StateBaseModel(model)
{
}

StatePing::~StatePing()
{

}

void StatePing::onEntry()
{
  std::cout << "Enter Ping state:";
}

void StatePing::onActive()
{
  std::cout << " Ping" << std::flush;

  if(rand()%100<30)
    _agent->transitionToVolatileState(new StatePong(_model));
}

void StatePing::onExit()
{
  std::cout << " ... leaving" << std::endl << std::flush;
}

} /* namespace obvious */
