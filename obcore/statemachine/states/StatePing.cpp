#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/StateMachine.h"
#include "StatePing.h"
#include "StatePong.h"

namespace obvious
{

StatePing::StatePing(StateMachine* machine) : StateBase(machine)
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
    _machine->setState(new StatePong(_machine));
    delete this;
  }
}

} /* namespace obvious */
