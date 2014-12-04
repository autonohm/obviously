#include <stdlib.h>
#include <iostream>

#include "obcore/statemachine/StateMachine.h"
#include "StatePong.h"
#include "StatePing.h"

namespace obvious
{

StatePong::StatePong(StateMachine* machine) : StateBase(machine)
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
    _machine->setState(new StatePing(_machine));
    delete this;
  }
}

} /* namespace obvious */
