#include "StateBase.h"
#include <stddef.h>

namespace obvious
{

StateBase::StateBase(bool persistant)
{
  _agent = NULL;
  _persistant = persistant;
}

void StateBase::setAgent(Agent* agent)
{
  _agent = agent;
}

} /* namespace obvious */

