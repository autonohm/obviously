#include "StateBase.h"
#include <stddef.h>

namespace obvious
{

StateBase::StateBase(bool autoCleanup)
{
  _agent = NULL;
  _autoCleanup = autoCleanup;
}

void StateBase::setAgent(Agent* agent)
{
  _agent = agent;
}

} /* namespace obvious */

