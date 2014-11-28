/*
 * Context.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include "Context.h"
#include <cstddef>

namespace obvious
{

static Context* _context;

Context::Context(void)
{
  _currentState = NULL;
}

Context::Context(Context& c)
{
  _currentState = NULL;
}

Context* Context::getInstance(void)
{
  if(_context==NULL) _context = new Context();

  return _context;
}

Context::~Context(void)
{
  if(_context) delete _context;
}

IState* Context::getState(void) const
{
   return _currentState;
}

void Context::setState(IState* state)
{
  _currentState = state;
}

void Context::process(void)
{
  if(_currentState) _currentState->process();
}

} /* namespace obvious */
