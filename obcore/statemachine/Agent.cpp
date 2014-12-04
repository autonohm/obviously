#include "Agent.h"

/**
 * @namespace  obvious
 */
namespace obvious
{

static unsigned int _AgentID = 0;

Agent::Agent(double x, double y)
{
  _x = x;
  _y = y;
  _ID = _AgentID++;
}

Agent::~Agent()
{

}

void Agent::setPosition(double x, double y)
{
  _x = x;
  _y = y;
}

void Agent::getPosition(double &x, double &y)
{
  x = _x;
  y = _y;
}

void Agent::setOrientation(double theta)
{
  _theta = theta;
}

double Agent::getOrientation()
{
  return _theta;
}

unsigned int Agent::getID()
{
  return _ID;
}

} // end namespace

