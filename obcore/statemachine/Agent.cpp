#include "Agent.h"
#include "string.h"

/**
 * @namespace  obvious
 */
namespace obvious
{

static unsigned int _AgentID = 0;

Agent::Agent(Point pos)
{
  _pos = pos;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
  _ID = _AgentID++;
  _machine = new StateMachine();
}

Agent::Agent(Point2D pos)
{
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
  _ID = _AgentID++;
  _machine = new StateMachine();
}

Agent::~Agent()
{
  delete _machine;
}

void Agent::process()
{
  _machine->process();
}

StateMachine* Agent::getStateMachine()
{
  return _machine;
}

void Agent::setState(StateBase* state)
{
  state->setAgent(this);
  _machine->setState(state);
}

void Agent::setPosition(double x, double y, double z)
{
  _pos.x = x;
  _pos.y = y;
  _pos.z = z;
}

void Agent::setPosition(Point2D pos)
{
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
}

void Agent::setPosition(Point pos)
{
  _pos = pos;
}

void Agent::getPosition(double& x, double& y, double& z)
{
  x = _pos.x;
  y = _pos.y;
  z = _pos.z;
}

void Agent::getPosition(Point& pos)
{
  pos = _pos;
}

void Agent::getPosition(Point2D& pos)
{
  pos.x = _pos.x;
  pos.y = _pos.y;
}

void Agent::setOrientation2D(double phi)
{
  _orientation[0] = 0.0;
  _orientation[1] = 0.0;
  _orientation[2] = phi;
}

void Agent::getOrientation(double orientation[3])
{
  memcpy(orientation, _orientation, 3*sizeof(*_orientation));
}

unsigned int Agent::getID()
{
  return _ID;
}

void Agent::setPath(std::vector<obvious::Point2D> path)
{
  _path = path;
}

void Agent::getPath(std::vector<obvious::Point2D>& path)
{
  path = _path;
}

void Agent::clearPath()
{
  _path.clear();
}

void Agent::addTarget(obvious::Point2D target)
{
  _targets.push_back(target);
}

bool Agent::getNextTarget(obvious::Point2D& target)
{
  if(_targets.size()==0) return false;

  target = _targets[0];
  _targets.erase(_targets.begin());

  return true;
}

} // end namespace

