#include "AgentRobot.h"
#include <string.h>

namespace obvious
{

AgentRobot::AgentRobot(StateBase* initState, Point pos) : Agent(initState)
{
  _pos = pos;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
}

AgentRobot::AgentRobot(StateBase* initState, Point2D pos) : Agent(initState)
{
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
}

AgentRobot::~AgentRobot()
{

}

bool AgentRobot::isPoseUpToDate(const double interval)
{
  return (_timer.elapsed() <= interval);
}

void AgentRobot::setPosition(Point pos)
{
  _timer.reset();
  _pos = pos;
}

void AgentRobot::setPosition(Point2D pos)
{
  _timer.reset();
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
}

void AgentRobot::setPosition(double x, double y, double z)
{
  _timer.reset();
  _pos.x = x;
  _pos.y = y;
  _pos.z = z;
}

void AgentRobot::getPosition(Point& pos)
{
  pos = _pos;
}

void AgentRobot::getPosition(Point2D& pos)
{
  pos.x = _pos.x;
  pos.y = _pos.y;
}

void AgentRobot::getPosition(double& x, double& y, double& z)
{
  x = _pos.x;
  y = _pos.y;
  z = _pos.z;
}

void AgentRobot::setOrientation2D(double phi)
{
  _orientation[0] = 0.0;
  _orientation[1] = 0.0;
  _orientation[2] = phi;
}

void AgentRobot::getOrientation(double orientation[3])
{
  memcpy(orientation, _orientation, 3*sizeof(*_orientation));
}

void AgentRobot::setPath(std::vector<obvious::Point2D> path)
{
  _path = path;
}

void AgentRobot::getPath(std::vector<obvious::Point2D>& path)
{
  path = _path;
}

void AgentRobot::clearPath()
{
  _path.clear();
}

void AgentRobot::pushTarget(obvious::Point2D target)
{
  _targets.push_back(target);
}

bool AgentRobot::getNextTarget(obvious::Point2D& target)
{
  if(_targets.size()==0) return false;
  target = *_targets.begin();

  return true;
}

bool AgentRobot::popNextTarget(obvious::Point2D& target)
{
  if(_targets.size()==0) return false;

  target = _targets[0];
  _targets.erase(_targets.begin());

  return true;
}

} // end namespace

