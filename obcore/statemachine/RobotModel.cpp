#include "RobotModel.h"
#include <string.h>

namespace obvious
{

RobotModel::RobotModel(Point pos)
{
  _pos = pos;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
}

RobotModel::RobotModel(Point2D pos)
{
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
  _orientation[0] = 0;
  _orientation[1] = 0;
  _orientation[2] = 0;
}

RobotModel::~RobotModel()
{

}

bool RobotModel::isPoseUpToDate(const double interval)
{
  return (_timer.elapsed() <= interval);
}

void RobotModel::setPosition(Point pos)
{
  _timer.reset();
  _pos = pos;
}

void RobotModel::setPosition(Point2D pos)
{
  _timer.reset();
  _pos.x = pos.x;
  _pos.y = pos.y;
  _pos.z = 0;
}

void RobotModel::setPosition(double x, double y, double z)
{
  _timer.reset();
  _pos.x = x;
  _pos.y = y;
  _pos.z = z;
}

void RobotModel::getPosition(Point& pos)
{
  pos = _pos;
}

void RobotModel::getPosition(Point2D& pos)
{
  pos.x = _pos.x;
  pos.y = _pos.y;
}

void RobotModel::getPosition(double& x, double& y, double& z)
{
  x = _pos.x;
  y = _pos.y;
  z = _pos.z;
}

void RobotModel::setOrientation2D(double phi)
{
  _orientation[0] = 0.0;
  _orientation[1] = 0.0;
  _orientation[2] = phi;
}

void RobotModel::getOrientation(double orientation[3])
{
  memcpy(orientation, _orientation, 3*sizeof(*_orientation));
}

void RobotModel::setPath(std::vector<obvious::Point2D> path)
{
  _path = path;
}

void RobotModel::getPath(std::vector<obvious::Point2D>& path)
{
  path = _path;
}

void RobotModel::clearPath()
{
  _path.clear();
}

void RobotModel::pushTarget(obvious::Point2D target)
{
  _targets.push_back(target);
}

bool RobotModel::getNextTarget(obvious::Point2D& target)
{
  if(_targets.size()==0) return false;
  target = *_targets.begin();

  return true;
}

bool RobotModel::popNextTarget(obvious::Point2D& target)
{
  if(_targets.size()==0) return false;

  target = _targets[0];
  _targets.erase(_targets.begin());

  return true;
}

} // end namespace

