#ifndef AGENTROBOT_H_
#define AGENTROBOT_H_

#include "obcore/statemachine/Agent.h"
#include "obcore/base/Point.h"
#include "obcore/base/Timer.h"

#include <vector>

/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   AgentRobot
 * @brief   Specific class for mobile agents (robots) with state machine
 * @author  Stefan May
 * @date    17.6.2015
 */
class AgentRobot : public Agent
{

public:

  /**
   * Constructor
   * @param pos 3D position, orientation is set to (0, 0, 0)
   */
  AgentRobot(Point pos);

  /**
   * Constructor
   * @param pos 2D position, orientation is set to (0, 0, 0)
   */
  AgentRobot(Point2D pos);

  /**
   * Destructor
   */
  virtual ~AgentRobot();

  /**
   * Check whether pose was updated within given interval in ms
   * @param interval time interval in seconds
   */
  bool isPoseUpToDate(const double interval);

  /**
   * Set position
   * @param pos position (x, y, z)
   */
  void setPosition(Point pos);

  /**
   * Set position
   * @param pos position (x, y, 0)
   */
  void setPosition(Point2D pos);

  /**
   * Set position
   * @param x x-coordinate
   * @param y y-coordinate
   * @param z z-coordinate
   */
  void setPosition(double x, double y, double z);

  /**
   * Access current position
   * @param pos position (x, y, z)
   */
  void getPosition(Point& pos);

  /**
   * Access current position
   * @param pos position (x, y)
   */
  void getPosition(Point2D& pos);

  /**
   * Access current position
   * @param x x-coordinate
   * @param y y-coordinate
   * @param z z-coordinate
   */
  void getPosition(double& x, double& y, double& z);

  /**
   * Set orientation of robot around z-axis (psi and theta will be set to 0)
   * @param phi
   */
  void setOrientation2D(double phi);

  /**
   * Access orientation vector
   * @return orientation orientation vector (x, y, z)
   */
  void getOrientation(double orientation[3]);

  /**
   * Set path
   * @param path
   */
  void setPath(std::vector<obvious::Point2D> path);

  /**
   * Get path
   * @param path
   */
  void getPath(std::vector<obvious::Point2D>& path);

  /**
   * Clear currently assigned path
   */
  void clearPath();

  /**
   * Add target to stack
   * @param target target
   */
  void pushTarget(obvious::Point2D target);

  /**
   * Get target, leave it on stack
   * @return target
   */
  bool getNextTarget(obvious::Point2D& target);

  /**
   * Get target, remove it from stack
   * @return target
   */
  bool popNextTarget(obvious::Point2D& target);

private:

  Point _pos;

  std::vector<obvious::Point2D> _path;

  std::vector<obvious::Point2D> _targets;

  double _orientation[3];

  Timer _timer;
};

} // end namespace

#endif
