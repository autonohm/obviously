#ifndef AGENT_H_
#define AGENT_H_

#include "obcore/statemachine/StateMachine.h"
#include "obcore/base/Point.h"

#include <vector>
/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   Agent
 * @brief   Generic class for mobile agents (robots) with state machine
 * @author  Stefan May
 * @date    23.10.2014
 */
class Agent
{

public:

  /**
   * Constructor
   * @param pos 3D position, orientation is set to (0, 0, 0)
   */
  Agent(Point pos);

  /**
   * Constructor
   * @param pos 2D position, orientation is set to (0, 0, 0)
   */
  Agent(Point2D pos);

  /**
   * Destructor
   */
  virtual ~Agent();

  /**
   * Main processing method
   */
  void process();

  /**
   * Access agent's state machine
   * @return state machine
   */
  StateMachine* getStateMachine();

  /**
   * Set state
   * @param state new state
   */
  void setState(StateBase* state);

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
   * Get unique ID of agent, each instance is assigned a sequential ID
   * @return ID
   */
  unsigned int getID();

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
   * Add target
   * @param target target
   */
  void addTarget(obvious::Point2D target);

  /**
   * Get target
   * @return target
   */
  bool getNextTarget(obvious::Point2D& target);

private:

  Point _pos;

  std::vector<obvious::Point2D> _path;

  std::vector<obvious::Point2D> _targets;

  double _orientation[3];

  unsigned int _ID;

  StateMachine* _machine;

};

} // end namespace

#endif
