#ifndef AGENT_H_
#define AGENT_H_

#include "obcore/statemachine/StateMachine.h"

/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   Agent
 * @author  Stefan May
 * @date    23.10.2014
 */
class Agent
{

public:

  /**
   *
   * @param x
   * @param y
   */
  Agent(double x, double y);

  /**
   *
   */
  virtual ~Agent();

  /**
   *
   */
  void process();

  /**
   *
   * @return
   */
  StateMachine* getStateMachine();

  /**
   *
   * @param state
   */
  void setState(StateBase* state);

  /**
   *
   * @param x
   * @param y
   */
  void setPosition(double x, double y);

  /**
   *
   * @param x
   * @param y
   */
  void getPosition(double &x, double &y);

  /**
   *
   * @param theta
   */
  void setOrientation(double theta);

  /**
   *
   * @return
   */
  double getOrientation();

  /**
   *
   * @return
   */
  unsigned int getID();

private:

  double _x;

  double _y;

  double _theta;

  unsigned int _ID;

  StateMachine* _machine;

};

} // end namespace

#endif
