/*
 * StateMachine.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "obcore/statemachine/IState.h"

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class   StateMachine
 * @author  Stefan May
 * @date    23.10.2014
 * @brief   StateMachine to manage state machine. Designed as singleton pattern.
 */
class StateMachine
{
public:
  /**
   * Access singleton instance
   * @return instance
   */
  static StateMachine* getInstance(void);

  /**
   * Default destructor
   */
  virtual ~StateMachine(void);

  /**
   * Function to set state
   * @param state
   */
  void setState(IState* state);

  /**
   * Function to get current state
   * @return
   */
  IState* getState(void) const;

  /**
   * Function for processing
   */
  void process(void);

private:
  /**
   * Private constructor for singleton pattern
   */
  StateMachine(void);

  /**
   * Private copy constructor for singleton pattern
   * @param c
   */
  StateMachine(StateMachine &c);

  IState* _currentState;

};

} /* namespace obvious */

#endif /* STATEMACHINE_H_ */
