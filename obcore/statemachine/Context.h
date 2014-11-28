/*
 * Context.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include "obcore/statemachine/IState.h"

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class   Context
 * @author  Stefan May
 * @date    23.10.2014
 * @brief   Context to manage state machine. Designed as singleton pattern.
 */
class Context
{
public:
   /**
    * Function to get instance for singleton pattern
    * @return
    */
  static Context* getInstance(void);

  /**
   * Default destructor
   */
  virtual ~Context(void);

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
  Context(void);
  /**
   * Private copy constructor for singleton pattern
   * @param c
   */
  Context(Context &c);

  IState*     _currentState;

};

} /* namespace obvious */

#endif /* CONTEXT_H_ */
