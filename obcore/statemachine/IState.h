/*
 * IState.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef ISTATE_H_
#define ISTATE_H_

/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   IState
 * @author  Stefan May
 * @date    23.10.2014
 */
class IState
{
public:
  /**
   * Default destructor
   */
  virtual ~IState(void){};

  /**
   * Processing
   */
  virtual void process(void) = 0;
};

} /* namespace obvious */

#endif /* ISTATE_H_ */
