#ifndef STATEBASE_H_
#define STATEBASE_H_

/**
 * @namespace  obvious
 */
namespace obvious
{

class StateMachine;

/**
 * @class   StateBase
 * @author  Stefan May
 * @date    23.10.2014
 */
class StateBase
{

public:

  StateBase(StateMachine* machine);

  /**
   * Default destructor
   */
  virtual ~StateBase(void){};

  /**
   * Processing
   */
  virtual void process(void) = 0;

protected:

  StateMachine* _machine;

};

} /* namespace obvious */

#endif /* STATEBASE_H_ */
