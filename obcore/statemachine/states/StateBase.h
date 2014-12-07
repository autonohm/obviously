#ifndef STATEBASE_H_
#define STATEBASE_H_

/**
 * @namespace  obvious
 */
namespace obvious
{

class Agent;

/**
 * @class   StateBase
 * @author  Stefan May
 * @date    23.10.2014
 */
class StateBase
{

public:

  /**
   * Constructor
   */
  StateBase();

  /**
   * Default destructor
   */
  virtual ~StateBase(){};

  /**
   * Set agent (needs to be set before call of process method)
   * @param agent agent instance
   */
  void setAgent(Agent* agent);

  /**
   * Processing
   */
  virtual void process() = 0;

protected:

  Agent* _agent;

};

} /* namespace obvious */

#endif /* STATEBASE_H_ */
