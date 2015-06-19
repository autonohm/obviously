#ifndef AGENTMODEL_H_
#define AGENTMODEL_H_

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
 * @class   AgentModel
 * @brief   Generic class for agent models. Use this as template for your concrete model.
 * @author  Stefan May
 * @date    17.6.2015
 */
class AgentModel
{

public:

  /**
   * Constructor
   */
  AgentModel();

  /**
   * Destructor
   */
  virtual ~AgentModel();

private:

};

} // end namespace

#endif
