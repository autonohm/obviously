#ifndef STATEBASEAGENT_H_
#define STATEBASEAGENT_H_

#include <iostream>
#include <map>

#include "obcore/statemachine/states/StateBase.h"
#include "obcore/statemachine/AgentModel.h"
/**
 * @namespace  obvious
 */
namespace obvious
{

/**
 * @class   StateBaseModel
 * @brief   This is an example of extending StateBase to allow access to an agent model.
 *          Implement your own StateBaseModel class to provide a specific model class instance to your states.
 * @author  Stefan May
 * @date    19.6.2015
 */
class StateBaseModel : public StateBase
{

public:

  /**
   * Constructor
   */
  StateBaseModel(AgentModel* model) { _model = model; };

  /**
   * Default destructor
   */
  virtual ~StateBaseModel(){};

  /**
   * Called once when activated
   */
  virtual void onEntry() { };

  /**
   * Called while active
   */
  virtual void onActive() = 0;

  /**
   * Called once when left
   */
  virtual void onExit() { };

protected:

  AgentModel* _model;

};

} /* namespace obvious */

#endif /* STATEBASEMODEL_H_ */
