#ifndef STATELUA_H_
#define STATELUA_H_

#include "obcore/statemachine/states/StateBase.h"
#include "obcore/scripting/LuaScriptManager.h"

namespace obvious
{

/**
 * @class StateLua
 * @brief Generic state for interfacing Lua scripts
 * @author Stefan May
 */
class StateLua: public StateBase
{
public:

  /**
   * Constructor
   */
  StateLua(const char* filepath);

  /**
   * Destructor
   */
  virtual ~StateLua();

  /**
   * Called once when activated
   */
  void onEntry();

  /**
   * Process method (step-wise, never block this method)
   */
  void onActive();

  /**
   * Called once when left
   */
  void onExit();

private:

  LuaScriptManager* _manager;

};

} /* namespace obvious */

#endif /* STATELUA_H_ */
