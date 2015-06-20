#ifndef STATELUA_H_
#define STATELUA_H_

#include <obcore/statemachine/states/StateBaseModel.h>
#include "obcore/scripting/LuaScriptManager.h"

namespace obvious
{

/**
 * @class StateLua
 * @brief Generic state for interfacing Lua scripts
 * @author Stefan May
 */
class StateLua: public StateBaseModel
{
public:

  /**
   * Constructor
   */
  StateLua(AgentModel* model, const char* filepath);

  /**
   * Destructor
   */
  virtual ~StateLua();

  /**
   * Set automatic reload flag. If true, the lua script is reloaded every cycle.
   * @param autoReload reload flag
   */
  void setAutoReload(bool autoReload);

  /**
   * Get automatic reload flag. If true, the lua script is reloaded every cycle.
   * @param return reload flag
   */
  bool getAutoReload();

  /**
   * Called once when registered
   */
  virtual void onSetup();

  /**
   * Called once when activated
   */
  virtual void onEntry();

  /**
   * Process method (step-wise, never block this method)
   */
  virtual void onActive();

  /**
   * Called once when left
   */
  virtual void onExit();

private:

  void reload();

  LuaScriptManager* _manager;

  bool _autoReload;

  bool _loadInThisCycle;

};

} /* namespace obvious */

#endif /* STATELUA_H_ */
