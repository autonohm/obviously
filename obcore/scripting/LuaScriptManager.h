#ifndef _LUASCRIPTMANAGER_H_
#define _LUASCRIPTMANAGER_H_

#include "lua5.1/lua.hpp"
#include "obcore/scripting/LuaTable.h"

namespace obvious
{

/**
 * @class LuaScriptManager
 * @brief
 * @author Stefan May
 */
class LuaScriptManager
{
public:
  /**
   *
   */
  LuaScriptManager();

  /**
   *
   */
  virtual ~LuaScriptManager();

  /**
   *
   * @param filepath
   * @param methodname
   */
  void execute(const char* filepath, const char* methodname);

  LuaTable* readTable(const char* filepath, const char* tablename);

private:

  lua_State* _L;

};

} /* namespace obvious */

#endif /* _LUASCRIPTMANAGER_H_ */
