#ifndef _LUASCRIPTMANAGER_H_
#define _LUASCRIPTMANAGER_H_

#include "lua5.1/lua.hpp"
#include "obcore/scripting/LuaTable.h"
#include "obcore/scripting/LuaMethod.h"

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
   * Constructor
   */
  LuaScriptManager();

  /**
   * Destructor
   */
  virtual ~LuaScriptManager();

  /**
   * Read table from Lua script
   * @param filepath filepath to Lua script
   * @param tablename name of table
   * @return
   */
  LuaTable* readTable(const char* filepath, const char* tablename);

  /**
   * Read method from Lua script
   * @param filepath
   * @param methodname
   * @return
   */
  LuaMethod* readMethod(const char* filepath, const char* methodname);

private:

  lua_State* _L;

};

} /* namespace obvious */

#endif /* _LUASCRIPTMANAGER_H_ */
