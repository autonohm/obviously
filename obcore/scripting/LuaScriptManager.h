#ifndef _LUASCRIPTMANAGER_H_
#define _LUASCRIPTMANAGER_H_

#include "lua.hpp"

#include <string>
#include <vector>

namespace obvious
{

/**
 * @class LuaScriptManager
 * @brief Interface for Lua scription language
 * @author Stefan May
 * @date 17.6.2015
 */
class LuaScriptManager
{
public:
  /**
   * Constructor
   */
  LuaScriptManager(const char* filepath);

  /**
   * Destructor
   */
  virtual ~LuaScriptManager();

  /**
   * Reload script
   */
  void reload();

  /**
   * Read table from a Lua script
   * @param groupname name of group
   * @param varnames names of variables in the group
   * @param sig signature of parameter list ("b" for bool, "d" for double, "i" for integer, "s" for string), e.g. "bids" = bool integer double char*
   * @return success flag
   */
  bool readTable(const char* groupname, std::vector<std::string> &varnames, const char* sig, ...);

  /**
   * Call function from a Lua script
   * @param func function name
   * @param sig signature of parameter list ("d" for double, "i" for integer, "s" for string), e.g. "id>s" = integer double -> return string
   * @return success flag
   */
  bool callFunction (const char* func, const char* sig, ...);

  /**
   * Register C callback function, available in Lua with specified name
   * @param func pointer to callback function
   * @param name function name available in Lua script
   */
  void registerCallback(lua_CFunction func, char name[]);

private:

  lua_State* _L;

  std::string _scriptFilePath;

  bool _init;

};

} /* namespace obvious */

#endif /* _LUASCRIPTMANAGER_H_ */
