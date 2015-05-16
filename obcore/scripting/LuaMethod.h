#ifndef _LUAMETHOD_H_
#define _LUAMETHOD_H_

#include "lua5.1/lua.hpp"

namespace obvious
{

/**
 * @class LuaMethod
 * @author Stefan May
 * @date = 8.5.2015
 */
class LuaMethod
{
public:
  /**
   * Constructor
   */
  LuaMethod(lua_State* L, const char* methodname);

  /**
   * Destructor
   */
  virtual ~LuaMethod();

  void pushDouble(double param);

  void pushBool(bool param);

  void pushString(const char* param);

  double execute();

private:

  void reset();

  lua_State* _L;

  const char* _methodname;

  unsigned int _params;

};

} /* namespace obvious */

#endif /* OBCORE_SCRIPTING_LUAMETHOD_H_ */
