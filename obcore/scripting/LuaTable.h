#ifndef _LUATABLE_H_
#define _LUATABLE_H_

#include "lua5.1/lua.hpp"

namespace obvious
{

class LuaTable
{
public:

  LuaTable(lua_State* L);

  virtual ~LuaTable();

  bool getBool(const char* key);

  double getDouble(const char* key);

private:

  lua_State* _L;
};

} /* namespace obvious */

#endif /* _LUATABLE_H_ */
