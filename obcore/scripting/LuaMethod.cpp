#include <obcore/scripting/LuaMethod.h>
#include <math.h>

namespace obvious
{

LuaMethod::LuaMethod(lua_State* L, const char* methodname)
{
  _L = L;
  _methodname = methodname;
  reset();
}

LuaMethod::~LuaMethod()
{

}

void LuaMethod::reset()
{
  lua_getglobal(_L, _methodname);
  _params = 0;
}

void LuaMethod::pushBool(bool param)
{
  lua_pushboolean(_L, param);
}

void LuaMethod::pushDouble(double param)
{
  lua_pushnumber(_L, param);
}

void LuaMethod::pushString(const char* param)
{
  lua_pushstring(_L, param);
}

double LuaMethod::execute()
{
  if (!lua_pcall(_L, _params, 1, 0))
  {
    if(lua_isnumber(_L,-1))
      return (double)lua_tonumber(_L, -1);
    else
    {
      return NAN;
    }
  }
  else
  {
    return NAN;
  }
}

} /* namespace obvious */
