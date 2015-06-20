#include <obcore/scripting/LuaScriptManager.h>
#include <obcore/base/Logger.h>

#include <cstdarg>

namespace obvious
{

LuaScriptManager::LuaScriptManager(const char* filepath)
{
  _init = false;
  _scriptFilePath = std::string(filepath);

  _L = luaL_newstate();
  luaL_openlibs(_L);

  // load script for the first time
  reload();
}

LuaScriptManager::~LuaScriptManager()
{
  lua_close(_L);
}

void LuaScriptManager::reload()
{
  _init = false;

  if(luaL_loadfile(_L, _scriptFilePath.c_str()) || lua_pcall(_L, 0, 0, 0))
  {
    LOGMSG(DBG_ERROR, "Error calling LUA script");
  }
  else
  {
    _init = true;
  }
}

bool LuaScriptManager::readTable(const char* groupname, std::vector<std::string> &varnames, const char* sig, ...)
{
  if(!_init) return false;

  bool retval = true;

  va_list vl;
  int narg; // number of variables

  va_start(vl, sig);
  lua_getglobal(_L, groupname);
  if(!lua_istable(_L, -1))
  {
    LOGMSG(DBG_ERROR, groupname << " is not a valid table");
    return false;
  }

  // push arguments
  narg = 0;
  while (*sig)
  {
    const char* key = varnames[narg].c_str();
    lua_pushstring(_L, key);
    lua_gettable(_L, -2);
    switch (*sig++)
    {
    case 'b':  // bool argument
      if(!lua_isboolean(_L, -1))
      {
        LOGMSG(DBG_ERROR, "invalid component \"" << key << "\" in table \"" << groupname << "\"");
        retval = false;
      }
      *va_arg(vl, bool *) = lua_toboolean(_L, -1);
      lua_pop(_L, 1);
      break;
    case 'd':  // double argument
      if(!lua_isnumber(_L, -1))
      {
        LOGMSG(DBG_ERROR, "invalid component \"" << key << "\" in table \"" << groupname << "\"");
        retval = false;
      }
      *va_arg(vl, double *) = lua_tonumber(_L, -1);
      lua_pop(_L, 1);
      break;
    case 'i':  // integer argument
      if(!lua_isnumber(_L, -1))
      {
        LOGMSG(DBG_ERROR, "invalid component \"" << key << "\" in table \"" << groupname << "\"");
        retval = false;
      }
      *va_arg(vl, int *) = lua_tonumber(_L, -1);
      lua_pop(_L, 1);
      break;
    case 's':  // string argument
      if(!lua_isstring(_L, -1))
      {
        LOGMSG(DBG_ERROR, "invalid component \"" << key << "\" in table \"" << groupname << "\"");
        retval = false;
      }
      *va_arg(vl, const char **) = lua_tostring(_L, -1);
      lua_pop(_L, 1);
      break;
    default:
      LOGMSG(DBG_ERROR, "invalid option " << *(sig - 1));
      retval = false;
    }
    narg++;
    luaL_checkstack(_L, 1, "too many arguments");
  }

  va_end(vl);

  return retval;
}

bool LuaScriptManager::callFunction (const char* func, const char* sig, ...)
{
  if(!_init) return false;

  bool retval = true;

  va_list vl;
  int narg, nres; // number of arguments and results

  va_start(vl, sig);
  lua_getglobal(_L, func); // get function

  // push arguments
  narg = 0;
  while (*sig)
  {
    switch (*sig++)
    {
    case 'd':  // double argument
      lua_pushnumber(_L, va_arg(vl, double));
      break;
    case 'i':  // integer argument
      lua_pushnumber(_L, va_arg(vl, int));
      break;
    case 's':  // string argument
      lua_pushstring(_L, va_arg(vl, char *));
      break;

    case '>':
      goto endwhile;

    default:
      LOGMSG(DBG_ERROR, "invalid option " << *(sig - 1));
      retval = false;
    }
    narg++;
    luaL_checkstack(_L, 1, "too many arguments");
  } endwhile:

  // do the call
  nres = strlen(sig);  // number of expected results
  if (lua_pcall(_L, narg, nres, 0) != 0)  // do the call
  {
    LOGMSG(DBG_ERROR, "error running function " << func << " " << lua_tostring(_L, -1));
    retval = false;
  }

  // retrieve results
  nres = -nres;  // stack index of first result
  while (*sig)
  {
    switch (*sig++)
    {

    case 'd':  // double result
      if (!lua_isnumber(_L, nres))
      {
        LOGMSG(DBG_ERROR, "'d' is wrong result type in function " << func);
        retval = false;
      }
      *va_arg(vl, double *) = lua_tonumber(_L, nres);
      break;

    case 'i':  // int result
      if (!lua_isnumber(_L, nres))
      {
        LOGMSG(DBG_ERROR, "'i' is wrong result type in function " << func);
        retval = false;
      }
      *va_arg(vl, int *) = (int)lua_tonumber(_L, nres);
      break;

    case 's':  // string result
      if (!lua_isstring(_L, nres))
      {
        LOGMSG(DBG_ERROR, "'s' is wrong result type in function " << func);
        retval = false;
      }
      *va_arg(vl, const char **) = lua_tostring(_L, nres);
      break;

    default:
      LOGMSG(DBG_ERROR, "invalid option " << *(sig - 1));
      retval = false;
    }
    nres++;
  }
  va_end(vl);

  return retval;
}

void LuaScriptManager::registerCallback(lua_CFunction func, const char name[])
{
  lua_pushcfunction(_L, func);
  lua_setglobal(_L, name);
}

} /* namespace obvious */
