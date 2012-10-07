#include <iostream>
#include "obcore/base/Logger.h"

using namespace std;
using namespace obvious;

int main(int argc, char* argv[])
{
	LOGMSG_CONF("logging_example.log", Logger::file_on|Logger::screen_on, DBG_DEBUG, DBG_ERROR);
	LOGMSG(DBG_DEBUG, "hello " << "world");
	LOGMSG(DBG_WARN, "something " << "strange");
	LOGMSG(DBG_ERROR, "this is an error");
}
