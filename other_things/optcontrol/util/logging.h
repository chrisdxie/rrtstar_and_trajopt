#ifndef _LOGGING_H__
#define _LOGGING_H__

#include <string>
#include <iostream>
#include "logging.h"
#include <cstdlib>
#include <cstdio>
using namespace std;


namespace util {

enum LogLevel {
  LevelFatal = 0,
  LevelError = 1,
  LevelWarn = 2,
  LevelInfo = 3,
  LevelDebug = 4,
  LevelTrace = 5
};

LogLevel gLogLevel;
inline LogLevel GetLogLevel() {return gLogLevel;}

#define FATAL_PREFIX "\x1b[31m[FATAL] "
#define ERROR_PREFIX "\x1b[31m[ERROR] "
#define WARN_PREFIX "\x1b[33m[WARN] "
#define INFO_PREFIX "[INFO] "
#define DEBUG_PREFIX "\x1b[32m[DEBUG] "
#define TRACE_PREFIX "\x1b[34m[TRACE] "
#define LOG_SUFFIX "\x1b[0m\n"
  
#define LOG_FATAL(msg, ...) if (util::GetLogLevel() >= util::LevelFatal) {printf(FATAL_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_ERROR(msg, ...) if (util::GetLogLevel() >= util::LevelError) {printf(ERROR_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_WARN(msg, ...) if (util::GetLogLevel() >= util::LevelWarn) {printf(WARN_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_INFO(msg, ...) if (util::GetLogLevel() >= util::LevelInfo) {printf(INFO_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_DEBUG(msg, ...) if (util::GetLogLevel() >= util::LevelDebug) {printf(DEBUG_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}
#define LOG_TRACE(msg, ...) if (util::GetLogLevel() >= util::LevelTrace) {printf(TRACE_PREFIX); printf(msg, ##__VA_ARGS__); printf(LOG_SUFFIX);}

inline int LoggingInit() {
	const char* VALID_THRESH_VALUES = "FATAL ERROR WARN INFO DEBUG TRACE";

	char* lvlc = getenv("OPTCON_LOG_THRESH");
	string lvlstr;
	if (lvlc == NULL) {
		printf("You can set logging level with OPTCON_LOG_THRESH. Valid values: %s. Defaulting to INFO\n", VALID_THRESH_VALUES);
		lvlstr = "INFO";
	}
	else lvlstr = string(lvlc);
	if (lvlstr == "FATAL") gLogLevel = LevelFatal;
	else if (lvlstr == "ERROR") gLogLevel =  LevelError;
	else if (lvlstr == "WARN") gLogLevel = LevelWarn;
	else if (lvlstr == "INFO") gLogLevel = LevelInfo;
	else if (lvlstr == "DEBUG") gLogLevel = LevelDebug;
	else if (lvlstr == "TRACE") gLogLevel = LevelTrace;
	else {
		printf("Invalid value for environment variable OPTCON_LOG_THRESH: %s\n", lvlstr.c_str());
		printf("Valid values: %s\n", VALID_THRESH_VALUES);
		abort();
	}
	return 1;
}

int this_is_a_hack_but_rhs_executes_on_library_load = LoggingInit();

}

#endif
