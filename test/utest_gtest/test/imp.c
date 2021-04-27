#include "rtklib.h"

#ifndef TRACE
extern int gettracelevel(void) { return 0;}
#endif
extern int showmsg(char* format, ...) { return 0; }
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
