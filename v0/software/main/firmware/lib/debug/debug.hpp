#pragma once

#include <string>
namespace debug {

#ifdef DEBUG
void printf(const char* format, ...);
#else
inline void printf(const char* format, ...) {return;}
#endif

}
