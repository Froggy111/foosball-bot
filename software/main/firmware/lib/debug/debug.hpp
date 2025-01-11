#pragma once

namespace debug {

#ifdef DEBUG
void printf(const char* format, ...);
#else
inline void printf(const std::string &format, ...) {return;}
#endif

}
