#include <cstdarg>
#include <cstdio>
#include <Arduino.h>
#include "types.hpp"

using namespace types;

namespace debug {

#ifdef DEBUG
void printf(const char* format, ...) {
  va_list args0;
  va_start(args0, format);
  va_list args1;
  va_copy(args1, args0);
  u32 buff_size = (vsnprintf(nullptr, 0, format, args0) + 1) * sizeof(char);
  va_end(args0);
  char* buffer = (char*) malloc(buff_size);
  vsprintf(buffer, format, args1);
  Serial.print(buffer);
  va_end(args1);
  return;
}
#endif

}
