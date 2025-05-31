#include "debug.hpp"

#include <cstdarg>
#include <cstdio>

#include "config.hpp"

#ifdef DEBUG_ENABLED

void print_with_prefix(FILE *stream, const char *prefix, const char *str,
                       va_list args) {
  if (prefix && prefix[0] != '\0') {
    fprintf(stream, "%s: ", prefix);
  }
  vfprintf(stream, str, args);
  fprintf(stream, "\r\n");
  fflush(stream);
}

#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_TRACE)
void debug::trace(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "TRACE", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_DEBUG)
void debug::debug(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "DEBUG", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_LOG)
void debug::log(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "LOG", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_NOTE)
void debug::note(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "NOTE", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_WARN)
void debug::warn(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "WARN", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_ERROR)
void debug::error(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "ERROR", str, args);
}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_FATAL)
void debug::fatal(const char *str, ...) {
  va_list args;
  va_start(args, str);
  print_with_prefix(stdout, "FATAL", str, args);
}
#endif
