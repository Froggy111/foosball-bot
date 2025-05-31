#include "config.hpp"

namespace debug {

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_TRACE)
void trace(const char *str, ...);
#else
inline void trace([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_DEBUG)
void debug(const char *str, ...);
#else
inline void debug([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_LOG)
void log(const char *str, ...);
#else
inline void log([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_NOTE)
void note(const char *str, ...);
#else
inline void note([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_WARN)
void warn(const char *str, ...);
#else
inline void warn([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_ERROR)
void error(const char *str, ...);
#else
inline void error([[maybe_unused]] const char *str, ...) {}
#endif

#if defined(DEBUG_ENABLED) && defined(DEBUG_LEVEL_FATAL)
void fatal(const char *str, ...);
#else
inline void fatal([[maybe_unused]] const char *str, ...) {}
#endif
} // namespace debug
