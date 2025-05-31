// configs for FOC firmware

/**
 * @brief Target for stdio functions (printf)
 * @options USB, CANFD, UART
 */
#define STDIO_TARGET STDIO_USB
#define STDIO_NULL 0
#define STDIO_USB 1
#define STDIO_CANFD 2
#define STDIO_UART 3

/**
 * @brief Debug output settings. Comment out defines to disable a log level.
 */
#ifndef NDEBUG

#define DEBUG_ENABLED
#define DEBUG_LEVEL_TRACE
#define DEBUG_LEVEL_DEBUG
#define DEBUG_LEVEL_LOG
#define DEBUG_LEVEL_NOTE
#define DEBUG_LEVEL_WARN
#define DEBUG_LEVEL_ERROR
#define DEBUG_LEVEL_FATAL

#endif
