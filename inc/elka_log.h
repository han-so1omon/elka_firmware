#ifndef ELKA_LOG
#define ELKA_LOG

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__PX4_QURT) || defined(__PX4_POSIX)

#define LOG_DEBUG(...) PX4_DEBUG(...);
#define LOG_INFO(...) PX4_INFO(...);
#define LOG_WARN(...) PX4_WARN(...);
#define LOG_ERR(...) PX4_ERR(...);

#elif defined(__ELKA_FREERTOS)

//TODO log thru ftdi 
// Debug output on the ELKA board
#include <stdio.h>
#define LOG_DEBUG(...) 
#define LOG_INFO(...) 
#define LOG_WARN(...)
#define LOG_ERR(...) 

#endif

#ifdef __cplusplus
}
#endif

#endif
