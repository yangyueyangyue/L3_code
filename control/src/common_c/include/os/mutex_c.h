// mutex.h
#ifndef PHOENIX_COMMON_OS_MUTEX_C_H_
#define PHOENIX_COMMON_OS_MUTEX_C_H_

#include "utils/macros.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <pthread.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <windows.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#ifdef __cplusplus
extern "C" {
#endif
#include <INTEGRITY.h>
#ifdef __cplusplus
}
#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif


/// 对互斥锁的封装
typedef struct _CommonMutex_t CommonMutex_t;
struct _CommonMutex_t {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_t mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  CRITICAL_SECTION critical;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Semaphore mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  Int32_t fake;
#else
  ERROR: Lock function has not been defined.  
#endif
};

void Phoenix_Common_Os_Mutex_Init(CommonMutex_t* const mu);
void Phoenix_Common_Os_Mutex_Destroy(CommonMutex_t* const mu);
void Phoenix_Common_Os_Mutex_Lock(CommonMutex_t* const mu);
void Phoenix_Common_Os_Mutex_Unlock(CommonMutex_t* const mu);


/// 对读写锁的封装
typedef struct _CommonReadWriteMutex_t CommonReadWriteMutex_t;
struct _CommonReadWriteMutex_t {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_t mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  CRITICAL_SECTION critical;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Semaphore mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  Int32_t fake;
#else
  ERROR: Lock function has not been defined.
#endif
};

void Phoenix_Common_Os_ReadWriteMutex_Init(
    CommonReadWriteMutex_t* const mu);
void Phoenix_Common_Os_ReadWriteMutex_Destroy(
    CommonReadWriteMutex_t* const mu);
void Phoenix_Common_Os_ReadWriteMutex_LockRead(
    CommonReadWriteMutex_t* const mu);
void Phoenix_Common_Os_ReadWriteMutex_LockWrite(
    CommonReadWriteMutex_t* const mu);
void Phoenix_Common_Os_ReadWriteMutex_Unlock(
    CommonReadWriteMutex_t* const mu);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_COMMON_OS_MUTEX_C_H_

