// mutex.h
#include "os/mutex_c.h"


/// 对互斥锁的封装
void Phoenix_Common_Os_Mutex_Init(CommonMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_init(&(mu->mutex), Nullptr_t);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  InitializeCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  CreateBinarySemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_Mutex_Destroy(CommonMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_destroy(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  DeleteCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  CloseSemaphore(mu->mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_Mutex_Lock(CommonMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_lock(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  EnterCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  WaitForSemaphore(mu->mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_Mutex_Unlock(CommonMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_unlock(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  LeaveCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  ReleaseSemaphore(mu->mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}


/// 对读写锁的封装
void Phoenix_Common_Os_ReadWriteMutex_Init(
    CommonReadWriteMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_init(&(mu->mutex), Nullptr_t);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  InitializeCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  CreateBinarySemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_ReadWriteMutex_Destroy(
    CommonReadWriteMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_destroy(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  DeleteCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  CloseSemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_ReadWriteMutex_LockRead(
    CommonReadWriteMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_rdlock(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  EnterCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  WaitForSemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_ReadWriteMutex_LockWrite(
    CommonReadWriteMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_wrlock(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  EnterCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  WaitForSemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

void Phoenix_Common_Os_ReadWriteMutex_Unlock(
    CommonReadWriteMutex_t* const mu) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_unlock(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  LeaveCriticalSection(&(mu->critical));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  ReleaseSemaphore(&(mu->mutex));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  mu->fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}


