// mutex.h
#ifndef PHOENIX_COMMON_OS_MUTEX_H_
#define PHOENIX_COMMON_OS_MUTEX_H_

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


namespace phoenix {
namespace common {
namespace os {


// 对互斥锁的封装
class Mutex {
public:
  inline Mutex() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_mutex_init(&mutex_, Nullptr_t);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    InitializeCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    CreateBinarySemaphore(&mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline ~Mutex() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
	pthread_mutex_destroy(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    DeleteCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    CloseSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline void Lock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_mutex_lock(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    EnterCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    WaitForSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline void Unlock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_mutex_unlock(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    LeaveCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    ReleaseSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

private:
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_mutex_t mutex_;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  CRITICAL_SECTION critical_;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Semaphore mutex_;
#else
  // Lock function has not been defined.
#endif
};

class ReadWriteMutex {
public:
  inline ReadWriteMutex() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_rwlock_init(&mutex_, NULL);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    InitializeCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    CreateBinarySemaphore(&mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline ~ReadWriteMutex() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_rwlock_destroy(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    DeleteCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    CloseSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline void LockRead() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_rwlock_rdlock(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    EnterCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    WaitForSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline void LockWrite() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_rwlock_wrlock(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    EnterCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    WaitForSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

  inline void Unlock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    pthread_rwlock_unlock(&mutex_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    LeaveCriticalSection(&critical_);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    ReleaseSemaphore(mutex_);
#else
    // Lock function has not been defined.
#endif
  }

private:
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_t mutex_;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  CRITICAL_SECTION critical_;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Semaphore mutex_;
#else
  // Lock function has not been defined.
#endif
};


class LockHelper {
public:
  LockHelper(Mutex& mu) : mutex_(mu) {
    mu.Lock();
  }
  ~LockHelper() {
    mutex_.Unlock();
  }

private:
  Mutex& mutex_;
};


class ReadLockHelper {
public:
  ReadLockHelper(ReadWriteMutex& mu) : mutex_(mu) {
    mu.LockRead();
  }
  ~ReadLockHelper() {
    mutex_.Unlock();
  }

private:
  ReadWriteMutex& mutex_;
};


class WriteLockHelper {
public:
  WriteLockHelper(ReadWriteMutex& mu) : mutex_(mu) {
    mu.LockWrite();
  }
  ~WriteLockHelper() {
    mutex_.Unlock();
  }

private:
  ReadWriteMutex& mutex_;
};


}  // namespace os
}  // namespace common
}  // namespace phoenix


#endif // PHOENIX_COMMON_OS_MUTEX_H_

