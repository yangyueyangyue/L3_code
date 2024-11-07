// thread.h
#ifndef PHOENIX_COMMON_OS_THREAD_H_
#define PHOENIX_COMMON_OS_THREAD_H_

#include <unistd.h>
#include <time.h>

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
#include "errorhandling.h"
#ifdef __cplusplus
}
#endif
#endif

#include "utils/macros.h"
#include "utils/log.h"


namespace phoenix {
namespace common {
namespace os {


class Thread {
public:
  struct Parameter {
    Int32_t priority;

    void Clear() {
      priority = 0;
    }

    Parameter() {
      Clear();
    }
  };

private:
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  template <typename ThreadHandlerClass>
  static void* ThreadFunction(void* user) {
    ThreadHandlerClass* handler = (ThreadHandlerClass*)user;
    (*handler)();

    return Nullptr_t;
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  template <typename ThreadHandlerClass>
  static Value ThreadFunction(Address user) {
    ThreadHandlerClass* handler = (ThreadHandlerClass*)user;
    (*handler)();

    return 0;
  }
#endif

public:
  Thread() {
    thread_running_flag_ = false;
  }
  ~Thread() {}

  template <typename ThreadHandlerClass>
  bool Create(const Parameter& param, ThreadHandlerClass* handler) {
    if (thread_running_flag_) {
      LOG_ERR << "Thread is already in running.";
      return false;
    }
    parameter_ = param;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    int ret = pthread_create(&thread_id_, Nullptr_t,
                             ThreadFunction<ThreadHandlerClass>, handler);
    if (0 != ret) {
      LOG_ERR << "Failed to create thread.";
      return false;
    }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    CHECK(CommonCreateTaskWithArgument(1,
                                       ThreadFunction<ThreadHandlerClass>,
                                       (Address)handler,0x4000,
                                       "usr_task",
                                       &thread_id_));
    CHECK(RunTask(thread_id_));
#endif

    thread_running_flag_ = true;

    return true;
  }

  bool Join() {
    if (thread_running_flag_) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
      pthread_join(thread_id_, Nullptr_t);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
      CommonCloseTask(thread_id_);
#endif
      thread_running_flag_ = false;
    }

    return true;
  }

private:
  Parameter parameter_;

  bool thread_running_flag_;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_t thread_id_;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Task thread_id_;
#endif
};


template<typename ThreadHandlerClass>
class ThreadFuncHelper{
public:
  ThreadFuncHelper() {
    thread_handler_class_ = Nullptr_t;
    thread_func_ = Nullptr_t;
  }

  void SetThreadFunc(ThreadHandlerClass* thread_handler_class,
                     void (ThreadHandlerClass::*func)(void)) {
    thread_handler_class_ = thread_handler_class;
    thread_func_ = func;
  }

  void operator ()() {
    // std::cout << "ThreadFuncHelper" << std::endl;
    if ((Nullptr_t != thread_handler_class_) && (Nullptr_t != thread_func_)) {
      (thread_handler_class_->*thread_func_)();
    } else {
      LOG_ERR << "Invalid thread function.";
    }
  }

private:
  ThreadHandlerClass* thread_handler_class_;
  void (ThreadHandlerClass::*thread_func_)(void);
};


} // namespace os
} // namespace common
} // namespace phoenix


#endif // PHOENIX_COMMON_OS_THREAD_H_

