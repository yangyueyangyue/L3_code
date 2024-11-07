/* Copyright 2018,2019 Kotei Co., Ltd.
 *
 ******************************************************************************
 ** 程序入口
 ******************************************************************************
 *
 *  程序入口，启动环境配置
 *
 *  @file       main.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/

#include <memory>
#include <iostream>
#include <string>
#include <signal.h>
#include <unistd.h>

#include "utils/macros.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) ||\
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include "QApplication"
#include "QtGui"
#include "QMessageBox"
#endif

#include "google/protobuf/stubs/common.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "boost/algorithm/string.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/format.hpp"
#include "boost/thread.hpp"
#include "boost/atomic.hpp"
#include "ros/ros.h"

#include "utils/log.h"
#include "utils/gps_tools.h"
#include "math/matrix.h"
#include "math/math_utils.h"
#include "pc/task_manager.h"
#include "ad_msg.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) ||\
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include "main_window.h"
#endif


#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
static bool s_main_window_ready = false;
static phoenix::hmi::MainWindow* s_main_window = Nullptr_t;
static boost::mutex s_main_window_mutex;
#endif

// for pausing process
boost::condition_variable s_process_pause_cond;
// handle sig_int
void HandleSigInt(int sig){
  printf("Caught signal %d\n", sig);

  if (SIGINT == sig) {
    // close main window
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    s_main_window_mutex.lock();
    {
      if (s_main_window_ready && (Nullptr_t != s_main_window)) {
        printf("Try to close main window.\n");
        s_main_window->close();
      }
    }
    s_main_window_mutex.unlock();
#endif
    // notify to exit process
    s_process_pause_cond.notify_one();
  }
}

int main(int argc, char* argv[]) {
  int exit_code = 0;

  char path[1024] = { 0 };
  if (readlink("/proc/self/exe", path, sizeof(path)-2) < 1) {
    std::cout << "readlink failed!" << std::endl;
    return (-1);
  }
  std::string current_path(path);
  std::cout << "current_path = " << current_path << std::endl;

  // Set work space
  int separator_count = 0;
  std::string::reverse_iterator it_current_path = current_path.rbegin();
  for (; it_current_path != current_path.rend(); ++it_current_path) {
    if (*it_current_path == '/') {
      if (it_current_path == current_path.rbegin()) {
        continue;
      }
      ++it_current_path;
      ++separator_count;
      if (separator_count > 1) {
        break;
      }
    }
  }
  std::string work_space(current_path.begin(), it_current_path.base());
  std::cout << "work_space = " << work_space << std::endl;

  // Set log dir
  std::string log_dir = work_space + "/log";
  std::cout << "log_dir = " << log_dir << std::endl;
  // Set log file dir
  phoenix::common::LogFile::SetLogDir(log_dir.c_str());
  // Set google log dir
  FLAGS_log_dir = log_dir.c_str();
  // Initialize google log
  google::InitGoogleLogging(argv[0]);
  // 初始化日志功能
  phoenix::common::InitializeLogging();
  // 配置日志选项
  phoenix::common::ConfigLogging(
        5,       // 一般性日志输出等级,对于跟踪日志及一般性日志，小于等于这个等级的才输出
        true,    // 是否输出时间信息
        true,    // 是否输出文件名称
        true,    // 是否输出到控制台
        true,   // 是否输出到环形缓冲区
        false    // 是否输出到文件
        );

  // Check protobuf's version
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Create triangle lookup tables
  phoenix::common::CreateTriangleLookupTables();

  LOG_INFO(5) << "Hello, world!";

#if (ENABLE_GTEST)
//  // Initialize google test
//  ::testing::InitGoogleTest(&argc, argv);
//  // Run testing ...
//  int gtest_ret = RUN_ALL_TESTS();
//  if (0 == gtest_ret) {
//    std::cout << "Successful to run all tests." << std::endl;
//  } else {
//    std::cerr << "Failed to run all tests, error_code="
//              << gtest_ret << std::endl;
//  }
#endif

  // Initialize ros timer
  ros::Time::init();

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) ||\
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  // Create QT Application
  QApplication qt_app(argc, argv);
  // Create windows
  s_main_window_mutex.lock();
  {
    s_main_window = new phoenix::hmi::MainWindow(argc, argv, work_space);
    s_main_window->show();
  }
  s_main_window_mutex.unlock();
#endif

  // Create task manager
  phoenix::framework::TaskManager* task_manager =
      new phoenix::framework::TaskManager(argc, argv, work_space);
  // Start task manager
  Int32_t start_task_ret = task_manager->Start();
  if (phoenix::framework::TaskManager::START_OK != start_task_ret) {
    LOG_ERR << "Failed to start task manager.";

    std::string notify_info;
    switch (start_task_ret) {
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_ROS):
      notify_info = "Failed to start task (Failed to start ROS)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_LCM):
      notify_info = "Failed to start task (Failed to start LCM)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_UDP):
      notify_info = "Failed to start task (Failed to start UDP)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_MSG_RECV):
      notify_info = "Failed to start task (Failed to start Message Receiver)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_MSG_SEND):
      notify_info = "Failed to start task (Failed to start Message Sender)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_ESR):
      notify_info = "Failed to start task (Failed to start ESR)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_MOBILEYE):
      notify_info = "Failed to start task (Failed to start Mobileye)!";
      break;
    case (phoenix::framework::TaskManager::START_ERR_FAILED_TO_START_GNSS):
      notify_info = "Failed to start task (Failed to start GNSS)!";
      break;
    default:
      notify_info = "Failed to start task (Unknown reason)!";
      break;
    }
    LOG_ERR << notify_info.c_str();

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) ||\
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    QMessageBox::critical(Nullptr_t, "Perception",
        notify_info.c_str(),
        QMessageBox::Ok, QMessageBox::NoButton, QMessageBox::NoButton);
#endif
  }

  /// 接收SIGINT信号 (Ctrl+C)
  struct sigaction sig_int_handler;
  sig_int_handler.sa_handler = HandleSigInt;
  sigemptyset(&sig_int_handler.sa_mask);
  sig_int_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_int_handler, NULL);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  s_main_window_mutex.lock();
  {
    s_main_window_ready = true;
  }
  s_main_window_mutex.unlock();
  // enter main window loop
  exit_code = qt_app.exec();
  // exit main window loop
  s_main_window_mutex.lock();
  {
    s_main_window_ready = false;
    delete s_main_window;
    s_main_window = Nullptr_t;
  }
  s_main_window_mutex.unlock();
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  /// 等待SIGINT信号 (Ctrl+C)
  {
    // Waiting
    boost::mutex pause_mutex;
    boost::unique_lock<boost::mutex> lock(pause_mutex);
    s_process_pause_cond.wait(lock);
  }
#endif

  // Stop
  task_manager->Stop();
  delete task_manager;

  LOG_INFO(5) << "See you again.";
  // Close google log
  google::ShutdownGoogleLogging();

  return (exit_code);
}


