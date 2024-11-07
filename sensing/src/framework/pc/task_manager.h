/******************************************************************************
 ** 任务管理模块
 ******************************************************************************
 *
 *  管理所有的任务(启动、停止、状态监测等)
 *
 *  @file       task_manager.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_MANAGER_H_
#define PHOENIX_FRAMEWORK_TASK_MANAGER_H_

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "utils/com_utils.h"
#include "os/mutex.h"
#include "common/task.h"
#include "util.h"
#include "work/work_monitor.h"


namespace phoenix {

namespace sensor {
namespace imu {
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
namespace mpsk {
class TaskRecvGnssDataMpsk;
}
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
namespace bdstar {
class TaskRecvGnssDataBdstar;
}
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
namespace intertial_lab {
class TaskRecvGnssIntertialLab;
}
#else
  // 未定义IMU设备
#endif
}  // namespace imu

class TaskRecvArsDataFront;
class TaskRecvMaxieyeData;
class TaskRecvLidarDataFront;
class TaskRecvAnngicRadarData;
class TaskRecvMpuData;
class TaskSaveDataToCsv;

}  // namespace sensor

namespace perception {
class FilterSrr2Detections;
}  // namespace perception


namespace framework {
class RosNode;
class LcmNode;
class UdpNode;
class MsgReceiver;
class MsgSender;


class TaskManager : public Task {
 public:
  TaskManager(int argc, char** argv, const std::string& work_space);
  ~TaskManager();

  enum {
    START_OK = 0,
    START_ERR_FAILED_TO_START_ROS,
    START_ERR_FAILED_TO_START_LCM,
    START_ERR_FAILED_TO_START_UDP,
    START_ERR_FAILED_TO_START_MSG_RECV,
    START_ERR_FAILED_TO_START_MSG_SEND,
    START_ERR_FAILED_TO_START_ESR,
    START_ERR_FAILED_TO_START_MOBILEYE,
    START_ERR_FAILED_TO_START_GNSS,
    START_ERR_FAILED_TO_START_RR51W_REAR,
    START_ERR
  };
  Int32_t Start();
  bool Stop();

 private:
  void ThreadCheckTasksStatus();
  bool HandleMessage(const Message& msg, Task* sender) override;

private:
  typedef boost::unique_lock<boost::mutex> Lock;

#if (ENABLE_ROS_NODE)
  std::unique_ptr<RosNode> ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  std::unique_ptr<LcmNode> lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  std::unique_ptr<UdpNode> udp_node_;
#endif
  std::unique_ptr<MsgReceiver> msg_receiver_;
  std::unique_ptr<MsgSender> msg_sender_;
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  std::unique_ptr<sensor::imu::mpsk::TaskRecvGnssDataMpsk> task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  std::unique_ptr<sensor::imu::bdstar::TaskRecvGnssDataBdstar> task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  std::unique_ptr<sensor::imu::intertial_lab::TaskRecvGnssIntertialLab> task_recv_gnss_data_;
#else
  // 未定义IMU设备
#endif
  std::unique_ptr<sensor::TaskRecvArsDataFront> task_recv_ars_data_front_;
  std::unique_ptr<sensor::TaskRecvMaxieyeData> task_recv_maxieye_data_;
  std::unique_ptr<sensor::TaskRecvLidarDataFront> task_recv_lidar_data_front_;
  std::unique_ptr<sensor::TaskRecvAnngicRadarData> task_recv_anngic_radar_data_;
  std::unique_ptr<sensor::TaskRecvMpuData> task_recv_mpu_data_;
  std::unique_ptr<sensor::TaskSaveDataToCsv> task_save_data_to_csv_;

  boost::atomic_bool thread_running_flag_check_tasks_status_;
  boost::thread thread_check_tasks_status_;

private:
  WorkMonitor work_monitor_;

  std::string work_space_;

  // log
  char str_buff_status_[1024*2];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TASK_MANAGER_H_
