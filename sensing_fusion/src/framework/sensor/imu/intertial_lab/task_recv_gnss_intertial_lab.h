//
#ifndef PHOENIX_SENSOR_TASK_RECV_GNSS_INTERTIAL_LAB_H_
#define PHOENIX_SENSOR_TASK_RECV_GNSS_INTERTIAL_LAB_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "common/task.h"
#include "can_dev/can_driver.h"
#include "math/math_utils.h"
#include "ad_msg.h"

namespace phoenix {
namespace sensor {
namespace imu {
namespace intertial_lab {


class TaskRecvGnssIntertialLab : public framework::Task {
public:
  explicit TaskRecvGnssIntertialLab(framework::Task* manager);
  ~TaskRecvGnssIntertialLab();

  bool Start();
  bool Stop();

private:
  void ThreadReceivingGnss();

  void ParseCanFrame(const can_dev::CanFrame& frame);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_gnss_;
  boost::thread thread_recv_gnss_;

  ad_msg::Gnss gnss_info_;
  ad_msg::Imu imu_info_;
};


}  // namespace intertial_lab
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_GNSS_INTERTIAL_LAB_H_
