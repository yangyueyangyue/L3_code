#ifndef PHOENIX_SENSOR_IMU_MPSK_TASK_RECV_GNSS_DATA_MPSK_H_
#define PHOENIX_SENSOR_IMU_MPSK_TASK_RECV_GNSS_DATA_MPSK_H_

#include <memory>
#include <queue>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "ad_msg.h"
#include "serial_dev/serial_driver.h"
#include "common/task.h"


namespace phoenix {
namespace sensor {
namespace imu {
namespace mpsk {


class TaskRecvGnssDataMpsk : public framework::Task {
public:
  explicit TaskRecvGnssDataMpsk(framework::Task* manager);
  ~TaskRecvGnssDataMpsk();

  bool Start();
  bool Stop();

private:
  void ThreadReceivingGnss();

  void SpinSerialData(const Uint8_t* buffer, Int32_t length);

  void ParseMessageFrame(const Uint8_t* buffer, Int32_t length);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_recv_gnss_;
  boost::thread thread_recv_gnss_;

  serial_dev::SerialDriver* serial_dev_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
  Int32_t data_length_;
};


}  // namespace mpsk
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_IMU_MPSK_TASK_RECV_GNSS_DATA_MPSK_H_
