#ifndef PHOENIX_SENSOR_TASK_RECV_MPU_DATA_H_
#define PHOENIX_SENSOR_TASK_RECV_MPU_DATA_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "math/math_utils.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "communication/udp_mpu_node.h"

#include "common/task.h"
#include "ad_msg.h"
#include "MpuState.pb.h"


namespace phoenix {
namespace sensor {


class TaskRecvMpuData : public framework::Task {
public:
  explicit TaskRecvMpuData(framework::Task* manager);
  ~TaskRecvMpuData();


  bool Start();
  bool Stop();

  bool IsMpuDataReady() const {
    return (mpu_data_ready_);
  }

private:
  typedef boost::unique_lock<boost::mutex> Lock;
  
  framework::mpu::UdpParam * udp_mpu_param_;
  framework::mpu::UdpMPUNode * udp_mpu_node_;

  boost::atomic_bool running_flag_recv_;

  ad_msg::Chassis chassis_;

  bool mpu_data_ready_;

};


}  // namespace sensor
}  // namespace phoenix










#endif
