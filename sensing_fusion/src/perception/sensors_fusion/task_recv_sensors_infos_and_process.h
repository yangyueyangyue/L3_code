//
#ifndef PHOENIX_PERCEPTION_SENSORS_FUSION_TASK_RECV_SENSORS_INFOS_AND_PROCESS_H_
#define PHOENIX_PERCEPTION_SENSORS_FUSION_TASK_RECV_SENSORS_INFOS_AND_PROCESS_H_

#include <memory>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

#include "math/math_utils.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"

#include "common/task.h"
#include "can_dev/can_driver.h"
#include "ad_msg.h"
#include "communication/tcp_client.h"


#include "common/message.h"
#include "work/work_sensors_fusion.h"
#include "src/framework/communication/shared_data.h"

namespace phoenix {
namespace perception {
namespace sensorsfusion{

class ISensorsFusionMainstream;

class TaskRecvSensorsInfosAndProcessor : public framework::Task {
public:
  explicit TaskRecvSensorsInfosAndProcessor(framework::Task* manager);
  ~TaskRecvSensorsInfosAndProcessor();

  bool Start();
  bool Stop();

private:
  void ThreadSensorsFusionProcessor();
  bool HandleMessage(const framework::Message& msg, Task* sender) override;

private: 
  // Thread Planning
  boost::atomic_bool thread_running_flag_sensors_fusion_;
  boost::thread thread_sensors_fusion_;
  boost::unique_lock<boost::mutex> lock_sensors_fusion_;

  framework::WorkSensorsFusion work_sensors_fusion_;

};

}  // namespace sensors fusion
}  // namespace perception
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_SRR2_DATA_REAR_H_
