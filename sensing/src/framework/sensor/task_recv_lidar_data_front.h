//
#ifndef PHOENIX_SENSOR_TASK_RECV_LIDAR_DATA_FRONT_H_
#define PHOENIX_SENSOR_TASK_RECV_LIDAR_DATA_FRONT_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "math/math_utils.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"

#include "common/task.h"
#include "can_dev/can_driver.h"
#include "ad_msg.h"


namespace phoenix {
namespace sensor {


class TaskRecvLidarDataFront : public framework::Task {
public:
  explicit TaskRecvLidarDataFront(framework::Task* manager);
  ~TaskRecvLidarDataFront();

  bool Start();
  bool Stop();

private:
  void ThreadReceiving();

  void FullCanFdFrame(const can_dev::CanFdFrame& frame);
  void ParseCanFrame(const can_dev::CanFdFrame& frame);

  void ParseObjFromCanFrame(const can_dev::CanFdFrame& frame);

  struct ObjList {
    ObjList() {
      Clear();
    }

    void Clear() {
      obstacle_list_.obstacle_num = 0;
      // obstacle_list_.Clear();
    }

    ad_msg::ObstacleLidar* Back() {
      ad_msg::ObstacleLidar* obj = Nullptr_t;

      if (obstacle_list_.obstacle_num <
          ad_msg::ObstacleLidarList::MAX_OBSTACLE_NUM) {
        obj = &(obstacle_list_.obstacles[obstacle_list_.obstacle_num]);
      }

      return (obj);
    }

    void PushBack() {
      obstacle_list_.obstacle_num++;
      if (obstacle_list_.obstacle_num >
          ad_msg::ObstacleLidarList::MAX_OBSTACLE_NUM) {
        obstacle_list_.obstacle_num =
            ad_msg::ObstacleLidarList::MAX_OBSTACLE_NUM;
      }
    }

    void PopBack() {
      obstacle_list_.obstacle_num--;
      if (obstacle_list_.obstacle_num < 0) {
        obstacle_list_.obstacle_num = 0;
      }
    }

    const ad_msg::ObstacleLidarList& GetObjList() const {
      return (obstacle_list_);
    }

    ad_msg::ObstacleLidarList obstacle_list_;
  };

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_;
  boost::thread thread_recv_;

  common::Matrix<Float32_t, 3, 3> mat_calibration_;
  ad_msg::Chassis chassis_;

  // 原始CANFD数据
  phoenix::can_dev::CanFdFrameList canfd_frame_list;

  bool obj_list_ready_;
  ObjList obj_list_;
};


}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_LIDAR_DATA_FRONT_H_
