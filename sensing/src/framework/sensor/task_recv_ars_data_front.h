//
#ifndef PHOENIX_SENSOR_TASK_RECV_ARS_DATA_FRONT_H_
#define PHOENIX_SENSOR_TASK_RECV_ARS_DATA_FRONT_H_

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


class TaskRecvArsDataFront : public framework::Task {
public:
  explicit TaskRecvArsDataFront(framework::Task* manager);
  ~TaskRecvArsDataFront();


  bool Start();
  bool Stop();


  bool IsAsrDataReady() const {
    return (asr_data_ready_);
  }

  const ad_msg::ObstacleRadarList& GetObstacles() const {
    return (obj_list_.GetObjList());
  }

private:
  void ThreadReceiving();

  void FullCanFrame(const can_dev::CanFrame& frame);
  void ParseCanFrame(const can_dev::CanFrame& frame);

  struct ObjList {
    ObjList() {
      Clear();
    }

    void Clear() {
      obstacle_list_.obstacle_num = 0;
    }

    ad_msg::ObstacleRadar* Back() {
      ad_msg::ObstacleRadar* obj = Nullptr_t;

      if (obstacle_list_.obstacle_num <
          ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM) {
        obj = &(obstacle_list_.obstacles[obstacle_list_.obstacle_num]);
      }

      return (obj);
    }

    void PushBack() {
      obstacle_list_.obstacle_num++;
      if (obstacle_list_.obstacle_num >
          ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM) {
        obstacle_list_.obstacle_num =
            ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM;
      }
    }

    void PopBack() {
      obstacle_list_.obstacle_num--;
      if (obstacle_list_.obstacle_num < 0) {
        obstacle_list_.obstacle_num = 0;
      }
    }

    const ad_msg::ObstacleRadarList& GetObjList() const {
      return (obstacle_list_);
    }

    ad_msg::ObstacleRadarList obstacle_list_;
  };

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_;
  boost::thread thread_recv_;

  common::Matrix<Float32_t, 3, 3> mat_calibration_;
  ad_msg::Chassis chassis_;

  // 大陆430雷达原始CAN数据
  phoenix::can_dev::CanFrameList can_frame_list;

  ObjList obj_list_;
  Int32_t single_obj_ready_mask_;
  Int32_t obj_can_frame_cnts_;
  bool asr_data_ready_;
};


}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_ARS_DATA_FRONT_H_
