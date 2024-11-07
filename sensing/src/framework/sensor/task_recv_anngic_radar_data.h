//
#ifndef PHOENIX_SENSOR_TASK_RECV_ANNGIC_RADAR_DATA_H_
#define PHOENIX_SENSOR_TASK_RECV_ANNGIC_RADAR_DATA_H_

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


class TaskRecvAnngicRadarData : public framework::Task {
public:
  explicit TaskRecvAnngicRadarData(framework::Task* manager);
  ~TaskRecvAnngicRadarData();

  bool Start();
  bool Stop();

private:
  void ThreadReceiving();

  void FullCanFdFrame(const can_dev::CanFdFrame& frame);
  void ParseCanFrame(const can_dev::CanFdFrame& frame);

  void ParseObjFromCanFrame(const can_dev::CanFdFrame& frame, Uint8_t radar_symbol, Uint16_t *message_count);
  void ParseOneObjFromData(const Uint8_t* data, Uint8_t radar_symbol, Uint16_t *message_count);
  void NotifyObjList(bool obj_list_ready, Uint8_t radar_symbol);

  struct ObjList {
    ObjList() {
      Clear();
    }

    void Clear() {
      obstacle_list_.obstacle_num = 0;
      // obstacle_list_.Clear();
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

  // 侧向雷达标定参数
  common::Matrix<Float32_t, 3, 3> mat_calibration_;
  ad_msg::Chassis chassis_;

  // 原始CANFD数据
  phoenix::can_dev::CanFdFrameList canfd_frame_list;

  // Message Count
  Uint8_t front_left_can_id_count_;
  Uint8_t front_right_can_id_count_;
  Uint8_t rear_left_can_id_count_;
  Uint8_t rear_right_can_id_count_;

  // 前左雷达
  bool front_left_obj_list_ready_;
  ObjList front_left_obj_list_;

  // 前右雷达
  bool front_right_obj_list_ready_;
  ObjList front_right_obj_list_;

  // 后左雷达
  bool rear_left_obj_list_ready_;
  ObjList rear_left_obj_list_;

  // 后右雷达
  bool rear_right_obj_list_ready_;
  ObjList rear_right_obj_list_;
};


}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_ANNGIC_RADAR_DATA_H_
