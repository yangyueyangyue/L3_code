//
#ifndef PHOENIX_SENSOR_TASK_RECV_MAXIEYE_DATA_H_
#define PHOENIX_SENSOR_TASK_RECV_MAXIEYE_DATA_H_

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


class TaskRecvMaxieyeData : public framework::Task {
public:
  explicit TaskRecvMaxieyeData(framework::Task* manager);
  ~TaskRecvMaxieyeData();

  bool Start();
  bool Stop();

private:
  void ThreadReceiving();

  void FullCanFdFrame(const can_dev::CanFdFrame& frame);
  
  void ParseCanFrame(const can_dev::CanFdFrame& frame);

  // 前视一体机
  void ParseLaneFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseOneLaneFromData(const Uint8_t* data);
  void ParseLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseLaneCurbFromCanFrame(const can_dev::CanFdFrame& frame); 
  void ParseOneLaneCurbFromData(const Uint8_t* data); 

  void ParseMaxieyeObjFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseMaxieyeOneObjFromData(const Uint8_t* data);

  void ParseTSRFromCanFrame(const can_dev::CanFdFrame& frame);

  void ParseTrafficConeFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseTwoTrafficConeFromData(const Uint8_t* data);

  // 视觉控制器
  void ParseVisualControlLaneFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseVisualControlOneLaneFromData(const Uint8_t* data);
  void ParseVisualControlLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame);

  void ParseVisualControlObjFromCanFrame(const can_dev::CanFdFrame& frame, Uint8_t camera_symbol);
  void ParseVisualControlObjFromData(const Uint8_t* data, Uint8_t camera_symbol);

  void NotifyObjList(bool obj_list_ready, Uint8_t camera_symbol);

  struct LaneList {
    LaneList() {
      Clear();
    }

    void Clear() {
      lane_mark_list_.lane_mark_num = 0;
    }

    ad_msg::LaneMarkCamera* Back() {
      ad_msg::LaneMarkCamera* obj = Nullptr_t;

      if (lane_mark_list_.lane_mark_num <
          ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM) {
        obj = &(lane_mark_list_.lane_marks[lane_mark_list_.lane_mark_num]);
      }

      return (obj);
    }

    void PushBack() {
      lane_mark_list_.lane_mark_num++;
      if (lane_mark_list_.lane_mark_num >
          ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM) {
        lane_mark_list_.lane_mark_num =
            ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM;
      }
    }

    void PopBack() {
      lane_mark_list_.lane_mark_num--;
      if (lane_mark_list_.lane_mark_num < 0) {
        lane_mark_list_.lane_mark_num = 0;
      }
    }

    const ad_msg::LaneMarkCameraList& GetLaneList() const {
      return (lane_mark_list_);
    }

    void SetLaneList(const ad_msg::LaneMarkCameraList &data) {
      lane_mark_list_ = data;
    }

    ad_msg::LaneMarkCameraList lane_mark_list_;
  };

  struct TraSignList {
    TraSignList() {
      Clear();
    }

    void Clear() {
      traffic_sign_list_.speed_restriction_num = 0;
    }

    ad_msg::TrafficSignalSpeedRestriction* Back() {
      ad_msg::TrafficSignalSpeedRestriction* obj = Nullptr_t;

      if (traffic_sign_list_.speed_restriction_num <
          ad_msg::TrafficSignalList::MAX_TRAFFIC_SIGNAL_NUM) {
        obj = &(traffic_sign_list_.speed_restrictions[traffic_sign_list_.speed_restriction_num]);
      }

      return (obj);
    }

    void PushBack() {
      traffic_sign_list_.speed_restriction_num++;
      if (traffic_sign_list_.speed_restriction_num >
          ad_msg::TrafficSignalList::MAX_TRAFFIC_SIGNAL_NUM) {
        traffic_sign_list_.speed_restriction_num =
            ad_msg::TrafficSignalList::MAX_TRAFFIC_SIGNAL_NUM;
      }
    }

    void PopBack() {
      traffic_sign_list_.speed_restriction_num--;
      if (traffic_sign_list_.speed_restriction_num < 0) {
        traffic_sign_list_.speed_restriction_num = 0;
      }
    }

    const ad_msg::TrafficSignalList& GetTraSignList() const {
      return (traffic_sign_list_);
    }

    ad_msg::TrafficSignalList traffic_sign_list_;
  };

  struct ObjList {
    ObjList() {
      Clear();
    }

    void Clear() {
      obstacle_list_.obstacle_num = 0;
      // obstacle_list_.Clear();
    }

    ad_msg::ObstacleCamera* Back() {
      ad_msg::ObstacleCamera* obj = Nullptr_t;

      if (obstacle_list_.obstacle_num <
          ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM) {
        obj = &(obstacle_list_.obstacles[obstacle_list_.obstacle_num]);
      }

      return (obj);
    }

    void PushBack() {
      obstacle_list_.obstacle_num++;
      if (obstacle_list_.obstacle_num >
          ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM) {
        obstacle_list_.obstacle_num =
            ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM;
      }
    }

    void PopBack() {
      obstacle_list_.obstacle_num--;
      if (obstacle_list_.obstacle_num < 0) {
        obstacle_list_.obstacle_num = 0;
      }
    }

    const ad_msg::ObstacleCameraList& GetObjList() const {
      return (obstacle_list_);
    }

    ad_msg::ObstacleCameraList obstacle_list_;
  };

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_;
  boost::thread thread_recv_;

  // 前视一体机标定参数
  common::Matrix<Float32_t, 3, 3> maxieye_mat_calibration_;
  // 环视相机标定参数
  common::Matrix<Float32_t, 3, 3> visual_control_mat_calibration_;

  ad_msg::Chassis chassis_;

  // 前视一体机和视觉控制器原始CANFD数据
  phoenix::can_dev::CanFdFrameList canfd_frame_list;

  // 前视一体机
  bool lane_list_ready_;
  LaneList lane_list_;
  bool lane_curb_list_ready_;
  LaneList lane_curb_list_;

  bool obj_list_ready_;
  ObjList obj_list_;

  bool traffic_signal_ready_;
  TraSignList traffic_signal_list_;

  // 视觉控制器
  // 视觉车道线
  bool visual_control_lane_list_ready_;
  LaneList visual_control_lane_list_;

  // 前摄像头
  bool front_obj_list_ready_;
  ObjList front_obj_list_;

  // 前左摄像头
  bool front_left_obj_list_ready_;
  ObjList front_left_obj_list_;

  // 前右摄像头
  bool front_right_obj_list_ready_;
  ObjList front_right_obj_list_;

  // 后左摄像头
  bool rear_left_obj_list_ready_;
  ObjList rear_left_obj_list_;

  // 后右摄像头
  bool rear_right_obj_list_ready_;
  ObjList rear_right_obj_list_;

  // 后摄像头
  bool rear_obj_list_ready_;
  ObjList rear_obj_list_;
};


}  // namespace sensor
}  // namespace phoenix


#endif  // PHOENIX_SENSOR_TASK_RECV_MAXIEYE_DATA_H_
