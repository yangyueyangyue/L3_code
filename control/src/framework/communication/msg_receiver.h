/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
#define PHOENIX_FRAMEWORK_MSG_RECEIVER_H_


#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include <std_msgs/ByteMultiArray.h>
#include "communication/ros_node.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"

#if ENTER_PLAYBACK_MODE_ADHMI
#include "control/CanFrame.h"
#include "can_dev/can_driver.h"
#endif

namespace phoenix {
namespace framework {

#if ENTER_PLAYBACK_MODE_ADHMI
 class Button {
  public:
    Button() {
      Clear();
    }

    void Clear() {
      init_flag_ = false;
      prev_status_ = 0;
      button_on_count_ = 0;
      button_off_count_ = 0;
      button_filter_count_ = 0;
    }

    void Update(Int32_t status, bool* rising_edge, bool* falling_edge) {
      *rising_edge = false;
      *falling_edge = false;
      if (!init_flag_) {
        init_flag_ = true;
        prev_status_ = status;
        return;
      }

      if ((0 == prev_status_) && (1 == status)) {
        button_on_count_ = 1;
        button_filter_count_ = 0;
      }
      if ((1 == prev_status_) && (0 == status)) {
        button_off_count_ = 1;
        button_filter_count_ = 0;
      }
      if ((button_on_count_ > 0) && (1 == status)) {
        button_on_count_++;
        if (button_on_count_ > 3) {
          button_on_count_ = 0;
          *rising_edge = true;
        }
      }
      if ((button_off_count_ > 0) && (0 == status)) {
        button_off_count_++;
        if (button_off_count_ > 3) {
          button_off_count_ = 0;
          *falling_edge = true;
        }
      }

      button_filter_count_++;
      if (button_filter_count_ > 5) {
        button_filter_count_ = 5;
        if (button_on_count_ > 0) {
          button_on_count_ = 0;
        }
        if (button_off_count_ > 0) {
          button_off_count_ = 0;
        }
      }

      prev_status_ = status;
    }

  private:
    bool init_flag_;
    Int32_t prev_status_;
    Int32_t button_on_count_;
    Int32_t button_off_count_;
    Int32_t button_filter_count_;
  };

class CalcVehAcceleration {
public:
  CalcVehAcceleration() {
    valid_ = false;
    velocity_ = 0;
    prev_acceleration_ = 0;
    common::com_memset(acceleration_filter_, 0, sizeof(acceleration_filter_));
  }

  void Reset() {
    valid_ = false;
    velocity_ = 0;
    prev_acceleration_ = 0;
    common::com_memset(acceleration_filter_, 0, sizeof(acceleration_filter_));
  }

  Float32_t Compute(float v) {
    Float32_t acceleration = 0.0f;
    if (valid_) {
      Float32_t elapsed = timer_.Elapsed();
      if ((200 <= elapsed) && (elapsed <= 400)) {
        acceleration = (v - velocity_) / elapsed * 1000;

        acceleration_filter_[2] = acceleration_filter_[1];
        acceleration_filter_[1] = acceleration_filter_[0];
        acceleration = 0.1f*acceleration_filter_[2] +
            0.4f*acceleration_filter_[1] + 0.5f*acceleration;
        acceleration_filter_[0] = acceleration;

        velocity_ = v;
        prev_acceleration_ = acceleration;
        timer_.Restart();
      } else if (elapsed > 400) {
        velocity_ = v;
        prev_acceleration_ = 0;
        timer_.Restart();
      } else {
        acceleration = prev_acceleration_;
      }
    } else {
      velocity_ = v;
      valid_ = true;
      prev_acceleration_ = 0;
      timer_.Restart();
    }

    return (acceleration);
  }

private:
  bool valid_;
  Float32_t velocity_;
  Float32_t prev_acceleration_;
  Float32_t acceleration_filter_[3];
  common::Stopwatch timer_;
};

class CalcSteeringSpeed {
public:
  CalcSteeringSpeed() {
    valid_ = false;
    steering_angle_ = 0;
    prev_speed_ = 0;
    common::com_memset(speed_filter_, 0, sizeof(speed_filter_));
  }

  void Reset() {
    valid_ = false;
    steering_angle_ = 0;
    prev_speed_ = 0;
    common::com_memset(speed_filter_, 0, sizeof(speed_filter_));
  }

  Float32_t Compute(float angle) {
    Float32_t speed = 0.0f;
    if (valid_) {
      Float32_t elapsed = timer_.Elapsed();
      if ((200 <= elapsed) && (elapsed <= 400)) {
        speed = (angle - steering_angle_) / elapsed * 1000;

        speed_filter_[2] = speed_filter_[1];
        speed_filter_[1] = speed_filter_[0];
        speed = 0.1f*speed_filter_[2] +
            0.4f*speed_filter_[1] + 0.5f*speed;
        speed_filter_[0] = speed;

        steering_angle_ = angle;
        prev_speed_ = speed;
        timer_.Restart();
      } else if (elapsed > 400) {
        steering_angle_ = angle;
        prev_speed_ = 0;
        timer_.Restart();
      } else {
        speed = prev_speed_;
      }
    } else {
      steering_angle_ = angle;
      valid_ = true;
      prev_speed_ = 0;
      timer_.Restart();
    }

    return (speed);
  }

private:
  bool valid_;
  Float32_t steering_angle_;
  Float32_t prev_speed_;
  Float32_t speed_filter_[3];
  common::Stopwatch timer_;
};

#endif

class MsgReceiver : public Task {
public:
  MsgReceiver(Task* manager);

#if (ENABLE_ROS_NODE)
  void SetRosNode(RosNode* node) {
    ros_node_ = node;
  }
#endif
#if (ENABLE_LCM_NODE)
  void SetLcmNode(LcmNode* node) {
    lcm_node_ = node;
  }
#endif
#if (ENABLE_UDP_NODE)
  void SetUdpNode(UdpNode* node) {
    udp_node_ = node;
  }
#endif

  bool Start();

#if (ENABLE_UDP_NODE)
  void HandleImuMessageUdp(const void *buf, Int32_t buf_len);
#if ENTER_PLAYBACK_MODE
  void HandleChassisMessageUdp(const void *buf, Int32_t buf_len);
#endif
  void HandlePlanningResultMessageUdp(const void *buf, Int32_t buf_len);
#endif

private:
#if (ENABLE_ROS_NODE)
  void HandleImuMessageRos(const std_msgs::ByteMultiArray& msg);
#if ENTER_PLAYBACK_MODE
  void HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg);
#endif
  void HandlePlanningResultMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleDFCVSpecialChassisInfoMessageRos(const std_msgs::ByteMultiArray& msg);

#if ENTER_PLAYBACK_MODE_ADHMI
  void HandleADHMICANPResultMessageRos(const ::control::CanFrame& msg);
  void HandleADHMICANBResultMessageRos(const ::control::CanFrame& msg);
  void HandleADHMICANCResultMessageRos(const ::control::CanFrame& msg);
  void HandleADHMICANRResultMessageRos(const ::control::CanFrame& msg);

  void ParseCanFrame(const can_dev::CanFrame& frame, int channel);
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::mutex lock_parse_can_frame_;

  CalcVehAcceleration calc_veh_acceleration_;
  CalcSteeringSpeed calc_steering_speed_;
  Float32_t steering_wheel_angle_offset_ = 0;

  // count
  Int32_t frame_alive_count_0x0CFF649E_ = 0;
  Int32_t frame_alive_count_0x0CFF64DC_ = 0;
  Int32_t frame_alive_count_0x0C040B9E_ = 0;
  Int32_t frame_alive_count_0x0C040BD7_ = 0;
  Int32_t frame_alive_count_0x0CFF799E_ = 0;
  Int32_t frame_alive_count_0x0CFF79DC_ = 0;

  struct {
    boost::mutex mutex;

    can_dev::CanFrame frame_0x18FEF131;
    can_dev::CanFrame frame_0x0CF00400;
    can_dev::CanFrame frame_0x18FED917;
    can_dev::CanFrame frame_0x18FEAE30;
  } retransmission_can_frames_;

  Button button_start_adas_;
#endif

#endif

#if (ENABLE_LCM_NODE)
  void HandleImuMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
#if ENTER_PLAYBACK_MODE
  void HandleChassisMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
#endif
  void HandlePlanningResultMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
#endif

private:
#if (ENABLE_ROS_NODE)
  RosNode* ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  LcmNode* lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  UdpNode* udp_node_;
#endif

#if (ENABLE_ROS_NODE)
  Imu_t ros_imu_info_;
#if ENTER_PLAYBACK_MODE
  Chassis_t ros_chassis_info_;
#endif
  PlanningResult_t ros_planning_result_;
  DFCV_SpecialChassisInfo_t ros_DFCV_special_chassis_info_;
#endif

#if (ENABLE_LCM_NODE)
  Imu_t lcm_imu_info_;
#if ENTER_PLAYBACK_MODE
  Chassis_t lcm_chassis_info_;
#endif
  PlanningResult_t lcm_planning_result_;
#endif

#if (ENABLE_UDP_NODE)
  Imu_t udp_imu_info_;
#if ENTER_PLAYBACK_MODE
  Chassis_t udp_chassis_info_;
#endif
  PlanningResult_t udp_planning_result_;
#endif
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
