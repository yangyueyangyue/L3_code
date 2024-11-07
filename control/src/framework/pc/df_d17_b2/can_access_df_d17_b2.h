#ifndef PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B2_H_
#define PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B2_H_
//
#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "utils/linear_interpolation_c.h"
#include "common/task.h"
#include "common/message.h"
#include "common_c/chassis_control_c.h"

#include "can_dev/kvaser/can_driver_kvaser.h"
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"



namespace phoenix {
namespace framework {
namespace df_d17_b2 {

#define CAN_ACCESS_DF_D17_B2_USING_KVASER (1)


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)

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

class CanAccessDfD17B2 : public Task {
public:
  explicit CanAccessDfD17B2(Task* manager);
  ~CanAccessDfD17B2();

  /**
   * @brief 配置模块参数
   */
  void Configurate(const ChassisControlConfig_t& conf);


  bool Start();
  bool Stop();

  void SendRetransmissionCanFrames();

  // 上电报文 20ms
  void SendFrame0x0CFF649E(Uint8_t enable, Uint8_t engage);
  void SendFrame0x0CFF64DC(Uint8_t enable, Uint8_t engage);
  // 油门/档位下发 10ms
  void SendFrame0x0CFF659E(const ChassisCtlCmd_t& cmd, Uint8_t gear_mode);
  // 制动下发 20ms
  void SendFrame0x0C040B9E(const ChassisCtlCmd_t& cmd, Uint8_t enable);
  void SendFrame0x0C040BDC();
  // 转向下发 20ms
  void SendFrame0x0CFF799E(
      const ChassisCtlCmd_t& cmd, Uint8_t ctl_mode,
      const Chassis_t& status, const SpecialChassisInfo_t& special_chassis);
  void SendFrame0x0CFF79DC();

  void SendFrame0x18FF769E();
  void SendFrame0x18FF76DC();

private:
  void ThreadReceivingChannel0();
  void ThreadReceivingChannel1();
  void ThreadReceivingChannel2();
  void ThreadReceivingChannel3();
  void ParseCanFrame(const can_dev::CanFrame& frame, int channel);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_can_device_;
#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  // P can
  can_dev::CanDriverKvaser can_channel_0_;
  // B can
  can_dev::CanDriverKvaser can_channel_1_;
  // C can
  can_dev::CanDriverKvaser can_channel_2_;
  // EHPS can
  can_dev::CanDriverKvaser can_channel_3_;
#else
  // P can
  can_dev::CanDriverZlgCanNet can_channel_0_;
  // B can
  can_dev::CanDriverZlgCanNet can_channel_1_;
  // C can
  can_dev::CanDriverZlgCanNet can_channel_2_;
  // EHPS can
  can_dev::CanDriverZlgCanNet can_channel_3_;
#endif
  boost::thread thread_can_recv_channel_0_;
  boost::thread thread_can_recv_channel_1_;
  boost::thread thread_can_recv_channel_2_;
  boost::thread thread_can_recv_channel_3_;
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
  Button button_stop_adas_;

  // log
  common::LogFile log_file_steering_cmd_;
  bool open_log_file_steering_cmd_ = false;
  Char_t str_buff_steering_cmd_[1024*2];
};


#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)


}  // namespace df_d17_b2
}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B2_H_
