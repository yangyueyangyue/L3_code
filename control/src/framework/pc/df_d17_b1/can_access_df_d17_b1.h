#ifndef PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B1_H_
#define PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B1_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "common/task.h"
#include "common/message.h"

#include "can_dev/kvaser/can_driver_kvaser.h"
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"


namespace phoenix {
namespace framework {
namespace df_d17_b1 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)

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


class CanAccessDfD17B1 : public Task {
public:
  explicit CanAccessDfD17B1(Task* manager);
  ~CanAccessDfD17B1();


  bool Start();
  bool Stop();

  // 转向下发 20ms
  void SendFrame0x412();
  // 转向下发 10ms
  void SendFrame0x413(const ChassisCtlCmd_t& cmd, Uint8_t ctl_mode);
  // 制动下发 20ms
  void SendFrame0x0C040B9E(const ChassisCtlCmd_t& cmd, Uint8_t enable);
  // 油门/档位下发 10ms
  void SendFrame0x0CFF659E(const ChassisCtlCmd_t& cmd, Uint8_t gear_mode);
  // 上电报文 20ms
  void SendFrame0x0CFF649E(Uint8_t enable, Uint8_t engage);

private:
  void ThreadReceivingChannel1();
  void ThreadReceivingChannel2();
  void ThreadReceivingChannel3();
  void ThreadReceivingChannel4();
  void ParseCanFrame(const can_dev::CanFrame& frame);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_can_device_;
//#if ENABLE_CAN_DEV_KVASER
//  // Steering
//  can_dev::CanDriverKvaser can_channel_0_;
//  // EBS
//  can_dev::CanDriverKvaser can_channel_1_;
//  // Engine
//  can_dev::CanDriverKvaser can_channel_2_;
//#elif ENABLE_CAN_DEV_ZLGCANNET
  // P can
  can_dev::CanDriverZlgCanNet can_channel_4_;
  // B can
  can_dev::CanDriverZlgCanNet  can_channel_5_;
  // C can
  can_dev::CanDriverZlgCanNet can_channel_6_;
  // EHPS can
  can_dev::CanDriverZlgCanNet can_channel_7_;
//#endif
  boost::thread thread_can_recv_channel_4_;
  boost::thread thread_can_recv_channel_5_;
  boost::thread thread_can_recv_channel_6_;
  boost::thread thread_can_recv_channel_7_;
  boost::mutex lock_parse_can_frame_;

  CalcVehAcceleration calc_veh_acceleration_;
  Float32_t steering_wheel_angle_offset_ = 0;

  Int32_t frame_alive_count_0x413_ = 0;
  Int32_t frame_alive_count_0x0CFF649E_ = 0;
  Int32_t frame_alive_count_steering_control_3_ = 0;
  Int32_t frame_alive_count_xbr_brake_req_ = 0;
};


#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)


}  // namespace df_d17_b1
}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_DF_D17_CAN_ACCESS_DF_D17_B1_H_
