//
#include "pc/df_d17_b2/can_access_df_d17_b2.h"

#include "container/ring_buffer_c.h"
#include "utils/com_clock_c.h"

#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "utils/linear_interpolation_c.h"
#include "math/math_utils_c.h"
#include "communication_c/shared_data_c.h"

#define ENABLE_LOG_FILE_STEERING_CMD (1)

namespace phoenix {
namespace framework {
namespace df_d17_b2 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)

#define CRC_POLYNOM 0xDF
static uint8_t CalculateCrc8(const uint8_t *addr){
  uint8_t checkSum = 0xFF;
  int byteIndex;
  int bitIndex;

  for ( byteIndex = 1; byteIndex < 8; byteIndex++){
    if(byteIndex != 8){
      checkSum ^= addr[byteIndex];
      for(bitIndex = 0; bitIndex < 8; bitIndex++){
        if((checkSum & 0x80) != 0){
          checkSum = (uint8_t)(uint8_t)(checkSum << 1) ^ CRC_POLYNOM;
        }else{
          checkSum = (uint8_t)(checkSum << 1);
        }
      }
    }
  }
  return  (uint8_t)(~checkSum);
}


// 用来平滑车辆的质量
typedef struct _WeightFilter_t WeightFilter_t;
struct _WeightFilter_t {
  Int64_t timestamp;
  Float32_t avg_weight;

  Float32_t data_buffer[20];
  RingBuffer_t weight_buff;
};
static WeightFilter_t s_gross_weight_filter;

static void WeightFilter_Init(WeightFilter_t* ins) {
  ins->timestamp = Phoenix_Common_GetClockNowMs();
  ins->avg_weight = 0;

  phoenix_com_memset(ins->data_buffer, 0, sizeof(ins->data_buffer));
  Phoenix_Com_RingBuffer_Init(
      &(ins->weight_buff), 20, sizeof(Float32_t), ins->data_buffer);
}

static Float32_t WeightFilter_Smooth(WeightFilter_t* ins, Float32_t w) {
  // Convert kg to t.
  Float32_t weight = w / 1000.0F;
  Int64_t cur_timestamp = Phoenix_Common_GetClockNowMs();
  Int64_t elapsed = 0;

  if (Phoenix_Com_RingBuffer_IsEmpty(&(ins->weight_buff))) {
    ins->avg_weight = weight;
    ins->timestamp = cur_timestamp;

    Phoenix_Com_RingBuffer_PushBack(&(ins->weight_buff), &weight);
  } else {
    elapsed = Phoenix_Common_CalcElapsedClockMs(ins->timestamp, cur_timestamp);

    if (1000 <= elapsed) {
      Int32_t count = 0;
      Float32_t prev_weight = 0.0F;
      RingBufferIterator_t it_begin;
      RingBufferIterator_t it_end;

      ins->timestamp = cur_timestamp;

      prev_weight =
          *(Float32_t*)Phoenix_Com_RingBuffer_GetBack(&(ins->weight_buff));
#if 0
      if (weight > 1.1F*prev_weight) {
        // printf("limit weight from %0.1f to %0.1f\n", weight*1000, 1.1F*prev_weight*1000);
        weight = 1.1F*prev_weight;
      } else if (weight < 0.9F*prev_weight) {
        // printf("limit weight from %0.1f to %0.1f\n", weight*1000, 0.9F*prev_weight*1000);
        weight = 0.9F*prev_weight;
      } else {
        // nothing to do
      }
#endif
      Phoenix_Com_RingBuffer_PushBackOverride(&(ins->weight_buff), &weight);

      ins->avg_weight = 0.0F;
      Phoenix_Com_RingBuffer_SetItToBegin(&(ins->weight_buff), &it_begin);
      Phoenix_Com_RingBuffer_SetItToEnd(&(ins->weight_buff), &it_end);
      while (Phoenix_Com_RingBufferIterator_IsNotEqual(&it_begin, &it_end)) {
        ins->avg_weight +=
            *(Float32_t*)Phoenix_Com_RingBufferIterator_GetCurrent(&it_begin);

        //printf("count=%d, weight=%0.1f\n", count,
        //       *(Float32_t*)Phoenix_Com_RingBufferIterator_GetCurrent(&it_begin)*1000);

        count++;
        Phoenix_Com_RingBufferIterator_Increase(&it_begin);
      }
      ins->avg_weight /= count;
      weight = ins->avg_weight;
      // printf("return avg_weight: %0.1f, count: %d\n", weight*1000, count);
    } else {
      weight = ins->avg_weight;
      // printf("return prev weight: %0.1f\n", weight*1000);
    }
  }

  return (weight*1000.0F);
}

CanAccessDfD17B2::CanAccessDfD17B2(Task* manager)
  : Task(TASK_ID_CAN_ACCESS, "Can Access", manager),
    log_file_steering_cmd_("steering_cmd") {
  running_flag_can_device_ = false;

  steering_wheel_angle_offset_ = common::com_deg2rad(0.0F);

  WeightFilter_Init(&s_gross_weight_filter);

#if ENABLE_LOG_FILE_STEERING_CMD
  open_log_file_steering_cmd_ = false;
#endif
}

CanAccessDfD17B2::~CanAccessDfD17B2() {
  Stop();
}

void CanAccessDfD17B2::Configurate(const ChassisControlConfig_t& conf) {
  steering_wheel_angle_offset_ = conf.steering_wheel_angle_offset;

  LOG_INFO(3) << "CanAccessDfD17B2: Set steering wheel angle offset to "
              << common::com_rad2deg(conf.steering_wheel_angle_offset)
              << "deg.";
}

bool CanAccessDfD17B2::Start() {
  if (running_flag_can_device_) {
    return (true);
  }

  LOG_INFO(3) << "Open CAN Device...";
#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  if (!can_dev::CanDriverKvaser::OpenDevice()) {
    LOG_ERR << "Open CAN device failed.";
    return (false);
  }
#else
  if (!can_dev::CanDriverZlgCanNet::OpenDevice()) {
    LOG_ERR << "Open CAN device failed.";
    return (false);
  }
#endif

#if 1
  Uint16_t base_port = 4001;
  Uint16_t base_notify_port = 6001;
#else
  Uint16_t base_port = 4005;
  Uint16_t base_notify_port = 6005;
#endif

  can_dev::CanChannelParam can_dev_param;
#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  can_dev_param.channel = 0;
#else
  can_dev_param.channel = 0;
#endif
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = base_port;
  can_dev_param.can_net.notify_port = base_notify_port;
  if (!can_channel_0_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_1 failed.";
    return false;
  }

#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  can_dev_param.channel = 1;
#else
  can_dev_param.channel = 1;
#endif
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = base_port + 1;
  can_dev_param.can_net.notify_port = base_notify_port + 1;
  if (!can_channel_1_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_2 failed.";
    goto ERROR_01;
  }

#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  can_dev_param.channel = 2;
#else
  can_dev_param.channel = 2;
#endif
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = base_port + 2;
  can_dev_param.can_net.notify_port = base_notify_port + 2;
  if (!can_channel_2_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_3 failed.";
    goto ERROR_02;
  }

#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  can_dev_param.channel = 3;
#else
  can_dev_param.channel = 3;
#endif
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = base_port + 3;
  can_dev_param.can_net.notify_port = base_notify_port + 3;
  if (!can_channel_3_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_4 failed.";
    goto ERROR_03;
  }

  LOG_INFO(3) << "Start CAN Message Receive Thread (channel 0 & 1 & 2)...";
  running_flag_can_device_ = true;
  thread_can_recv_channel_0_ = boost::thread
      (boost::bind(&CanAccessDfD17B2::ThreadReceivingChannel0, this));
  thread_can_recv_channel_1_ = boost::thread
      (boost::bind(&CanAccessDfD17B2::ThreadReceivingChannel1, this));
  thread_can_recv_channel_2_ = boost::thread
      (boost::bind(&CanAccessDfD17B2::ThreadReceivingChannel2, this));
  thread_can_recv_channel_3_ = boost::thread
      (boost::bind(&CanAccessDfD17B2::ThreadReceivingChannel3, this));

  frame_alive_count_0x0CFF649E_ = 0;
  frame_alive_count_0x0CFF64DC_ = 0;
  frame_alive_count_0x0C040B9E_ = 0;
  frame_alive_count_0x0C040BD7_ = 0;
  frame_alive_count_0x0CFF799E_ = 0;
  frame_alive_count_0x0CFF79DC_ = 0;
  
  LOG_INFO(3) << "Open CAN Device... [OK]";

  return (true);


ERROR_03:
  can_channel_2_.CloseChannel();
ERROR_02:
  can_channel_1_.CloseChannel();
ERROR_01:
  can_channel_0_.CloseChannel();
#if CAN_ACCESS_DF_D17_B2_USING_KVASER
  can_dev::CanDriverKvaser::CloseDevice();
#else
  can_dev::CanDriverZlgCanNet::CloseDevice();
#endif

  LOG_INFO(3) << "Open CAN Device... [NG]";

  return (false);
}

bool CanAccessDfD17B2::Stop() {
  if (running_flag_can_device_) {
    running_flag_can_device_ = false;

    bool ret = thread_can_recv_channel_3_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread "
                 "\"CAN Receiving (channel 7)\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 7) ... [OK]";
    } else {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 7) ... [NG]";
    }

    ret = thread_can_recv_channel_2_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread "
                 "\"CAN Receiving (channel 6)\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 6) ... [OK]";
    } else {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 6) ... [NG]";
    }

    ret = thread_can_recv_channel_1_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread "
                 "\"CAN Receiving (channel 5)\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 5) ... [OK]";
    } else {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 5) ... [NG]";
    }

    ret = thread_can_recv_channel_0_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread "
                 "\"CAN Receiving (channel 4)\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 4) ... [OK]";
    } else {
      LOG_INFO(3) << "Stop CAN Receiving Thread (channel 4) ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Device.";
    can_channel_3_.CloseChannel();
    can_channel_2_.CloseChannel();
    can_channel_1_.CloseChannel();
    can_channel_0_.CloseChannel();
#if CAN_ACCESS_DF_D17_B2_USING_KVASER
    can_dev::CanDriverKvaser::CloseDevice();
#else
    can_dev::CanDriverZlgCanNet::CloseDevice();
#endif
  }

  return (true);
}

void CanAccessDfD17B2::ThreadReceivingChannel0() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 1) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_0_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame, 0);

      //printf("channel 0 receiving id[%08X] data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
      //       can_frame.id,
      //       can_frame.data[0], can_frame.data[1], can_frame.data[2], can_frame.data[3],
      //       can_frame.data[4], can_frame.data[5], can_frame.data[6], can_frame.data[7]);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_0);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 1) ... [Stopped]";
}

void CanAccessDfD17B2::ThreadReceivingChannel1() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 2) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_1_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame, 1);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_1);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 2) ... [Stopped]";
}

void CanAccessDfD17B2::ThreadReceivingChannel2() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_2_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame, 2);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_2);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Stopped]";
}

void CanAccessDfD17B2::ThreadReceivingChannel3() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 4) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_3_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame, 3);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_3);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Stopped]";
}

void CanAccessDfD17B2::ParseCanFrame(const can_dev::CanFrame& frame, int channel) {
  Lock lock(lock_parse_can_frame_);

  Chassis_t chassis;
  SpecialChassisInfo_t special_chassis;
  Phoenix_SharedData_GetChassis(&chassis);
  Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  DFCV_SpecialChassisInfo_t dfcv_special_chassis;
  Phoenix_SharedData_GetDFCVSpecialChassisInfo(&dfcv_special_chassis);
#endif

  static Uint8_t SteeringOverridePS = 0x00;

  const Uint8_t* data = frame.data;

  switch (frame.id) {

  case (0x18FF6931):{
    //  VECU状态  0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
    special_chassis.df_d17.VECU_state = (data[0] & 0x03);
    // std::cout<<"VECU状态 = "<<(int)special_chassis.df_d17.VECU_state<<std::endl;
    chassis.acc_pedal_value = data[2] * 0.4;
    break;
  }
  case (0x18FF6832):{
    //  BCM BCM状态  0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
    special_chassis.df_d17.BCM_state = ((data[0] >> 4)& 0x03);
    // std::cout<<" BCM状态 = "<<(int)special_chassis.df_d17.BCM_state<<std::endl;
    //转向拨杆信号反馈
    int8_t TurnSwitch_Left = (data[3] >> 4) & 0x03;
    int8_t TurnSwitch_Right = (data[3] >> 6) & 0x03;

    if(0x01 == TurnSwitch_Left && 0x00 == TurnSwitch_Right) {
      chassis.signal_turning_indicator = VEH_TURNING_INDICATOR_LEFT;
    } else if(0x00 == TurnSwitch_Left && 0x01 == TurnSwitch_Right) {
      chassis.signal_turning_indicator = VEH_TURNING_INDICATOR_RIGHT;
    } else if(0x00 == TurnSwitch_Left && 0x00 == TurnSwitch_Right) {
      chassis.signal_turning_indicator = VEH_TURNING_INDICATOR_NONE;
    } else {
      chassis.signal_turning_indicator = VEH_TURNING_INDICATOR_INVALID;
    }

    break;
  }
  case (0x18FEBF0B): {
    if (2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }

    // EBS系统：EBC2-轮速信息, 50ms
    // 整车速度,精度0.00390625，单位有km/h -> m/s, FE00h-错误指示, FFFFh-无效
    float front_axle_velocity = (float)(((data[0]) | (data[1] << 8)) * 0.00390625)  / 3.6;
    if(front_axle_velocity!= 0xFAFF && front_axle_velocity != 0xFFFF) {
      chassis.v = front_axle_velocity;
      chassis.v_valid = 1;
      chassis.a = calc_veh_acceleration_.Compute(front_axle_velocity);
      chassis.a_valid = 1;
    } else {
      chassis.v_valid = 0;
      chassis.a_valid = 0;
      calc_veh_acceleration_.Reset();
    }

    // 前轴左轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double front_axle_left_wheel_relative_velocity =
        (frame.data[2] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[2] == 0xFE || frame.data[2] == 0xFF) {
      chassis.wheel_speed_fl_valid = false;
      chassis.wheel_speed_fl = 0;
    } else {
      chassis.wheel_speed_fl_valid = true;
      chassis.wheel_speed_fl = front_axle_left_wheel_relative_velocity;
    }
    // 前轴右轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double front_axle_right_wheel_relative_velocity =
        (frame.data[3] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[3] == 0xFE || frame.data[3] == 0xFF) {
      chassis.wheel_speed_fr_valid = false;
      chassis.wheel_speed_fr = 0;
    } else {
      chassis.wheel_speed_fr_valid = true;
      chassis.wheel_speed_fr = front_axle_right_wheel_relative_velocity;
    }
    // #1后轴左轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double first_rear_axle_left_wheel_relative_velocity =
        (frame.data[4] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[4] == 0xFE || frame.data[4] == 0xFF) {
      chassis.wheel_speed_rl_valid = false;
      chassis.wheel_speed_rl = 0;
    } else {
      chassis.wheel_speed_rl_valid = true;
      chassis.wheel_speed_rl = first_rear_axle_left_wheel_relative_velocity;
    }
    // #1后轴右轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double first_rear_axle_right_wheel_relative_velocity =
        (frame.data[5] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[5] == 0xFE || frame.data[5] == 0xFF) {
      chassis.wheel_speed_rr_valid = false;
      chassis.wheel_speed_rr = 0;
    } else {
      chassis.wheel_speed_rr_valid = true;
      chassis.wheel_speed_rr = first_rear_axle_right_wheel_relative_velocity;
    }
    // #2后轴左轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double second_rear_axle_left_wheel_relative_velocity =
        (frame.data[6] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[6] == 0xFE || frame.data[6] == 0xFF) {
      chassis.wheel_speed_rl2_valid = false;
      chassis.wheel_speed_rl2 = 0;
    } else {
      chassis.wheel_speed_rl2_valid = true;
      chassis.wheel_speed_rl2 = second_rear_axle_left_wheel_relative_velocity;
    }
    // #2后轴右轮相对速度, 1/16km/h/bit, (-7.8125)-(+7.8125)km/h,
    // FEh-错误指示, FFh-无效或暂未安装
    double second_rear_axle_right_wheel_relative_velocity =
        (frame.data[7] * 0.0625 - 7.8125) * 0.27777777778;
    if (frame.data[7] == 0xFE || frame.data[7] == 0xFF) {
      chassis.wheel_speed_rr2_valid = false;
      chassis.wheel_speed_rr2 = 0;
    } else {
      chassis.wheel_speed_rr2_valid = true;
      chassis.wheel_speed_rr2 = second_rear_axle_right_wheel_relative_velocity;
    }
    break;
  }

  case (0x18F0010B): {
    if(2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }

    // EBS系统：EBC1-电子制动控制器，100ms
    // ASR发动机控制激活状态, 00-ASR发动机控制未激活但已安装, 01-ASR发动机控制激活, 10-预留状态, 11-不采取行动
    unsigned char asr_engine_control_active_status = data[0] & 0x03;

    // ASR制动控制激活状态, 00-ASR制动控制未激活但已安装, 01-ASR制动控制激活, 10-预留状态, 11-不采取行动
    unsigned char asr_brake_control_active_status = (data[0] >> 2) & 0x03;

    // ABS激活状态, 00-ABS未激活但已安装, 01-ABS激活, 10-预留状态, 11-不采取行动
    unsigned char abs_active_status = (data[0] >> 4) & 0x03;
    if(0x01 == abs_active_status) {
      // info->absStatus = 0x01;
    } else {
      // info->absStatus = 0;
    }

    // EBS制动开关, 00-制动踏板未被踩下, 01-制动踏板踩下, 10-错误指示, 11-无效或暂未安装
    unsigned char ebs_brake_switch_status = (data[0] >> 6) & 0x03;
    if(0x01 == ebs_brake_switch_status) {
      // 制动踏板位置, 0.4%/bit, 0-100%, FEh-错误指示, FFh-无效或暂未安装
      float brake_pedal_position = data[1] * 0.4f;
      if (data[1] == 0xFF || data[1] == 0xFE) {
        chassis.brake_pedal_value = 0;
      } else {
        chassis.brake_pedal_value = brake_pedal_position;
      }
    } else {
      chassis.brake_pedal_value = 0;
    }

    // ABS Offroad开关, 00-ABS Offroad开关未打开, 01-ABS Offroad开关打开, 10-错误指示, 11-无效或暂未安装
    unsigned char abs_offroad_switch = data[2] & 0x03;

    // ASR Offroad开关, 00-ASR Offroad开关未打开, 01-ASR Offroad开关打开, 10-错误指示, 11-无效或暂未安装
    unsigned char asr_offroad_switch = (data[2] >> 2) & 0x03;

    // ASR"hill holder"开关, 00-AS"hill holder"开关未打开, 01-ASR"hill holder"开关打开, 10-错误指示, 11-无效或暂未安装
    unsigned char asr_hill_holder_switch = (data[2] >> 4) & 0x03;

    // 油门互锁开关, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char accelerator_interlock_switch = data[3] & 0x03;

    // 发动机derate开关, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char engine_derate_switch = (data[3] >> 2) & 0x03;

    // 备用发动机停机开关, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char spare_engine_stop_switch = (data[3] >> 4) & 0x03;

    // 远程油门启动开关, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char remote_accelerator_start_switch = (data[3] >> 6) & 0x03;

    // 发动机减速器选择, 0.4%/bit, 0-100%, FEh-错误指示, FFh-无效或暂未安装
    float engine_decelerator_select = data[4] * 0.4f;

    // ABS完全运行, 00-ABS未完全运行, 01-ABS完全运行, 10-错误指示, 11-无效或暂未安装
    unsigned char abs_fully_running = data[5] & 0x03;
    if((0x01 == abs_active_status) && (0x01 == abs_fully_running)) {
      // info->absStatus = 0x02;
    }

    // ABS/EBS黄色预警状态, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char abs_ebs_yellow_alarm_lamp = (data[5] >> 4) & 0x03;
    if(0x01 == abs_ebs_yellow_alarm_lamp) {
      // info->ebsStatus = 0x02;
    } else if(0x03 == abs_ebs_yellow_alarm_lamp) {
      // info->ebsStatus = 0x00;
    } else {
      // info->ebsStatus = 0x01;
    }

    // EBS红色警报灯状态, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char ebs_red_alarm_lamp = (data[5] >> 2) & 0x03;
    if(ebs_red_alarm_lamp == 0x01) {
      // info->ebsStatus = 0x03;
    } else if(0x03 == ebs_red_alarm_lamp) {
      // info->ebsStatus = 0x00;
    } else {
      // info->ebsStatus = 0x01;
    }

    // ATC/ASR指示灯状态, 00-关闭, 01-打开, 10-错误指示, 11-无效或暂未安装
    unsigned char atc_asr_indicator_lamp = (data[5] >> 6) & 0x03;

    // 制动控制控制器源地址, 1/bit, 0-253, FEh-错误指示, FFh-无效或暂未安装
    unsigned char brake_controller_source_address = data[6];

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    dfcv_special_chassis.source_address_brake_control_device = brake_controller_source_address;
#endif

    break;
  }

  case (0x18FE700B): {
    //EBS系统：CVW-车体质量信息，1000ms
    //牵引车头质量（Powered Vehicle Weight）,2kg/bit,0kg-128510kg,FEH-错误指示，FFH-无效
    float powered_vehicle_weight =(float)(((unsigned int)(data[0]) | (unsigned int)(data[1] << 8)) * 2.0);
    if (data[0] != 0xFE && data[0] != 0xFF) {
      // info->weight = powered_vehicle_weight;
    }

    //车体总质量（Gross Combination Weight）,10kg/bit,0kg-642550kg,FEH-错误指示，FFH-无效
    float gross_combination_weight =(float)(((unsigned int)(data[2])|(unsigned int)(data[3] << 8 ))* 10.0);

    if (data[2] == 0xFE || data[2] == 0xFF) {
      // vehicle_status_.VehicleWeightStatus = VEH_WEIGHT_STSTUS_INVALID;
      // vehicle_status_.GrossCombinationWeight = 0;
    } else {
      // vehicle_status_.VehicleWeightStatus = VEH_WEIGHT_STSTUS_VALID;
      // vehicle_status_.GrossCombinationWeight = gross_combination_weight;
      chassis.gross_weight_valid = true;
      chassis.gross_weight =
          WeightFilter_Smooth(&s_gross_weight_filter, gross_combination_weight);
      // printf("\n\n$$$$ gross_weight: %0.1f, after smooth %0.1f\n", gross_combination_weight, chassis.gross_weight);

      // 临时根据重量判断是否带挂
      if(12000.0F < chassis.gross_weight ) {
        chassis.trailer_status = VEH_TRAILER_STATUS_CONNECTED;
      } else {
        chassis.trailer_status = VEH_TRAILER_STATUS_NOT_CONNECTED;
      }

      // std::cout<<"gross_combination_weight = "<< gross_combination_weight<<std::endl;
      // std::cout<<"chassis.gross_weight = "<< chassis.gross_weight<<std::endl;
      // std::cout<<"chassis.trailer_status = "<< chassis.trailer_status<<std::endl;
      // std::cout<<std::endl;
    }

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    dfcv_special_chassis.vehicle_mass = gross_combination_weight;
#endif
    break;
  }

  case (0x18FDC40B): {
    // EBS系统：EBC5-电子制动控制器, 100ms
    // 制动器温度报警, 00-衰落警告未激活, 01-衰落警告激活, 10-预留状态, 11-无效或暂未安装
    unsigned char brake_temperature_alarm = data[0] & 0x03;

    // 驻车制动模式, 000-在工作, 001-已激活, 010-已激活，但未正常工作, 011-101未定义, 110-预留状态, 111-不关心
    unsigned char epb_mode = (data[0] >> 2) & 0x07;
    if (0x00 == epb_mode) {
      chassis.epb_status = VEH_EPB_STATUS_OFF;
    } else if (0x01 == epb_mode) {
      chassis.epb_status = VEH_EPB_STATUS_ON;
    } else {
      chassis.epb_status = VEH_EPB_STATUS_INVALID;
    }

    // 坡道保持模式, 000-在工作, 001-已激活, 010-已激活，但会在短时间内切换至工作状态, 011-101未定义, 110-预留状态, 111-不关心
    unsigned char hill_hold_mode = (data[0] >> 5) & 0x07;

    // 主制动器是否使用, 00-主制动器未使用, 01-主制动器已使用, 10-预留状态, 11-不关心
    unsigned char main_brake_is_using = data[1] & 0x03;

    // XBR系统状态, 00-制动系统全面运行，任何外部制动要求都会被接受, 01-只有最高优先级的外部制动要求会被接受, 10-所有外部制动要求都不会被接受, 11-未定义
    unsigned char xbr_status = (data[1] >> 2) & 0x03;

    // XBR激活控制模式, 0000-无制动要求被执行（默认模式）, 0001-驾驶员的制动要求被执行，无外部制动要求, 0010-执行XBR附加模式的加速度控制,
    // 0011-执行XBR最大模式的加速度控制, 0100-1110未定义, 1111-无效
    unsigned char xbr_active_control_mode = (data[1] >> 4) & 0x0F;

    // XBR加速度限制, 0.1m/s^2, (-10)-(+10)m/s^2
    float xbr_acceleration_limit = data[2] * 0.1f;

    break;
  }

  case (0x18FECA0B): {
    // EBS系统：DM1-诊断故障代码
    // 保护灯状态, 00-关闭, 01-打开, 10-预留状态, 11-不采取行动
    unsigned char guard_lamp = data[0] & 0x03;

    // 黄色警告灯状态, 00-关闭, 01-打开, 10-预留状态, 11-不采取行动
    unsigned char yellow_alarm_lamp = (data[0] >> 2) & 0x03;

    // 红色停车灯状态, 00-关闭, 01-打开, 10-预留状态, 11-不采取行动
    unsigned char red_stop_lamp = (data[0] >> 4) & 0x03;

    // 故障指示灯状态, 00-关闭, 01-打开, 10-预留状态, 11-不采取行动
    unsigned char fault_indicating_lamp = (data[0] >> 6) & 0x03;

    // 最少8位有意义的SPN码-DTC格式
    unsigned char spn_code_1 = data[2];

    // 第二字节SPN码-DTC路劲
    unsigned char spn_code_2 = data[3];

    // FMI
    unsigned char fmi_code = data[4] & 0x1F;

    // 最少3位有意义SPN码
    unsigned char spn_code_3 = (data[4] >> 5) & 0x07;

    // 发生数（OC）(7位数)
    unsigned char occurrence_count = data[5] & 0x7F;

    // 故障代码激活状态
    unsigned char fault_code_active = (data[5] >> 7) & 0x01;

    break;
  }

  case (0x18F0090B): {
    if (2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }

    // EBS: Vehicle Dynamic Stability Control #2: VDC2, Optional – can be enbabled/disabled by EoL
    // Yaw Rate
    double yaw_rate = ((unsigned int)(data[3]) | (unsigned int)(data[4] << 8)) / 8192.0 - 3.92;
    chassis.yaw_rate = yaw_rate;
    if (data[4] == 0xFE || data[4] == 0xFF) {
      // FExxh - Error Indicator, FFxxh - Not available
      chassis.yaw_rate_valid = 0;
    } else {
      chassis.yaw_rate_valid = 1;
    }

    //printf("can: yaw_rate_valid=%d, yaw_rate=%f\n",
    //       chassis.yaw_rate_valid, common::com_rad2deg(chassis.yaw_rate));
    // Lateral Acceleration
    double lat_accl = ((unsigned int)(data[5]) | (unsigned int)(data[6] << 8)) / 2048.0 - 15.687;
    if (data[6] == 0xFE || data[6] == 0xFF) {
      // FExxh - Error Indicator, FFxxh - Not available
    } else {
    }
    // Longitudinal Acceleration
    double lon_accl = data[7] * 0.1 - 12.5;
    if (data[7] == 0xFE || data[7] == 0xFF) {
      // FEh - Error Indicator, FFh - Not available
    } else {
    }

    break;
  }

  case (0x18F00503): {
    // AMT: ETC2-电子传动控制器2
    /// !!! 请解析Gear !!!
    // 选择档位, 1 gear value/bit,偏移:-125, -125, -125…+125
    
    signed char selected_gear = (signed char)(data[0] - 125);
    chassis.selected_gear_number = selected_gear;
    // 实际传动比, 0.001/bit,偏移:0, 0…64.255
    float transmission_ratio = ((unsigned int)(data[1]) | (unsigned int)(data[2] << 8)) * 0.001f;

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    dfcv_special_chassis.transmission_selected_gear = selected_gear;
    dfcv_special_chassis.current_gear_ratio = transmission_ratio;
#endif

    // 当前档位, 1 gear value/bit,偏移:-125, -125, -125…+125
    signed char current_gear = (signed char)(data[3] - 125);

    chassis.gear_number = current_gear;
    // "N 125,R1 124,D从126一直往上累加(1-14挡)"
    // zero is netral, 251(0xFB) is park.
    switch (current_gear) {
    //case -2:
    //    vehicle_status_.Gear = VEH_GEAR_R2;
    //    break;
    case -1: // 倒车档 R1
      chassis.gear = VEH_GEAR_R;
      break;
    case 0: // 空挡
      chassis.gear = VEH_GEAR_N;
      break;
    case 1: // 1-14档
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      chassis.gear = VEH_GEAR_D;
      break;
      // P档需要进行验证
    case 126: // 驻车档
      chassis.gear = VEH_GEAR_P;
      break;
    default:
      chassis.gear = VEH_GEAR_INVALID;
      break;
    }
    break;
  }

  case (0x0CF00300): {
    // 发动机: EEC2—电子发动机控制器
    // 加速踏板1低怠速开关
    unsigned char accelerator_pedal_low_idle_switch = data[0] & 0x03;

    // 加速踏板位置1, 0.4%, 0%, 0%~100%
    unsigned char accelerator_pedal_position = (unsigned char)(data[1] * 0.4f);
    // chassis.acc_pedal_value = accelerator_pedal_position;

    // 加速踏板强迫换挡开关
    unsigned char accelerator_pedal_enforce_shift_switch = data[2] & 0x3F;

    break;
  }

  case (0x18FEDF00): {
    // 发动机: EEC3—电子发动机控制器
    // 标称摩擦转矩, 1%, -125%, -125% ~ +125%
    float nominal_friction_torque = data[0] - 125.0f;

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    dfcv_special_chassis.nominal_fricton_troque_percent = nominal_friction_torque;

    float estimated_lossed_torque_percent = data[4] - 125.0f;

    dfcv_special_chassis.estimated_lossed_torque_percent = estimated_lossed_torque_percent;
#endif

    break;
  }

  case (0x18ECFF00): {
    // 发动机: EC1-发动机配置1
    // 发动机空转转速, 0.125 rpm/bit, 0 ~ 8,031.875 rpm
    float engine_idling_speed = ((unsigned int)(data[0]) | (unsigned int)(data[1] << 8)) * 0.125f;
    // 参考发动机转矩, 1Nm per bit, 0 ~ 64255 Nm

    break;
  }

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  case (0xCF00203): {
    //变速箱：ETC2
    unsigned char transmission_shift_status = (data[0] >> 4) & 0x3 ;
    unsigned char transmission_engage_status = data[0] & 0x3;
    dfcv_special_chassis.transmission_shift_status = transmission_shift_status;
    dfcv_special_chassis.transmission_engage_status = transmission_engage_status;

    Float32_t tcu_output_shaft_speed = (Int16_t)((Uint32_t)data[1] | (Uint32_t)(data[2] << 8)) / 8;
    dfcv_special_chassis.tcu_output_shaft_speed = tcu_output_shaft_speed;
    break;
  }

  case (0xC000003): {
    //变速箱：TSC1_TE_TCU
    unsigned char tcu_engine_control_mode = data[0] & 0x3;
    dfcv_special_chassis.tcu_engine_control_mode = tcu_engine_control_mode;
    break;
  }

  case (0x18FE4A03): {
    //变速箱：ETC7_TCU
    unsigned char trans_ready_for_brake_release = data[0] & 0x03;
    dfcv_special_chassis.trans_ready_for_brake_release = trans_ready_for_brake_release;
    break;
  }
#if 0
  case (0x18FED917): {
    //挂车状态
    unsigned char trailer_connected_status = (data[1] >> 6);
    chassis.trailer_connected_status = trailer_connected_status;
  }
  #endif
#endif
  
  case (0x18FF2632): {
    // 车身灯光控制（与车型有关）
    // info->turnLamp = 0;
    // info->brekeLamp = 0;
    unsigned char SignalLeft = data[2] & 0x03;
    unsigned char SignalRight = (data[2] >> 2) & 0x03;
    unsigned char AlarmLamp = (data[1] >> 6) & 0x03;
    unsigned char BrakeLamp = data[4] & 0x03;

    chassis.signal_turn_lamp = VEH_TURN_LAMP_OFF;
    chassis.signal_brake_lamp = VEH_LAMP_OFF;
    if (0x01 == SignalLeft) {
      chassis.signal_turn_lamp = VEH_TURN_LAMP_LEFT;
    }
    if (0x01 == SignalRight) {
      chassis.signal_turn_lamp = VEH_TURN_LAMP_RIGHT;
    }
    if (0x01 == AlarmLamp) {
      chassis.signal_turn_lamp = VEH_TURN_LAMP_EMERGENCY;
    }
    if (0x01 == BrakeLamp) {
      chassis.signal_brake_lamp = VEH_LAMP_ON;
    }
    break;
  }

 case (0x18FF7A13): {
    if (2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }

    // Steering angle, CW: minus, CCW: plus, -3276.8~3276.8 deg
    Float32_t steering_angle = (Int16_t)((Uint32_t)data[0] | (Uint32_t)(data[1] << 8)) * 0.1F;
    // Steering angle speed, -2000~2000 deg/s
    Float32_t steering_angle_speed = (Int16_t)((Uint32_t)(data[2]) | (Uint32_t)(data[3] << 8)) * 0.1F;
    chassis.steering_wheel_angle_valid = 1;
    chassis.steering_wheel_angle =
        common::com_deg2rad(steering_angle) - steering_wheel_angle_offset_;
    chassis.steering_wheel_speed_valid = 1;
#if 0
    chassis.steering_wheel_speed = common::com_deg2rad(steering_angle_speed);
#else
    chassis.steering_wheel_speed = calc_steering_speed_.Compute(chassis.steering_wheel_angle);
#endif

    break;
  }

  case (0x18FF7B13): {
    SteeringOverridePS = (Uint8_t)(data[3] & 0x0F);
    // std::cout<<"SteeringOverridePS == "<<(int)SteeringOverridePS<<std::endl;
    // if(SteeringOverridePS == 0x02) {
    //   chassis.eps_status = VEH_EPS_STATUS_MANUAL_INTERRUPT;
    // }

    special_chassis.df_d17.SteeringErrorEvent1ActivePS = (data[5] & 0x03);

    // 1 - ADU PS故障
    // 2 - AD Mode Enable PS丢失
    // 3 - AD Mode Engage PS丢失
    // 4 - SW Angel Request PS丢失
    // 5 - SW Angel Request PS超出范围
    // 6 - SW Angel Request PS斜率超出范围
    // 7 - SW Angel Request PS信号无效
    // 8 - SW Angel Request Valid PS丢失
    special_chassis.df_d17.SteeringErrorEvent1PS = ((data[5] >> 2) & 0x3F);
    //printf("** SteeringErrorEvent1PS=%d\n", special_chassis.df_d17.SteeringErrorEvent1PS);
    break;
  }

  case (0x18FF7CDD): {
    special_chassis.df_d17.CEPS1_state = data[0] & 0x07;
    special_chassis.df_d17.CEPS2_state = (data[0] & 0x38) >> 3;
    // std::cout<<"R转向状态1 (0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error) = "<<State_CEPS1<<std::endl;
    // std::cout<<"R转向状态2 (0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error) = "<<State_CEPS2<<std::endl;
    break;
  }

  case (0x18FF7C13): {
    if (2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }
    Int8_t SteeringSystemADModeStatePS = data[0] & 0x07;
    special_chassis.df_d17.EPS_state = SteeringSystemADModeStatePS;
    if ((0x03 == SteeringSystemADModeStatePS) ||
        (0x04 == SteeringSystemADModeStatePS)) {
      // if(chassis.eps_status != VEH_EPS_STATUS_MANUAL_INTERRUPT) {
        chassis.eps_status = VEH_EPS_STATUS_ROBOTIC;
      // }
    } else if (0x05 == SteeringSystemADModeStatePS) {
        chassis.eps_status = VEH_EPS_STATUS_ERROR;
    } else {
        chassis.eps_status = VEH_EPS_STATUS_MANUAL;
    }

    if(SteeringOverridePS == 0x02)
    {
      chassis.eps_status = VEH_EPS_STATUS_MANUAL_INTERRUPT;
    }
    // printf("SteeringSystemADModeStatePS=%d\n", (Int32_t)SteeringSystemADModeStatePS);
    // std::cout<<"主转向状态 (0:Not Ready / 1:Ready / 2:Reserve / 3:ADU engaged / 4:ADU engaged Degrade / 5:Error)= "
    //         << (Int32_t)SteeringSystemADModeStatePS<<std::endl;
    break;
  }

  // ECUControlInfomation4_EHPS
  case (0x18FF7D13): {
    if (2 != channel) {
      // 避免多通道接收重复的报文
      break;
    }

    special_chassis.df_d17.AHPSFailureCode = data[0] | (data[1] << 8);
    // printf("*****  AHPSFailureCode=%d(0x%04X)\n", 
    //     special_chassis.df_d17.AHPSFailureCode, special_chassis.df_d17.AHPSFailureCode);
    break;
  }

  case (0x18FF7BDD) : {
    if (3 == channel) {
      Int8_t Steering_Error_Event3_Active_CEPS = data[5] & 0x03;
      Int8_t Steering_Error_Event3_CEPS = (data[5] & 0xFC) >> 2;
      //std::cout<<"r转向错误 (0:Inactive / 1:acive) = "<<Steering_Error_Event3_Active_CEPS<<std::endl;
      //std::cout<<"r转向错误事件= "<<Steering_Error_Event3_CEPS<<std::endl;
      //printf("** Steering_Error_Event3_CEPS=%d\n", Steering_Error_Event3_CEPS);
    }
    break;
  }

  case (0x18FF750B): {
    if (2 == channel) {
      Int8_t BrakingSystemADModeState1 = data[3] & 0x07;
      Int8_t BrakingErrorEventActive1 = (data[3] & 0x40) >> 6;
      special_chassis.df_d17.EBS_state = BrakingSystemADModeState1;
      // std::cout<<"c刹车状态 (0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error)= "<<BrakingSystemADModeState1<<std::endl;
      // std::cout<<"c刹车错误事件 (0:Inactive / 1:acive) = "<<BrakingErrorEventActive1<<std::endl;
    }

    if(3 == channel) {
      Int8_t BrakingSystemADModeState1 = data[3] & 0x07;
      Int8_t BrakingSystemADModeState2 = (data[3] & 0x38) >> 3;
      Int8_t BrakingErrorEventActive1 = (data[3] & 0x40) >> 6;
      Int8_t BrakingErrorEventActive2 = (data[3] & 0x80) >> 7;
      Int8_t BrakingErrorEvent1 = data[4] & 0x0F;
      Int8_t BrakingErrorEvent2 = (data[4] & 0xF0) >> 4;
      special_chassis.df_d17.CEBS_state = BrakingSystemADModeState1;
      // std::cout<<"r刹车状态1 (0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error)= "<<BrakingSystemADModeState1<<std::endl;
      // std::cout<<"r刹车状态2 (0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error)= "<<BrakingSystemADModeState2<<std::endl;
      // std::cout<<"r刹车错误事件1 (0:Inactive / 1:acive)= "<<BrakingErrorEventActive1<<std::endl;
      // std::cout<<"r刹车错误事件2 (0:Inactive / 1:acive) = "<<BrakingErrorEventActive2<<std::endl;
      // std::cout<<"r刹车错误事件列表1 = "<<BrakingErrorEvent1<<std::endl;
      // std::cout<<"r刹车错误事件列表2 = "<<BrakingErrorEvent2<<std::endl;
    }

    break;
  }

  case (0x18FEF131): {
    retransmission_can_frames_.mutex.lock();

    // 设置巡航速度 + 
    Int8_t CruiseCtrlSetSwt = data[4] & 0x03;   
    // 设置巡航速度 -
    Int8_t CruiseCtrlResumeSwt = (data[4] >> 4) & 0x03;

    can_dev::CanFrame& frame = retransmission_can_frames_.frame_0x18FEF131;
    frame.id = 0x18FEF131;
    frame.EXT = true;
    frame.RTR = false;
    frame.data_len = 8;
    common::com_memcpy(&frame.data, data, 8);

    retransmission_can_frames_.mutex.unlock();
    break;
  }

  case (0x18FF7332) : {
    Int8_t PowerOnSwitchOfADCU = data[2] & 0x03;
    Int8_t ActiveSwitchOfADCU = (data[2] >> 2) & 0x03;
    bool start_adas = false;
    bool invalid_adas = false;
    bool stop_adas = false;

#if 0
    // 默认状态
    special_chassis.start_adas = 0;
    special_chassis.enable_lka = 0;
    special_chassis.enable_acc = 0;
    special_chassis.enable_aeb = 0;
    // 进入自动驾驶
    button_start_adas_.Update(ActiveSwitchOfADCU, &start_adas, &invalid_adas);
    if(start_adas) {
      special_chassis.start_adas = 2;
      special_chassis.enable_lka = 2;
      special_chassis.enable_acc = 2;
      special_chassis.enable_aeb = 2;
    }
    // 退出自动驾驶
    button_stop_adas_.Update(PowerOnSwitchOfADCU, &stop_adas, &invalid_adas);
    if(stop_adas) {
      special_chassis.start_adas = 1;
      special_chassis.enable_lka = 1;
      special_chassis.enable_acc = 1;
      special_chassis.enable_aeb = 1;
    }
#endif
    break;
  }

  case (0x0CF00400): {
    retransmission_can_frames_.mutex.lock();

    can_dev::CanFrame& frame = retransmission_can_frames_.frame_0x0CF00400;
    frame.id = 0x0CF00400;
    frame.EXT = true;
    frame.RTR = false;
    frame.data_len = 8;
    common::com_memcpy(&frame.data, data, 8);

    retransmission_can_frames_.mutex.unlock();

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
     // 发动机: EEC1—电子发动机控制器, 10-100ms
    static int flag_count = 0;
    // 转矩控制模式, 0000-无需求, 0001-加速踏板, 0010-巡航控制, 0011-PTO, 0100-Road Speed Governor (HGB)
    // 0101-ASR, 0110-Transmission Contro (GS）, 0111-ABS Control (ABS), 1000-Torque Limiting (MBEG)
    // 1001-High Speed Governor (EDR), 1010-Braking System (EBS)
    unsigned char engine_torque_control_mode = data[0] & 0x0F;

    // vehicle_status_.EngineTorqueControlMode = engine_torque_control_mode;
    // if(engine_torque_control_mode == 0x0A)	//EBS
    // if(!(data[0] >> 4)) {
    //   flag_count++;
    //   if (flag_count > 5) {
    //     flag_count = 5;
    //     info->vlcStatus = 1;
    //   }
    // } else {
    //   flag_count = 0;
    //   info->vlcStatus = 0;
    // }

    // 驾驶员需求驱动-转矩百分比, 1%, -125%, -125% ~ +125%
    float driver_request_drive_torque_rate = data[1] - 125.0f;
    dfcv_special_chassis.driver_damand_torque_percent = driver_request_drive_torque_rate;
    // vehicle_status_.EngineDriverRequestTorqueRate = driver_request_drive_torque_rate;

    // 实际发动机-转矩百分比, 1%, -125%, -125% ~ +125%
    float actual_engine_torque_rate = data[2] - 125.0f;
    dfcv_special_chassis.actual_engine_torque_percent = actual_engine_torque_rate;
    // vehicle_status_.EngineTorqueRate = actual_engine_torque_rate;
    unsigned char source_address_engine_control_device = data[5];
    dfcv_special_chassis.source_address_engine_control_device = source_address_engine_control_device;
#endif

    // 发动机转速, 0.125rpm, 0, 0 ~ 8031.875rpm
    //vehicle_status_.EngineSpeedStatus = VEH_DATA_STATUS_VALID;
    float engine_speed = ((unsigned int)(data[3]) | (unsigned int)(data[4] << 8)) * 0.125f;
    chassis.engine_speed = engine_speed;
    chassis.engine_speed_valid = 1;

    chassis.engine_torque = 3316 * (data[2] - 125.0f) / 100.0;
    chassis.engine_torque_valid = 1;

    break;
  }

  case (0x18FED917): {
    retransmission_can_frames_.mutex.lock();

    can_dev::CanFrame& frame = retransmission_can_frames_.frame_0x18FED917;
    frame.id = 0x18FED917;
    frame.EXT = true;
    frame.RTR = false;
    frame.data_len = 8;
    common::com_memcpy(&frame.data, data, 8);

    retransmission_can_frames_.mutex.unlock();

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    //挂车状态
    unsigned char trailer_connected_status = (data[1] >> 6);
    dfcv_special_chassis.trailer_connected_status = trailer_connected_status;
#endif
    
    break;
  }

  case (0x18FEAE30): {
    retransmission_can_frames_.mutex.lock();

    can_dev::CanFrame& frame = retransmission_can_frames_.frame_0x18FEAE30;
    frame.id = 0x18FEAE30;
    frame.EXT = true;
    frame.RTR = false;
    frame.data_len = 8;
    common::com_memcpy(&frame.data, data, 8);

    retransmission_can_frames_.mutex.unlock();
    break;
  }

  default:
    break;
  }

  chassis.msg_head.valid = 1;
  Phoenix_SharedData_SetChassis(&chassis);
  Phoenix_SharedData_SetSpecialChassisInfo(&special_chassis);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  Phoenix_SharedData_SetDFCVSpecialChassisInfo(&dfcv_special_chassis);
#endif

}

void CanAccessDfD17B2::SendRetransmissionCanFrames() {
  auto func_send_tar_frame = [this](can_dev::CanFrame& tar_frame) {
    bool valid_frame = false;
    can_dev::CanFrame frame;

    retransmission_can_frames_.mutex.lock();

    if (tar_frame.data_len > 0) {
      valid_frame = true;
      common::com_memcpy(&frame, &tar_frame, sizeof(can_dev::CanFrame));
      tar_frame.data_len = 0;
    }

    retransmission_can_frames_.mutex.unlock();

    if (valid_frame) {
      can_channel_3_.Send(&frame);
    }
  };

  func_send_tar_frame(retransmission_can_frames_.frame_0x18FEF131);
  func_send_tar_frame(retransmission_can_frames_.frame_0x0CF00400);
  func_send_tar_frame(retransmission_can_frames_.frame_0x18FED917);
  func_send_tar_frame(retransmission_can_frames_.frame_0x18FEAE30);
}


// AutoDrive_ADCU  send to P,B,C,R   20ms
void CanAccessDfD17B2::SendFrame0x0CFF649E(
    Uint8_t enable, Uint8_t engage) {
  can_dev::CanFrame frame;
  frame.id = 0x0CFF649E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  // 0x0 :inhibit, 0x1 :enable, 0x2 :fail, 0x3 :off
  Uint8_t ADModeEnablePS = enable;
  // 0x0 :inhibit, 0x1 :engage, 0x2 :reserve, 0x3 :reserve
  Uint8_t ADModeEngagePS = engage;

  // 信息计数器
  Uint8_t count = (Uint8_t)frame_alive_count_0x0CFF649E_;

  frame.data[1] = 0xF0 | count;
  frame.data[2] =  0xCC | ((ADModeEnablePS & 0x03) | ((ADModeEngagePS & 0x03) << 4));
  frame.data[3] = (0xFF);
  frame.data[4] = (0xFF);
  frame.data[5] = (0xFF);
  frame.data[6] = (0xFF);
  frame.data[7] = (0xFF);
  frame.data[0] = CalculateCrc8(frame.data);

  frame_alive_count_0x0CFF649E_ ++;
  if (frame_alive_count_0x0CFF649E_ >= 15) {
    frame_alive_count_0x0CFF649E_ = 0;
  }

  can_channel_0_.Send(&frame);
  can_channel_1_.Send(&frame);
  can_channel_2_.Send(&frame);
  can_channel_3_.Send(&frame);
}

// AutoDrive_rADCU  send to P,B,C,R  20ms
void CanAccessDfD17B2::SendFrame0x0CFF64DC(
    Uint8_t enable, Uint8_t engage) {
  can_dev::CanFrame frame;
  frame.id = 0x0CFF64DC;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  // 0x0 :inhibit, 0x1 :enable, 0x2 :fail, 0x3 :off
  Uint8_t ADModeEnablePS = enable;
  // 0x0 :inhibit, 0x1 :engage, 0x2 :reserve, 0x3 :reserve
  Uint8_t ADModeEngagePS = engage;

  // 信息计数器
  Uint8_t count = (Uint8_t)frame_alive_count_0x0CFF64DC_;

  frame.data[1] =  0xF0 | count;
  frame.data[2] = 0x33 | (((ADModeEnablePS & 0x03) << 2) | ((ADModeEngagePS & 0x03) << 6));
  frame.data[3] = (0xFF);
  frame.data[4] = (0xFF);
  frame.data[5] = (0xFF);
  frame.data[6] = (0xFF);
  frame.data[7] = (0xFF);
  frame.data[0] = CalculateCrc8(frame.data);

  frame_alive_count_0x0CFF64DC_ ++;
  if (frame_alive_count_0x0CFF64DC_ >= 15) {
    frame_alive_count_0x0CFF64DC_ = 0;
  }
  can_channel_0_.Send(&frame);
  can_channel_1_.Send(&frame);
  can_channel_2_.Send(&frame);
  can_channel_3_.Send(&frame);
}

// 油门档位下发 Send to P  10ms
void CanAccessDfD17B2::SendFrame0x0CFF659E(
    const ChassisCtlCmd_t& cmd, Uint8_t gear_mode) {
  can_dev::CanFrame frame;
  frame.id = 0x0CFF659E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  Uint8_t transmission_requested_gear = 0;

  Uint8_t accelerater_pedal_position_demand = 0;
  // 加速踏板开度下发 精度0.4
  if (cmd.acc_value < 0) { // 油门开度值由控制模块做平滑处理，以解决"大油门到滑行时出现的咯噔响”. 在PCC模式下,acc_value值可能会出现为-1的情况，目的是在一定条件下开启缓速器
    accelerater_pedal_position_demand = 0 * 2.5F;
  } else {
    accelerater_pedal_position_demand = cmd.acc_value * 2.5F;
  }
  
  if (cmd.brake_value > 0.1F) {
    accelerater_pedal_position_demand = 0;
  }

  if (gear_mode) {
    // Transmission Requested Gear
    // 0XE0 = No Command
    // 0xFE = Error
    // 0xFC = Forward Drive Position
    // 0xDF = Reverse Selector Position
    // 0xDE = Neutral Position
    switch (cmd.gear) {
    case (VEH_GEAR_P):
    case (VEH_GEAR_N):
      transmission_requested_gear = 0xDE;
      break;
    case (VEH_GEAR_R):
      transmission_requested_gear = 0xDF;
      break;
    case (VEH_GEAR_D):
      transmission_requested_gear = 0xFC;
      break;
    default:
      transmission_requested_gear = 0xE0;
      break;
    }
  } else {
    transmission_requested_gear = 0xE0;
  }
  transmission_requested_gear = 0xE0;

  // 经济模式/动力模式, 0x0:E, 0x1:P, 0x2:Reserved, 0x3:Take no action
  Uint8_t E_P_mode_demand = 0;
  // 蠕动模式, 0x0:DM, 0x1:RM, 0x2:Reserved, 0x3:Take no action
  Uint8_t creep_mode = 0;
  // 辅助制动模式, 0x0:OFF, 0x1:恒速档, 0x2:辅助制动1档, 0x3:助制动2档, 0x4:辅助制动3档
  Uint8_t auxiliary_braking_mode = 1;

  if (15 == cmd.tar_type) {
    if (cmd.acc_value < 0.0) {
      auxiliary_braking_mode = 1;
    } else {
      auxiliary_braking_mode = 0; // PCC 长下坡 超速于目标车速10% 开启缓速器
    }
  }

  frame.data[0] = accelerater_pedal_position_demand;
  frame.data[1] = transmission_requested_gear;
  frame.data[2] =
      (E_P_mode_demand & 0x03) |
      ((creep_mode & 0x03) << 2) |
      ((auxiliary_braking_mode & 0x0F) << 4);
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  can_channel_0_.Send(&frame);
}

// XBR-制动需求 Send to P,C,R 20ms
void CanAccessDfD17B2::SendFrame0x0C040B9E(
    const ChassisCtlCmd_t& cmd, Uint8_t enable) {
  can_dev::CanFrame frame;
  frame.id = 0x0C040B9E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

//  Float32_t brake_key_ratio = 0.0F;
//  Float32_t deceleration_key_ratio = 0.0F;
//  Int32_t brake_key_index = Phoenix_Common_LerpInOrderedTable_f(
//        current_brake_key_table,
//        DECELERATION_TABLE_SIZE,
//        cmd.brake_value,
//        &brake_key_ratio);
//  Float32_t deceleration_tgap = Phoenix_Common_Lerp_f(
//        deceleration_tgap_table[brake_key_index],
//        deceleration_tgap_table[brake_key_index + 1],
//        deceleration_key_ratio
//        );
  // 外部加速要求，1/2048m/s^2，(-10)~(+10)m/s^2，偏移：-15.69m/s2
  // unsigned short external_accelerate_requirement = (-cmd_.BrakeValue *0.01f*10.0f + 15.687f) * 2048.0f;
  Uint16_t external_accelerate_requirement = (-cmd.brake_value * 0.01F * 10.0F/* * deceleration_tgap*/ + 15.687F) * 2048.0F;
  if (cmd.brake_value < 1.0F) {
    external_accelerate_requirement = 0x7D7F;
  }
  // 外部制动要求EBI模式，00-不允许耐久制动集成，01-只允许耐久制动，10-允许耐久制动集成，11-未定义
  Uint8_t external_brake_requirement_ebi_mode = 0x02;
  // 外部制动要求优先级，00-最高优先级，01-高等优先级，10-中等优先级，11-低等优先级
  // 2. XBR Priority : 对于ACC功能中的制动控制，需要发送10b-Medium priority;
  //                   对于AEBS功能中的制动控制，需发送00b-Highest Priority.
  //                   无制动控制请求时发送11b。
  Uint8_t external_brake_requirement_priority = 0x03;
  if (cmd.brake_value > 1.0F) {
    external_brake_requirement_priority = 0x02;
  }
  // 外部制动要求控制模式，00-超载禁用，01-附加模式的加速度控制，10-最大模式的加速度控制，11-未定义
  // 3. XBR control mode：在外部制动请求发送时通常选择发送Maximum mode.
  //                      含义：此时会有可能驾驶员和外部制动请求同时起作用（AEBS介入时），
  //                           在驾驶员需求减速度与外部制动请求减速度之间取最大的减速度请求。
  // Uint8_t external_brake_requirement_control_mode = 0x02;
  if ((0x02 == enable) && (cmd.brake_value > 1.0F)) {
    enable = 0x02;
  } else {
    enable = 0x00;
  }
  Uint8_t external_brake_requirement_control_mode = enable;
  // 外部制动需求紧迫性，0.4%，1%~100%
  Uint8_t external_brake_requirement_urgency = (unsigned char)(100*2.5f);
  // Uint8_t external_brake_requirement_urgency = 0x00;
  // 信息计数器
  Uint8_t count = (Uint8_t)frame_alive_count_0x0C040B9E_;
  // printf("external_accelerate_requirement == %x,\n", external_accelerate_requirement);
  frame.data[0] = (external_accelerate_requirement & 0xFF);
  frame.data[1] = (external_accelerate_requirement >> 8);
  frame.data[2] = (external_brake_requirement_ebi_mode & 0x03) |
      ((external_brake_requirement_priority & 0x03) << 2) |
      ((external_brake_requirement_control_mode & 0x03) << 4);
  frame.data[3] = external_brake_requirement_urgency;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;

  Uint8_t msg_counter = count & 0x0F;
  Uint8_t id_sum = (frame.id & 0xFF) + ((frame.id >> 8) & 0xFF) +
      ((frame.id >> 16) & 0xFF) + ((frame.id >> 24) & 0xFF);
  Uint8_t check_sum = frame.data[0] + frame.data[1] + frame.data[2] +
      frame.data[3] + frame.data[4] + frame.data[5] + frame.data[6] +
      msg_counter + id_sum;
  Uint8_t xbr_check_sum = ((check_sum >> 4) + check_sum) & 0x0F;
  frame.data[7] = msg_counter | (xbr_check_sum << 4);

  frame_alive_count_0x0C040B9E_ ++;
  if (frame_alive_count_0x0C040B9E_ > 15) {
    frame_alive_count_0x0C040B9E_ = 0;
  }

  can_channel_0_.Send(&frame);
  can_channel_2_.Send(&frame);
  can_channel_3_.Send(&frame);
}

// rXBR-制动需求 Send to R  20ms
void CanAccessDfD17B2::SendFrame0x0C040BDC() {
  can_dev::CanFrame frame;
  frame.id = 0x0C040BDC;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  frame.data[0] = 0x7F;
  frame.data[1] = 0x7D;
  frame.data[2] = 0xC0;
  frame.data[3] = 0x00;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;

  Uint8_t msg_counter = frame_alive_count_0x0C040BD7_ & 0x0F;
  Uint8_t id_sum = (frame.id & 0xFF) + ((frame.id >> 8) & 0xFF) +
      ((frame.id >> 16) & 0xFF) + ((frame.id >> 24) & 0xFF);
  Uint8_t check_sum = frame.data[0] + frame.data[1] + frame.data[2] +
      frame.data[3] + frame.data[4] + frame.data[5] + frame.data[6] +
      msg_counter + id_sum;
  Uint8_t xbr_check_sum = ((check_sum >> 4) + check_sum) & 0x0F;
  frame.data[7] = msg_counter | (xbr_check_sum << 4);

  frame_alive_count_0x0C040BD7_ ++;
  if (frame_alive_count_0x0C040BD7_ > 15) {
    frame_alive_count_0x0C040BD7_ = 0;
  }
  can_channel_3_.Send(&frame);
}

// 方向盘转速限制
#define STEERING_SPEED_LIMIT_TABLE_SIZE (10)
// 之前的方向盘指令转角
static Int32_t s_valid_prev_steering_angle_cmd = 0;
static Float32_t s_prev_steering_angle_cmd = 0.0F;

// 通过速度限制方向盘转角速率
static const Float32_t s_key_tab_str_spd_lmt_by_spd[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
    0.0F/3.6F,  30.0F/3.6F,  40.0F/3.6F, 50.0F/3.6F,  60.0F/3.6F,
   70.0F/3.6F,  80.0F/3.6F,  90.0F/3.6F, 100.0F/3.6F, 110.0F/3.6F
};
static const Float32_t s_val_tab_str_spd_lmt_by_spd[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
  150.0F*0.017453293F, 150.0F*0.017453293F, 130.0F*0.017453293F, 120.0F*0.017453293F,  90.0F*0.017453293F,
   60.0F*0.017453293F,  50.0F*0.017453293F,  40.0F*0.017453293F,  30.0F*0.017453293F,  30.0F*0.017453293F,
};

// 限制指令角度相对于当前角度的增量
static const Float32_t s_key_tab_str_spd_lmt_by_cur[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
    0.0F,               15.0F*0.017453293F,  45.0F*0.017453293F,  90.0F*0.017453293F, 135.0F*0.017453293F,
  180.0F*0.017453293F, 225.0F*0.017453293F, 270.0F*0.017453293F, 315.0F*0.017453293F, 360.0F*0.017453293F,
};
static const Float32_t s_val_tab_str_spd_lmt_by_cur[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
    0.0F,                3.0F*0.017453293F,   5.0F*0.017453293F,  10.0F*0.017453293F,  20.0F*0.017453293F,
   25.0F*0.017453293F,  30.0F*0.017453293F,  40.0F*0.017453293F,  50.0F*0.017453293F,  60.0F*0.017453293F,
};

// 限制指令角度相对于上一次指令转角的增量
static const Float32_t s_key_tab_str_spd_lmt_by_cmd[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
    0.0F,               15.0F*0.017453293F,  45.0F*0.017453293F,  90.0F*0.017453293F, 135.0F*0.017453293F,
  180.0F*0.017453293F, 225.0F*0.017453293F, 270.0F*0.017453293F, 315.0F*0.017453293F, 360.0F*0.017453293F,
};
static const Float32_t s_val_tab_str_spd_lmt_by_cmd[STEERING_SPEED_LIMIT_TABLE_SIZE] = {
    0.0F,                0.3F*0.017453293F,   0.9F*0.017453293F,   1.8F*0.017453293F,   2.7F*0.017453293F,
    3.6F*0.017453293F,   4.5F*0.017453293F,   5.4F*0.017453293F,   6.3F*0.017453293F,   7.2F*0.017453293F,
};

// ADU_PsControlRequest_ADCU 转向下发 Send to C,R
void CanAccessDfD17B2::SendFrame0x0CFF799E(
    const ChassisCtlCmd_t& cmd, Uint8_t ctl_mode,
    const Chassis_t& status, const SpecialChassisInfo_t& special_chassis) {
  // 方向盘转速限制
  // 通过速度限制方向盘转角速率
  Float32_t key_ratio_by_spd = 0.0F;
  Int32_t key_idx_by_spd = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_str_spd_lmt_by_spd,
        STEERING_SPEED_LIMIT_TABLE_SIZE,
        status.v,
        &key_ratio_by_spd);
  Float32_t str_spd_lmt_by_spd = Phoenix_Common_Lerp_f(
        s_val_tab_str_spd_lmt_by_spd[key_idx_by_spd],
        s_val_tab_str_spd_lmt_by_spd[key_idx_by_spd+1],
        key_ratio_by_spd);
  Float32_t steering_wheel_speed =
      (cmd.steering_wheel_speed > str_spd_lmt_by_spd) ?
        str_spd_lmt_by_spd : cmd.steering_wheel_speed;

  // 限制指令角度相对于当前角度的增量
  Float32_t key_ratio_by_cur = 0.0F;
  Int32_t key_idx_by_cur = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_str_spd_lmt_by_cur,
        STEERING_SPEED_LIMIT_TABLE_SIZE,
        phoenix_com_abs_f(steering_wheel_speed),
        &key_ratio_by_cur);
  Float32_t step_lmt_by_cur = Phoenix_Common_Lerp_f(
        s_val_tab_str_spd_lmt_by_cur[key_idx_by_cur],
        s_val_tab_str_spd_lmt_by_cur[key_idx_by_cur+1],
        key_ratio_by_cur);
  Float32_t step_to_cur =
      cmd.steering_wheel_angle - status.steering_wheel_angle;

  // 限制指令角度相对于上一次指令转角的增量
  Float32_t key_ratio_by_cmd = 0.0F;
  Int32_t key_idx_by_cmd = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_str_spd_lmt_by_cmd,
        STEERING_SPEED_LIMIT_TABLE_SIZE,
        phoenix_com_abs_f(steering_wheel_speed),
        &key_ratio_by_cmd);
  Float32_t step_lmt_by_cmd = Phoenix_Common_Lerp_f(
        s_val_tab_str_spd_lmt_by_cmd[key_idx_by_cmd],
        s_val_tab_str_spd_lmt_by_cmd[key_idx_by_cmd+1],
        key_ratio_by_cmd);

  // 当前方向盘指令转角
  Float32_t tar_steering_angle = cmd.steering_wheel_angle;

#if ENABLE_LOG_FILE_STEERING_CMD
#endif

#if 0
  printf("***  ctl_mode=%d, eps_st=%d, prev_angle=%0.1fdeg, cur_angle=%0.1fdeg, tar_angle=%0.1fdeg, tar_speed=%0.1fdeg/s\n"
         "     spd=%0.1fkm/h, str_spd_lmt_by_spd=%0.1fdeg/s, steering_wheel_speed=%0.1fdeg/s\n"
         "     step_lmt_by_cur=%0.1fdeg/s, step_lmt_by_cmd=%0.1fdeg/s"
         "\n",
         ctl_mode, status.eps_status, s_prev_steering_angle_cmd*57.29578F, status.steering_wheel_angle*57.29578F,
         cmd.steering_wheel_angle*57.29578F, cmd.steering_wheel_speed*57.29578F,
         status.v*3.6F, str_spd_lmt_by_spd*57.29578F, steering_wheel_speed*57.29578F,
         step_lmt_by_cur*57.29578F, step_lmt_by_cmd*57.29578F);
#endif

  // 限制指令角度
  Int32_t limit_flag_cur = 0;
  if (step_to_cur > step_lmt_by_cur) {
    tar_steering_angle = status.steering_wheel_angle + step_lmt_by_cur;
    limit_flag_cur = 1;
#if 0
    printf("     (cur)(+) limit angle to %0.1fdeg\n", tar_steering_angle*57.29578F);
#endif
  } else if (step_to_cur < -step_lmt_by_cur) {
    tar_steering_angle = status.steering_wheel_angle - step_lmt_by_cur;
    limit_flag_cur = 2;
#if 0
    printf("     (cur)(-) limit angle to %0.1fdeg\n", tar_steering_angle*57.29578F);
#endif
  } else {
    // nothing to do
  }

  Int32_t limit_flag_cmd = 0;
#if 1
  if (s_valid_prev_steering_angle_cmd && ctl_mode) {
    if (tar_steering_angle > (s_prev_steering_angle_cmd + step_lmt_by_cmd)) {
      tar_steering_angle = s_prev_steering_angle_cmd + step_lmt_by_cmd;
      limit_flag_cmd = 1;
#if 0
      printf("     (cmd)(+) limit angle to %0.1fdeg\n", tar_steering_angle*57.29578F);
#endif
    } else if (tar_steering_angle < (s_prev_steering_angle_cmd - step_lmt_by_cmd)) {
      tar_steering_angle = s_prev_steering_angle_cmd - step_lmt_by_cmd;
      limit_flag_cmd = 2;
#if 0
      printf("     (cmd)(-) limit angle to %0.1fdeg\n", tar_steering_angle*57.29578F);
#endif
    } else {
      // nothing to do
    }
  }
#endif

#if ENABLE_LOG_FILE_STEERING_CMD
  if (!open_log_file_steering_cmd_) {
    open_log_file_steering_cmd_ = true;
    log_file_steering_cmd_.Open();
    log_file_steering_cmd_.Enable(true);

    std::snprintf(
          str_buff_steering_cmd_, sizeof(str_buff_steering_cmd_)-1,
          "timestamp,ctl_mode,eps_st,Event1ActivePS,Event1PS,veh_spd,"
          "prv_str_angle,cur_str_angle,cur_str_spd,"
          "tar_str_angle_planning,tar_str_spd_planning,str_spd_lmt_by_spd,"
          "tar_str_spd,step_lmt_by_cur,step_lmt_by_cmd,"
          "limit_flag_cur,limit_flag_cmd,tar_str_angle,delta_tar_str_angle");
    log_file_steering_cmd_.Write(str_buff_steering_cmd_);
  }

  std::snprintf(
        str_buff_steering_cmd_, sizeof(str_buff_steering_cmd_)-1,
        "%ld,%d,%d,%d,%d,%0.1f,"
        "%0.1f,%0.1f,%0.1f,"
        "%0.1f,%0.1f,%0.1f,"
        "%0.1f,%0.1f,%0.1f,"
        "%d,%d,%0.1f,%0.1f"
        "\n",
        Phoenix_Common_GetSysClockNowMs(), ctl_mode, status.eps_status, special_chassis.df_d17.SteeringErrorEvent1ActivePS, special_chassis.df_d17.SteeringErrorEvent1PS, status.v*3.6F,
        s_prev_steering_angle_cmd*57.29578F, status.steering_wheel_angle*57.29578F, status.steering_wheel_speed*57.29578F,
        cmd.steering_wheel_angle*57.29578F, cmd.steering_wheel_speed*57.29578F, str_spd_lmt_by_spd*57.29578F,
        steering_wheel_speed*57.29578F, step_lmt_by_cur*57.29578F, step_lmt_by_cmd*57.29578F,
        limit_flag_cur, limit_flag_cmd, tar_steering_angle*57.29578F, (tar_steering_angle-s_prev_steering_angle_cmd)*57.29578F);
  log_file_steering_cmd_.Write(str_buff_steering_cmd_);
#endif

  // 保存指令角度
  s_valid_prev_steering_angle_cmd = 1;
  s_prev_steering_angle_cmd = tar_steering_angle;


  // printf("### cmd.steering_wheel_speed=%0.1f\n",
  //       common::com_rad2deg(cmd.steering_wheel_speed));

  can_dev::CanFrame frame;
  frame.id = 0x0CFF799E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;
  Int16_t steering_angle_request =
      (Int16_t)(phoenix::common::com_rad2deg(
                  tar_steering_angle + steering_wheel_angle_offset_) * 10);
//  Int8_t ctl_mode_ = 0x01;
//  if (ctl_mode) {
//    ctl_mode_ = 0x00;
//  } else {
//    ctl_mode_ = 0x01;
//  }

  frame.data[0] = (steering_angle_request & 0xFF);
  frame.data[1] = ((steering_angle_request >> 8) & 0xFF);
  frame.data[2] = 0x00;
  frame.data[3] = (0x00);
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = frame_alive_count_0x0CFF799E_;
  frame.data[7] = frame.data[0] + frame.data[1] + frame.data[2] +
      frame.data[3] + frame.data[4] + frame.data[5] + frame.data[6];

  frame_alive_count_0x0CFF799E_ ++;
  if(frame_alive_count_0x0CFF799E_ > 255){
    frame_alive_count_0x0CFF799E_ = 0;
  }
  can_channel_2_.Send(&frame);
  can_channel_3_.Send(&frame);
}

// ADU_PsControlRequest_rADCU 转向下发 Send to R  20ms
void CanAccessDfD17B2::SendFrame0x0CFF79DC() {
  can_dev::CanFrame frame;
  frame.id = 0x0CFF79DC;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;
  Int16_t steering_angle_request =
      (Int16_t)(phoenix::common::com_rad2deg(
                  0.0 + steering_wheel_angle_offset_) * 10);

  frame.data[0] = (steering_angle_request & 0xFF);
  frame.data[1] = ((steering_angle_request >> 8) & 0xFF);
  frame.data[2] = (0x00);
  frame.data[3] = (0x00);
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = frame_alive_count_0x0CFF79DC_;
  frame.data[7] = frame.data[0] + frame.data[1] + frame.data[2] +
      frame.data[3] + frame.data[4] + frame.data[5] + frame.data[6];

  frame_alive_count_0x0CFF79DC_ ++;
  if(frame_alive_count_0x0CFF79DC_ > 255){
    frame_alive_count_0x0CFF79DC_ = 0;
  }
  can_channel_3_.Send(&frame);
}

// Warnning_EBS_ADCU  Send to C,R  100ms
void CanAccessDfD17B2::SendFrame0x18FF769E() {
  can_dev::CanFrame frame;
  frame.id = 0x18FF769E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  frame.data[0] = (0x00);
  frame.data[1] = (0x00);
  frame.data[2] = (0x00);
  frame.data[3] = (0x00);
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = (0x00);
  frame.data[7] = (0x00);

  can_channel_2_.Send(&frame);
  can_channel_3_.Send(&frame);
}

// Warnning_EBS_rADCU  Send to R  100ms
void CanAccessDfD17B2::SendFrame0x18FF76DC() {
  can_dev::CanFrame frame;
  frame.id = 0x18FF76DC;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  frame.data[0] = (0x00);
  frame.data[1] = (0x00);
  frame.data[2] = (0x00);
  frame.data[3] = (0x00);
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = (0x00);
  frame.data[7] = (0x00);

  can_channel_3_.Send(&frame);
}


#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)


}  // namespace df_d17_b2
}  // namespace framework
}  // namespace phoenix
