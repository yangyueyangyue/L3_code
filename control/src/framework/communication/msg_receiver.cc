/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_receiver.h"

#include "utils/com_utils.h"
#include "utils/com_clock_c.h"
#include "data_serialization_c.h"
#include "communication/parse_proto_msg.h"

#if ENTER_PLAYBACK_MODE_ADHMI
#include "container/ring_buffer_c.h"
#include "communication_c/shared_data_c.h"
#include "math/math_utils.h"
#endif

namespace phoenix {
namespace framework {

#if ENTER_PLAYBACK_MODE_ADHMI
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
      if (weight > 1.1F*prev_weight) {
        // printf("limit weight from %0.1f to %0.1f\n", weight*1000, 1.1F*prev_weight*1000);
        weight = 1.1F*prev_weight;
      } else if (weight < 0.9F*prev_weight) {
        // printf("limit weight from %0.1f to %0.1f\n", weight*1000, 0.9F*prev_weight*1000);
        weight = 0.9F*prev_weight;
      } else {
        // nothing to do
      }
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

#endif

#if (ENABLE_UDP_NODE)
static void UDP_RecvImu(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver *handler = (MsgReceiver *)user;

  handler->HandleImuMessageUdp(buf, buf_len);
}

#if ENTER_PLAYBACK_MODE
static void UDP_RecvChassis(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver *handler = (MsgReceiver *)user;

  handler->HandleChassisMessageUdp(buf, buf_len);
}
#endif

static void UDP_RecvPlanningResult(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver *handler = (MsgReceiver *)user;

  handler->HandlePlanningResultMessageUdp(buf, buf_len);
}
#endif


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (IN)           任务管理模块
 *              ros_node        (IN)           ros节点模块
 *
 *  @retval     None
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
MsgReceiver::MsgReceiver(Task* manager) :
  Task(TASK_ID_MSG_RECEIVER, "Message Receiver", manager) {
#if (ENABLE_ROS_NODE)
  ros_node_ = Nullptr_t;
#endif
#if (ENABLE_LCM_NODE)
  lcm_node_ = Nullptr_t;
#endif
#if (ENABLE_UDP_NODE)
  udp_node_ = Nullptr_t;
#endif
}

/******************************************************************************/
/** 启动消息接收模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息接收模块，向总线订阅需要的报文
 *
 *  <Attention>
 *       None
 *
 */
bool MsgReceiver::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  /// Comunicate by ROS
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->Subscribe(
          "localization/imu", 1,
          &MsgReceiver::HandleImuMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

#if ENTER_PLAYBACK_MODE
    ret = ros_node_->Subscribe(
          "control/chassis", 1,
          &MsgReceiver::HandleChassisMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }
#endif

    ret = ros_node_->Subscribe(
          "planning/planning_result", 1,
          &MsgReceiver::HandlePlanningResultMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"planning/planning_result\".";
    }

    ret = ros_node_->Subscribe(
          "control/DFCV_special_chassis_info", 1,
          &MsgReceiver::HandleDFCVSpecialChassisInfoMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/DFCV_special_chassis_info\".";
    }

#if ENTER_PLAYBACK_MODE_ADHMI
    /**
     * CAN4 -> P CAN
     * CAN5 -> B CAN
     * CAN6 -> C CAN
     * CAN7 -> R CAN
     */
    ret = ros_node_->Subscribe(
          "/can_4", 1,
          &MsgReceiver::HandleADHMICANPResultMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"/can_4\".";
    } 

    ret = ros_node_->Subscribe(
          "/can_5", 1,
          &MsgReceiver::HandleADHMICANBResultMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"/can_5\".";
    }   

    ret = ros_node_->Subscribe(
          "/can_6", 1,
          &MsgReceiver::HandleADHMICANCResultMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"/can_6\".";
    }   

    ret = ros_node_->Subscribe(
          "/can_7", 1,
          &MsgReceiver::HandleADHMICANRResultMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"/can_7\".";
    }
#endif
  }
#endif  // #if (ENABLE_ROS_NODE)


#if (ENABLE_LCM_NODE)
  /// Comunicate by LCM
  if (Nullptr_t != lcm_node_) {
    ret = lcm_node_->Subscribe(
          "localization/imu",
          &MsgReceiver::HandleImuMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

#if ENTER_PLAYBACK_MODE
    ret = lcm_node_->Subscribe(
          "control/chassis",
          &MsgReceiver::HandleChassisMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }
#endif

    ret = lcm_node_->Subscribe(
          "planning/planning_result",
          &MsgReceiver::HandlePlanningResultMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }
  }
#endif  // #if (ENABLE_LCM_NODE)


#if (ENABLE_UDP_NODE)
  /// Comunicate by UDP
  if (Nullptr_t != udp_node_) {
    ret = udp_node_->Subscribe(
          "localization/imu",
          &UDP_RecvImu,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

#if ENTER_PLAYBACK_MODE
    ret = udp_node_->Subscribe(
          "control/chassis",
          &UDP_RecvChassis,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }
#endif

    ret = udp_node_->Subscribe(
          "planning/planning_result",
          &UDP_RecvPlanningResult,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"planning/planning_result\".";
    }
  }
#endif  // #if (ENABLE_UDP_NODE)

  return (true);
}


#if (ENABLE_ROS_NODE)
/// Comunicate by ROS
void MsgReceiver::HandleImuMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeImuMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_imu_info_)) {

      MessageImu msg_imu(&ros_imu_info_);
      Notify(msg_imu);
    }
  }
}

#if ENTER_PLAYBACK_MODE
void MsgReceiver::HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_chassis_info_)) {

      MessageChassis msg_chassis(&ros_chassis_info_);
      Notify(msg_chassis);
    }
  }
}
#endif

void MsgReceiver::HandlePlanningResultMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodePlanningResultMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_planning_result_)) {

      MessagePlanningResult msg_planning_result(&ros_planning_result_);
      Notify(msg_planning_result);
    }
  }
}

void MsgReceiver::HandleDFCVSpecialChassisInfoMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeDFCVSpecialChassisInfoMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_DFCV_special_chassis_info_)) {

      MessageDFCVSpecialChassisInfo msg_DFCV_special_chassis_info(&ros_DFCV_special_chassis_info_);
      #if ENTER_PLAYBACK_MODE
      std::cout << "dfcv_chassis_info vehicle_mass:" << msg_DFCV_special_chassis_info.dfcv_special_chassis_info()->vehicle_mass << std::endl;
      #endif
      Notify(msg_DFCV_special_chassis_info);
    }
  }
}

#if ENTER_PLAYBACK_MODE_ADHMI
void MsgReceiver::HandleADHMICANPResultMessageRos(const ::control::CanFrame& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.data.size() > 0) {
    static Uint8_t data_len = (msg.data.data.size() > 8) ? 8 : (msg.data.data.size());
    static can_dev::CanFrame p_can_frame;
    p_can_frame.id = msg.id;
    for(int i = 0; i < data_len; i++) {
      p_can_frame.data[i] = msg.data.data[i];
    }
    ParseCanFrame(p_can_frame, 0);

    // Monitor
    MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_0);
    Notify(message);
  }
}

void MsgReceiver::HandleADHMICANBResultMessageRos(const ::control::CanFrame& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.data.size() > 0) {
    static Uint8_t data_len = (msg.data.data.size() > 8) ? 8 : (msg.data.data.size());
    static can_dev::CanFrame b_can_frame;
    b_can_frame.id = msg.id;
    for(int i = 0; i < data_len; i++) {
      b_can_frame.data[i] = msg.data.data[i];
    }
    ParseCanFrame(b_can_frame, 1);

    // Monitor
    MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_1);
    Notify(message);
  }
}

void MsgReceiver::HandleADHMICANCResultMessageRos(const ::control::CanFrame& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.data.size() > 0) {
    static Uint8_t data_len = (msg.data.data.size() > 8) ? 8 : (msg.data.data.size());
    static can_dev::CanFrame c_can_frame;
    c_can_frame.id = msg.id;
    for(int i = 0; i < data_len; i++) {
      c_can_frame.data[i] = msg.data.data[i];
    }
    ParseCanFrame(c_can_frame, 2);

    // Monitor
    MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_2);
    Notify(message);
  }
}

void MsgReceiver::HandleADHMICANRResultMessageRos(const ::control::CanFrame& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.data.size() > 0) {
    static Uint8_t data_len = (msg.data.data.size() > 8) ? 8 : (msg.data.data.size());
    static can_dev::CanFrame r_can_frame;
    r_can_frame.id = msg.id;
    for(int i = 0; i < data_len; i++) {
      r_can_frame.data[i] = msg.data.data[i];
    }
    ParseCanFrame(r_can_frame, 3);

    // Monitor
    MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_3);
    Notify(message);
  }
}

/**
 * @brief copy from CanAccessDfD17B2::ParseCanFrame 
 * TODO: How to reusing
 */
void MsgReceiver::ParseCanFrame(const can_dev::CanFrame& frame, int channel) {
  Lock lock(lock_parse_can_frame_);

  Chassis_t chassis;
  SpecialChassisInfo_t special_chassis;
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  ChassisDfD17_adrecord adecu_record;
  #endif
  Phoenix_SharedData_GetChassis(&chassis);
  Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis);
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  Phoenix_SharedData_GetChassisDfD17adrecordInfo(&adecu_record);
  #endif
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
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  case (0x18FF739E): {
    // adecu 回放代码 
    unsigned int ad_mode = data[0] & 0x0F;
    unsigned int tar_vel_set = data[2]& 0xFF;
    unsigned int tar_trajectory_type = (data[3] >> 4) & 0x0F;
    unsigned int tar_vel_type = data[3] & 0x0F;
    double vel_obj_dist = data[4];
    double vel_obj_vel = data[5];

    adecu_record.ad_mode = ad_mode;
    adecu_record.tar_vel_set = tar_vel_set;
    adecu_record.tar_trajectory_type = tar_trajectory_type;
    adecu_record.tar_vel_type = tar_vel_type;
    adecu_record.vel_obj_dist = vel_obj_dist;
    adecu_record.vel_obj_vel = vel_obj_vel;

    std::cout<<" ad_mode = "<< ad_mode 
       << " . =" << tar_vel_set 
       << " tar_trajectory_type = " << tar_trajectory_type 
       << " tar_vel_type = " << tar_vel_type 
       << " vel_obj_dist = " << vel_obj_dist 
       << " vel_obj_vel = " << vel_obj_vel << std::endl;
    break;
  }

 case (0x0CFF659E) : {
    unsigned int plan_vel_target = (data[4] << 8) | (data[3]); 
    
    unsigned int plan_ax_target = (data[6] << 8) | (data[5]);
    
    adecu_record.planning_send_vel = (float)plan_vel_target / 100 - 100;

    adecu_record.planning_send_ax = (float)plan_ax_target / 100 - 20;

    std::cout<<" planning_send_vel = "<< adecu_record.planning_send_vel 
    << " planning_send_ax =" << adecu_record.planning_send_ax << std::endl;

    break;
  }       
  #endif                   
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
    // std::cout<<"主转向状态 (0:Not Ready / 1:Ready / 2:Reserve / 3:ADU engaged / 4:ADU engaged Degrade / 5:Error)= "
    //         << (Int32_t)SteeringSystemADModeStatePS<<std::endl;
    break;
  }

  case (0x18FF7BDD) : {
    if (3 == channel) {
      Int8_t Steering_Error_Event3_Active_CEPS = data[5] & 0x03;
      Int8_t Steering_Error_Event3_CEPS = (data[5] & 0xFC) >> 2;
      //std::cout<<"r转向错误 (0:Inactive / 1:acive) = "<<Steering_Error_Event3_Active_CEPS<<std::endl;
      //std::cout<<"r转向错误事件= "<<Steering_Error_Event3_CEPS<<std::endl;

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
    Int8_t PowerOffSwitchOfADCU = (data[2] >> 2) & 0x03;
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
    button_start_adas_.Update(PowerOnSwitchOfADCU, &start_adas, &invalid_adas);
    if(start_adas) {
      special_chassis.start_adas = 2;
      special_chassis.enable_lka = 2;
      special_chassis.enable_acc = 2;
      special_chassis.enable_aeb = 2;
    }
    // 退出自动驾驶
    button_start_adas_.Update(PowerOnSwitchOfADCU, &stop_adas, &invalid_adas);
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
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  /*
  printf("\n set_ad_mode is %d, \
           set_tar_vel_set is s%d \
          , set_tar_trajectory_type is %d \
          , set_tar_vel_type is %d \
          , set_vel_obj_dist is %lf \
          , set_vel_obj_vel is %lf \n",
          adecu_record.ad_mode ,
          adecu_record.tar_vel_set,
          adecu_record.tar_trajectory_type ,
          adecu_record.tar_vel_type,
          adecu_record.vel_obj_dist,
          adecu_record.vel_obj_vel);
  std::cout << "\n set_ad_mode is " << adecu_record.ad_mode << std::endl;
  */
  Phoenix_SharedData_SetChassisDfD17adrecordInfo(&adecu_record);
  #endif
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  Phoenix_SharedData_SetDFCVSpecialChassisInfo(&dfcv_special_chassis);
#endif

}

#endif


#endif // #if (ENABLE_ROS_NODE)


#if (ENABLE_LCM_NODE)
/// Communicate by LCM
void MsgReceiver::HandleImuMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeImuMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size, &lcm_imu_info_)) {

      MessageImu msg(&lcm_imu_info_);
      Notify(msg);
    }
  }
}

#if ENTER_PLAYBACK_MODE
void MsgReceiver::HandleChassisMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size, &lcm_chassis_info_)) {

      MessageChassis msg(&lcm_chassis_info_);
      Notify(msg);
    }
  }
}
#endif

void MsgReceiver::HandlePlanningResultMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodePlanningResultMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_planning_result_)) {

      MessagePlanningResult msg_planning_result(&lcm_planning_result_);
      Notify(msg_planning_result);
    }
  }
}
#endif // #if (ENABLE_LCM_NODE)


#if (ENABLE_UDP_NODE)
void MsgReceiver::HandleImuMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    Phoenix_Common_DecodeImuArray(buf, 0, buf_len, &udp_imu_info_, 1);

    // Update message head
    udp_imu_info_.msg_head.valid = true;
    Phoenix_AdMsg_UpdateSequenceNum(&(udp_imu_info_.msg_head), 1);
    udp_imu_info_.msg_head.timestamp = Phoenix_Common_GetClockNowMs();

    MessageImu msg(&udp_imu_info_);
    Notify(msg);
  }
}

#if ENTER_PLAYBACK_MODE
void MsgReceiver::HandleChassisMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    Phoenix_Common_DecodeChassisArray(buf, 0, buf_len, &udp_chassis_info_, 1);

    // Update message head
    udp_chassis_info_.msg_head.valid = true;
    Phoenix_AdMsg_UpdateSequenceNum(&(udp_chassis_info_.msg_head), 1);
    udp_chassis_info_.msg_head.timestamp = Phoenix_Common_GetClockNowMs();

    MessageChassis msg(&udp_chassis_info_);
    Notify(msg);
  }
}
#endif

void MsgReceiver::HandlePlanningResultMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    Phoenix_Common_DecodePlanningResultArray(
          buf, 0, buf_len, &udp_planning_result_, 1);

    // Update message head
    udp_planning_result_.msg_head.valid = true;
    Phoenix_AdMsg_UpdateSequenceNum(&(udp_planning_result_.msg_head), 1);
    udp_planning_result_.msg_head.timestamp = Phoenix_Common_GetClockNowMs();

    MessagePlanningResult msg_planning_result(&udp_planning_result_);
    Notify(msg_planning_result);
  }
}
#endif // #if (ENABLE_UDP_NODE)


}  // namespace framework
}  // namespace phoenix
