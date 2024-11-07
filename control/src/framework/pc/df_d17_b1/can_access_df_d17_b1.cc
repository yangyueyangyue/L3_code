//
#include "pc/df_d17_b1/can_access_df_d17_b1.h"

#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "communication_c/shared_data_c.h"


namespace phoenix {
namespace framework {
namespace df_d17_b1 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)

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

CanAccessDfD17B1::CanAccessDfD17B1(Task* manager)
  : Task(TASK_ID_CAN_ACCESS, "Can Access", manager) {
  running_flag_can_device_ = false;

  steering_wheel_angle_offset_ = common::com_deg2rad(-20.0F);

  frame_alive_count_0x413_ = 0;
}

CanAccessDfD17B1::~CanAccessDfD17B1() {
  Stop();
}

bool CanAccessDfD17B1::Start() {
  if (running_flag_can_device_) {
    return (true);
  }

  LOG_INFO(3) << "Open CAN Device...";
  //#if ENABLE_CAN_DEV_KVASER
  //  if (!can_dev::CanDriverKvaser::OpenDevice()) {
  //    LOG_ERR << "Open CAN device failed.";
  //    return (false);
  //  }
  //#elif ENABLE_CAN_DEV_ZLGCANNET
  if (!can_dev::CanDriverZlgCanNet::OpenDevice()) {
    LOG_ERR << "Open CAN device failed.";
    return (false);
  }
  //#endif

  can_dev::CanChannelParam can_dev_param;
  can_dev_param.channel = 4;
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = 4005;
  can_dev_param.can_net.notify_port = 6005;
  if (!can_channel_4_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_1 failed.";
    goto ERROR_01;
  }

  can_dev_param.channel = 5;
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = 4006;
  can_dev_param.can_net.notify_port = 6006;
  if (!can_channel_5_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_2 failed.";
    goto ERROR_02;
  }

  can_dev_param.channel = 6;
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = 4007;
  can_dev_param.can_net.notify_port = 6007;
  if (!can_channel_6_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_3 failed.";
    goto ERROR_03;
  }

  can_dev_param.channel = 7;
  can_dev_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_dev_param.can_net.ip_addr, 30, "192.168.1.177");
  can_dev_param.can_net.port = 4008;
  can_dev_param.can_net.notify_port = 6008;
  if (!can_channel_7_.OpenChannel(can_dev_param)) {
    LOG_ERR << "Open CAN send_channel_4 failed.";
    goto ERROR_04;
  }

  LOG_INFO(3) << "Start CAN Message Receive Thread (channel 0 & 1 & 2)...";
  running_flag_can_device_ = true;
  thread_can_recv_channel_4_ = boost::thread
      (boost::bind(&CanAccessDfD17B1::ThreadReceivingChannel1, this));
  thread_can_recv_channel_5_ = boost::thread
      (boost::bind(&CanAccessDfD17B1::ThreadReceivingChannel2, this));
  thread_can_recv_channel_6_ = boost::thread
      (boost::bind(&CanAccessDfD17B1::ThreadReceivingChannel3, this));
  thread_can_recv_channel_7_ = boost::thread
      (boost::bind(&CanAccessDfD17B1::ThreadReceivingChannel4, this));

  frame_alive_count_0x0CFF649E_ = 0;
  frame_alive_count_0x413_ = 0;
  frame_alive_count_xbr_brake_req_ = 0;

  LOG_INFO(3) << "Open CAN Device... [OK]";

  return (true);

ERROR_01:
  can_channel_7_.CloseChannel();
ERROR_02:
  can_channel_5_.CloseChannel();
ERROR_03:
  can_channel_6_.CloseChannel();
ERROR_04:
  can_channel_7_.CloseChannel();
  //#if ENABLE_CAN_DEV_KVASER
  //  can_dev::CanDriverKvaser::CloseDevice();
  //#elif ENABLE_CAN_DEV_ZLGCANNET
  can_dev::CanDriverZlgCanNet::CloseDevice();
  //#endif

  LOG_INFO(3) << "Open CAN Device... [NG]";

  return (false);
}

bool CanAccessDfD17B1::Stop() {
  if (running_flag_can_device_) {
    running_flag_can_device_ = false;

    LOG_INFO(3) << "Stop CAN Receiving Thread (channel 7) ...";
    bool ret = thread_can_recv_channel_7_.timed_join(
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

    LOG_INFO(3) << "Stop CAN Receiving Thread (channel 6) ...";
    ret = thread_can_recv_channel_6_.timed_join(
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

    LOG_INFO(3) << "Stop CAN Receiving Thread (channel 5) ...";
    ret = thread_can_recv_channel_5_.timed_join(
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

    LOG_INFO(3) << "Stop CAN Receiving Thread (channel 4) ...";
    ret = thread_can_recv_channel_4_.timed_join(
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
    can_channel_7_.CloseChannel();
    can_channel_6_.CloseChannel();
    can_channel_5_.CloseChannel();
    can_channel_4_.CloseChannel();
    //  #if ENABLE_CAN_DEV_KVASER
    //    can_dev::CanDriverKvaser::CloseDevice();
    //  #elif ENABLE_CAN_DEV_ZLGCANNET
    can_dev::CanDriverZlgCanNet::CloseDevice();
    //  #endif
  }

  return (true);
}

void CanAccessDfD17B1::ThreadReceivingChannel1() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 1) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_4_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_0);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 1) ... [Stopped]";
}

void CanAccessDfD17B1::ThreadReceivingChannel2() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 2) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_5_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_1);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 2) ... [Stopped]";
}

void CanAccessDfD17B1::ThreadReceivingChannel3() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_6_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_2);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Stopped]";
}

void CanAccessDfD17B1::ThreadReceivingChannel4() {
  LOG_INFO(3) << "CAN Receiving Thread (channel 4) ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_can_device_) {
    frame_num = can_channel_7_.ReadWait(&can_frame);
    if (1 == frame_num) {
      ParseCanFrame(can_frame);

      // Monitor
      MessageModuleStatus message(MSG_ID_MODULE_STATUS_CAN_RECV_CH_3);
      Notify(message);
    }
  }

  LOG_INFO(3) << "CAN Receiving Thread (channel 3) ... [Stopped]";
}

void CanAccessDfD17B1::ParseCanFrame(const can_dev::CanFrame& frame) {
  Lock lock(lock_parse_can_frame_);

  Chassis_t chassis;
  SpecialChassisInfo_t special_chassis;
  Phoenix_SharedData_GetChassis(&chassis);
  Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis);

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
    break;
  }
  case (0x18FEBF0B): {
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
    }
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

  case 0x0CF00400: {
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
    // vehicle_status_.EngineDriverRequestTorqueRate = driver_request_drive_torque_rate;

    // 实际发动机-转矩百分比, 1%, -125%, -125% ~ +125%
    float actual_engine_torque_rate = data[2] - 125.0f;
    // vehicle_status_.EngineTorqueRate = actual_engine_torque_rate;

    // 发动机转速, 0.125rpm, 0, 0 ~ 8031.875rpm
    //vehicle_status_.EngineSpeedStatus = VEH_DATA_STATUS_VALID;
    float engine_speed = ((unsigned int)(data[3]) | (unsigned int)(data[4] << 8)) * 0.125f;
    chassis.engine_speed = engine_speed;
    chassis.engine_speed_valid = 1;

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

    break;
  }

  case (0x18ECFF00): {
    // 发动机: EC1-发动机配置1
    // 发动机空转转速, 0.125 rpm/bit, 0 ~ 8,031.875 rpm
    float engine_idling_speed = ((unsigned int)(data[0]) | (unsigned int)(data[1] << 8)) * 0.125f;
    // 参考发动机转矩, 1Nm per bit, 0 ~ 64255 Nm

    break;
  }

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

  case (0x425): {
    // AHPS-ECU Control information 1
    Uint8_t check_sum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6];
    if (check_sum == data[7]) {
      // Steering angle, CW: minus, CCW: plus, -3276.8~3276.8 deg
      Float32_t steering_angle = (Int16_t)((Uint32_t)data[0] | (Uint32_t)(data[1] << 8)) * 0.1F;
      // Steering angle speed, -2000~2000 deg/s
      Float32_t steering_angle_speed = (Int16_t)((Uint32_t)(data[2]) | (Uint32_t)(data[3] << 8)) * 0.1F;
      // Steering Torque, CW: minus, CCW: plus, -20.0~20.0 Nm
      Float32_t steering_torque = (Int16_t)((Uint32_t)(data[4]) | (Uint32_t)(data[5] << 8)) * 0.1F;

      chassis.steering_wheel_angle_valid = 1;
      chassis.steering_wheel_angle =
          common::com_deg2rad(steering_angle) - steering_wheel_angle_offset_;
      chassis.steering_wheel_speed_valid = 1;
      chassis.steering_wheel_speed = common::com_deg2rad(steering_angle_speed);
      chassis.steering_wheel_torque_valid = 1;
      chassis.steering_wheel_torque = steering_torque;
    } else {
      LOG_ERR << "Check sum error, required(" << data[7] << ") actual(" << check_sum << ").";
    }

    break;
  }

    // AHPS-ECU Control information 2
  case (0x426): {
    Uint8_t control_mode_response = 0;
    Uint8_t check_sum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6];
    if (check_sum == data[7]) {
      // LKAS demand response: 0 ~ Control OFF, 1 ~ Control ON, 2 ~ Limited control
      Uint8_t demand_response = data[0] & 0x0F;
      // Control mode response
      // 0: Steering assist mode
      // 1: Control off
      // 2: Troque control mode with active return with override
      // 3: Angle control mode with override
      // 4: Troque control mode without active return with override
      // 5: Troque control mode with active return without override
      // 6: Angle control mode without override
      // 7: Troque control mode without active return without override
      control_mode_response = (data[0] & 0xF0) >> 4;
      // std::cout<<"demand_response == "<<(int)demand_response<<std::endl;
      // std::cout<<"control_mode_response == "<<(int)control_mode_response<<std::endl;
      // std::cout<<"=============="<<std::endl;
    } else {
      LOG_ERR << "Check sum error, required(" << data[7] << ") actual(" << check_sum << ").";
    }
    if (control_mode_response == 3) {
      chassis.eps_status = VEH_EPS_STATUS_ROBOTIC;
    }else if (control_mode_response == 0) {
      chassis.eps_status = VEH_EPS_STATUS_MANUAL;
    }else{
      chassis.eps_status = VEH_EPS_STATUS_INVALID;
    }
    
    break;
  }

    ///转向故障码
  case (0x427): {
    Uint16_t failure_code =  (Uint32_t)(data[0]) | (Uint32_t)(data[1] << 8);
    break;
  }

  default:
    break;
  }

  chassis.msg_head.valid = 1;
  Phoenix_SharedData_SetChassis(&chassis);
  Phoenix_SharedData_SetSpecialChassisInfo(&special_chassis);
}


// 上电发送报文
void CanAccessDfD17B1::SendFrame0x0CFF649E(Uint8_t enable, Uint8_t engage) {
  can_dev::CanFrame frame;
  frame.id = 0x0CFF649E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  // 1.整车上电，BCM和VECU状态Not Ready
  // 2.PC上电，分别向BCAN 和 PCAN 发送Enable 为1，需满足E2E规范，发送周期为20ms,请参考 AutoDrive_ADCU
  // 3.待BCM 和 VECU状态为为1，发送Engage 为1，需满足E2E规范，发送周期为20ms，请参考 AutoDrive_ADCU
  // 4.待BCM 和 VECU状态为为2后，BCM 和 VECU均可控
  // 5.AutoDrive_ADCU 信号需要发送到BCAN 和 PCAN

  // 0x0 :inhibit, 0x1 :enable, 0x2 :fail, 0x3 :off
  Uint8_t ADModeEnablePS = enable;
  // 0x0 :inhibit, 0x1 :engage, 0x2 :reserve, 0x3 :reserve
  Uint8_t ADModeEngagePS = engage;

  // 信息计数器
  Uint8_t count = (Uint8_t)(frame_alive_count_0x0CFF649E_++);
  if (frame_alive_count_0x0CFF649E_ > 15) {
    frame_alive_count_0x0CFF649E_ = 0;
  }

  frame.data[1] = count;
  frame.data[2] = (ADModeEnablePS & 0x03) | ((ADModeEngagePS & 0x03) << 4);
  frame.data[3] = (0x00);
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = (0x00);
  frame.data[7] = (0x00);
  frame.data[0] = CalculateCrc8(frame.data);

  // Pcan和Bcan发送
  can_channel_4_.Send(&frame);
  can_channel_5_.Send(&frame);
}

// XBR-制动需求
void CanAccessDfD17B1::SendFrame0x0C040B9E(const ChassisCtlCmd_t& cmd, Uint8_t enable) {
  can_dev::CanFrame frame;
  frame.id = 0x0C040B9E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

  // 外部加速要求，1/2048m/s^2，(-10)~(+10)m/s^2，偏移：-15.69m/s2
  // unsigned short external_accelerate_requirement = (-cmd_.BrakeValue *0.01f*10.0f + 15.687f) * 2048.0f;
  Uint16_t external_accelerate_requirement = (-cmd.brake_value * 0.01F * 10.0F + 15.687F) * 2048.0F;
  if (cmd.brake_value < 1.0F) {
    external_accelerate_requirement = 0x7D7F;
  }
  // 外部制动要求EBI模式，00-不允许耐久制动集成，01-只允许耐久制动，10-允许耐久制动集成，11-未定义
  Uint8_t external_brake_requirement_ebi_mode = 0x00;
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
  //                            在驾驶员需求减速度与外部制动请求减速度之间取最大的减速度请求。
  // Uint8_t external_brake_requirement_control_mode = 0x02;
  Uint8_t external_brake_requirement_control_mode = enable;
  // 外部制动需求紧迫性，0.4%，1%~100%
  // Uint8_t external_brake_requirement_urgency = (unsigned char)(100*2.5f);
  Uint8_t external_brake_requirement_urgency = 0x00;
  // 信息计数器
  Uint8_t count = (Uint8_t)(frame_alive_count_xbr_brake_req_++);
  if (frame_alive_count_xbr_brake_req_ > 15) {
    frame_alive_count_xbr_brake_req_ = 0;
  }

  // printf("external_accelerate_requirement == %x,\n", external_accelerate_requirement);
  frame.data[0] = (external_accelerate_requirement & 0xFF);
  frame.data[1] = (external_accelerate_requirement >> 8);
  frame.data[2] = (external_brake_requirement_ebi_mode & 0x03) |
      ((external_brake_requirement_priority & 0x03) << 2) |
      ((external_brake_requirement_control_mode & 0x03) << 4) |
      (0x03 << 6);
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

  // Pcan和Bcan和Ccan发送。
  can_channel_4_.Send(&frame);
  can_channel_5_.Send(&frame);
  can_channel_6_.Send(&frame);
}

// 油门档位下发
void CanAccessDfD17B1::SendFrame0x0CFF659E(const ChassisCtlCmd_t& cmd, Uint8_t gear_mode) {
  can_dev::CanFrame frame;
  Uint8_t transmission_requested_gear = 0;
  // 加速踏板开度下发 精度0.4
  Uint8_t accelerater_pedal_position_demand = cmd.acc_value * 2.5F;
  if (cmd.brake_value > 0.1F) {
    accelerater_pedal_position_demand = 0;
  }
  
  frame.id = 0x0CFF659E;
  frame.RTR = false;
  frame.EXT = true;
  frame.data_len = 8;

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
  
  // 经济模式/动力模式, 0x0:E, 0x1:P, 0x2:Reserved, 0x3:Take no action
  Uint8_t E_P_mode_demand = 0;
  // 蠕动模式, 0x0:DM, 0x1:RM, 0x2:Reserved, 0x3:Take no action
  Uint8_t creep_mode = 0;
  // 辅助制动模式, 0x0:OFF, 0x1:恒速档, 0x2:辅助制动1档, 0x3:助制动2档, 0x4:辅助制动3档
  Uint8_t auxiliary_braking_mode = 1;

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

  // Pcan发送
  can_channel_4_.Send(&frame);
}


// AHPS Activation (IGN information)
void CanAccessDfD17B1::SendFrame0x412() {
  can_dev::CanFrame frame;
  frame.id = 0x412;
  frame.RTR = false;
  frame.EXT = false;
  frame.data_len = 8;

  // 转向前提条件：ign_status 为1, Straight_flag 为0
  // IGN state, 0:IGN OFF, 1:IGN ON, voltage >= 20V, 2:IGN ON, voltage < 20V
  Uint8_t ign_status = 1;
  // Straight flag, 0:normal, 1:go straght
  Uint8_t straight_flag = 0x00;

  frame.data[0] =(ign_status & 0x0F);
  frame.data[1] = (0x00);
  frame.data[2] = (0x00);
  frame.data[3] = straight_flag;
  frame.data[4] = (0x00);
  frame.data[5] = (0x00);
  frame.data[6] = (0x00);
  frame.data[7] = (0x00);

  // EHPS can发送
  can_channel_7_.Send(&frame);
}

// LKAS master ECU Control demand
void CanAccessDfD17B1::SendFrame0x413(
    const ChassisCtlCmd_t& cmd, Uint8_t ctl_mode) {
  can_dev::CanFrame frame;
  frame.id = 0x413;
  frame.RTR = false;
  frame.EXT = false;
  frame.data_len = 8;

  // LKAS demand, 0:No demand, 1:demand with override judgement,
  //              2:demand without override judgement
  Uint8_t lkas_demand = 0x00;
  // Control mode demand
  // 0:Steering assist on
  // 1:Steering assist off
  // 2:Torque demand mode with active return
  // 3:Angle demand mode
  // 4:torque demand mode without active return
  Uint8_t control_mode_demand = 0x01;

  switch (ctl_mode) {
  case 0x00:
    lkas_demand = 0x00;
    control_mode_demand = 0x01;
    break;
  case 0x01:
    lkas_demand = 0x01;
    control_mode_demand = 0x03;
    break;
  case 0x02:
    lkas_demand = 0x00;
    control_mode_demand = 0x00;
    break;
  default:
    lkas_demand = 0x00;
    control_mode_demand = 0x01;
    break;
  }
  // std::cout<<"send lkas_demand = "<<(int)lkas_demand<<std::endl;
  // std::cout<<"send control_mode_demand = "<<(int)control_mode_demand<<std::endl;

  // Torque demand, -327.68~327.68 Nm, CW: minus, CCW: plus
  Int16_t torque_demand = (Int16_t)(cmd.steering_wheel_torque * 100);
  // Steering angle, -3276.8~3276.8 deg,
  Int16_t steering_angle_request =
      (Int16_t)(phoenix::common::com_rad2deg(
                  cmd.steering_wheel_angle + steering_wheel_angle_offset_) * 10);

  frame.data[0] = (torque_demand & 0xFF);
  frame.data[1] = ((torque_demand >> 8) & 0xFF);
  frame.data[2] = (steering_angle_request & 0xFF);
  frame.data[3] = ((steering_angle_request >> 8) & 0xFF);
  frame.data[4] = lkas_demand;
  frame.data[5] = control_mode_demand;
  frame.data[6] = frame_alive_count_0x413_++;
  frame.data[7] = frame.data[0] + frame.data[1] + frame.data[2] +
      frame.data[3] + frame.data[4] + frame.data[5] + frame.data[6];

  if(frame_alive_count_0x413_ > 255){
    frame_alive_count_0x413_ = 0;
  }

  // 发送
  can_channel_7_.Send(&frame);
}

#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)


}  // namespace df_d17_b1
}  // namespace framework
}  // namespace phoenix
