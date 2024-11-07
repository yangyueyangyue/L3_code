/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_chassis.h
 * @brief      车身消息定义
 * @details    定义了车身消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_CHASSIS_H_
#define PHOENIX_AD_MSG_MSG_CHASSIS_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @enum
 * @brief 急停信号的状态
 */
enum {
  /// 无效的状态
  VEH_E_STOP_INVALID = 0,
  /// 急停信号 OFF
  VEH_E_STOP_OFF,
  /// 急停信号 ON
  VEH_E_STOP_ON
};

/**
 * @enum
 * @brief 车辆自动控制的状态
 */
enum {
  /// 无效的状态
  VEH_DRIVING_MODE_INVALID = 0,
  /// 人工驾驶模式
  VEH_DRIVING_MODE_MANUAL,
  /// 自动驾驶模式
  VEH_DRIVING_MODE_ROBOTIC
};

/**
 * @enum
 * @brief 方向盘控制状态
 */
enum {
  /// 无效的状态
  VEH_EPS_STATUS_INVALID = 0,
  /// 人工驾驶模式
  VEH_EPS_STATUS_MANUAL,
  /// 自动驾驶模式
  VEH_EPS_STATUS_ROBOTIC,
  /// Manual interrupt
  VEH_EPS_STATUS_MANUAL_INTERRUPT,
  /// Error
  VEH_EPS_STATUS_ERROR
};

/**
 * @enum
 * @brief 油门系统控制状态
 */
enum {
  /// 无效的状态
  VEH_THROTTLE_SYS_STATUS_INVALID = 0,
  /// 人工驾驶模式
  VEH_THROTTLE_SYS_STATUS_MANUAL,
  /// 自动驾驶模式
  VEH_THROTTLE_SYS_STATUS_ROBOTIC,
  /// Error
  VEH_THROTTLE_SYS_STATUS_ERROR
};

/**
 * @enum
 * @brief 制动系统控制状态
 */
enum {
  /// 无效的状态
  VEH_EBS_STATUS_INVALID = 0,
  /// 人工驾驶模式
  VEH_EBS_STATUS_MANUAL,
  /// 自动驾驶模式
  VEH_EBS_STATUS_ROBOTIC,
  /// Error
  VEH_EBS_STATUS_ERROR
};

/**
 * @enum
 * @brief 驻车系统的状态
 */
enum {
  /// 无效的状态
  VEH_EPB_STATUS_INVALID = 0,
  /// 驻车未启动
  VEH_EPB_STATUS_OFF,
  /// 驻车启动
  VEH_EPB_STATUS_ON
};

/**
 * @enum
 * @brief 档位的状态
 */
enum {
  /// 无效的状态
  VEH_GEAR_INVALID = 0,
  /// P 档
  VEH_GEAR_P,
  /// N 档
  VEH_GEAR_N,
  /// R 档
  VEH_GEAR_R,
  /// D 档
  VEH_GEAR_D
};

/**
 * @enum
 * @brief 灯光的状态
 */
enum {
  /// 无效的状态
  VEH_LAMP_INVALID = 0,
  /// 灯关闭
  VEH_LAMP_OFF,
  /// 灯打开
  VEH_LAMP_ON
};

/**
 * @enum
 * @brief 转向灯的状态
 */
enum {
  /// 无效的状态
  VEH_TURN_LAMP_INVALID = 0,
  /// 转向灯关闭
  VEH_TURN_LAMP_OFF,
  /// 左转向灯开
  VEH_TURN_LAMP_LEFT,
  /// 右转向灯开
  VEH_TURN_LAMP_RIGHT,
  /// 应急灯开
  VEH_TURN_LAMP_EMERGENCY
};

/**
 * @enum
 * @brief 转向拨杆的状态
 */
enum {
  /// 无效的状态
  VEH_TURNING_INDICATOR_INVALID = 0,
  /// 无指示
  VEH_TURNING_INDICATOR_NONE,
  /// 左转指示
  VEH_TURNING_INDICATOR_LEFT,
  /// 右转指示
  VEH_TURNING_INDICATOR_RIGHT
};

/**
 * @enum
 * @brief 雨刮器的状态
 */
enum {
  /// 无效的状态
  VEH_WIPER_INVALID = 0,
  /// 关闭
  VEH_WIPER_OFF,
  /// SHORT PRESS WITH CLICK
  VEH_WIPER_SHORT_PRESS_WITH_CLICK,
  /// LONG PRESS WITH CLICK
  VEH_WIPER_LONG_PRESS_WITH_CLICK,
  /// INT 1
  VEH_WIPER_INT_1,
  /// INT 2
  VEH_WIPER_INT_2,
  /// INT 3
  VEH_WIPER_INT_3,
  /// INT 4
  VEH_WIPER_INT_4,
  /// LO
  VEH_WIPER_LO,
  /// HI
  VEH_WIPER_HI
};


/**
 * @struct Chassis
 * @brief 车身信息
 */
struct Chassis {
  /// 消息头
  MsgHead msg_head;
  /// 驾驶模式
  Int8_t driving_mode;
  /// 紧急停止信号
  Int8_t e_stop;

  /// 方向盘控制状态
  Int8_t eps_status;
  /// 油门系统控制状态
  Int8_t throttle_sys_status;
  /// 制动系统控制状态
  Int8_t ebs_status;

  /// 方向盘角度信号是否有效
  Int8_t steering_wheel_angle_valid;
  /// 方向盘角度 (rad)
  Float32_t steering_wheel_angle;
  /// 方向盘转速有效位
  Int8_t steering_wheel_speed_valid;
  /// 方向盘转速 (rad/s)
  Float32_t steering_wheel_speed;
  /// 实际转向扭矩有效位
  Int8_t steering_wheel_torque_valid;
  /// 实际转向扭矩(N.m)
  Float32_t steering_wheel_torque;

  /// 车速信号是否有效
  Int8_t v_valid;
  /// 车辆速度,m/s
  Float32_t v;
  /// 车辆加速度信号是否有效
  Int8_t a_valid;
  /// 车辆加速度,m/s^2
  Float32_t a;
  /// 角速度有效位
  Int8_t yaw_rate_valid;
  /// 角速度,rad/s
  Float32_t yaw_rate;
  /// AX 有效位
  Int8_t ax_valid;
  /// AX(m/s^2)
  Float32_t ax;
  /// AY 有效位
  Int8_t ay_valid;
  /// AY(m/s^2)
  Float32_t ay;

  /// 左前轮速有效位
  Int8_t wheel_speed_fl_valid;
  /// 左前轮速(m/s)
  Float32_t wheel_speed_fl;
  /// 右前轮速有效位
  Int8_t wheel_speed_fr_valid;
  /// 右前轮速(m/s)
  Float32_t wheel_speed_fr;
  /// 左后轮速有效位
  Int8_t wheel_speed_rl_valid;
  /// 左后轮速(m/s)
  Float32_t wheel_speed_rl;
  /// 右后轮速有效位
  Int8_t wheel_speed_rr_valid;
  /// 右后轮速(m/s)
  Float32_t wheel_speed_rr;
  /// 左后2#轮速有效位
  Int8_t wheel_speed_rl2_valid;
  /// 左后2#轮速(m/s)
  Float32_t wheel_speed_rl2;
  /// 右后2#轮速有效位
  Int8_t wheel_speed_rr2_valid;
  /// 右后2#轮速(m/s)
  Float32_t wheel_speed_rr2;

  /// 驻车系统状态
  Int8_t epb_status;
  /// 当前档位
  Int8_t gear;
  /// 当前档位 Number
  Int8_t gear_number;
  /// 转向拨杆信号
  Int8_t signal_turning_indicator;
  /// 转向灯信号
  Int8_t signal_turn_lamp;
  /// 制动灯信号
  Int8_t signal_brake_lamp;
  /// 制动踏板深度（百分比）, invalid if less then 0
  Int8_t brake_pedal_value;
  /// 油门踏板深度（百分比）, invalid if less then 0
  Int8_t acc_pedal_value;

  /// 发动机转速有效位
  Int8_t engine_speed_valid;
  /// 发动机转速
  Float32_t engine_speed;
  /// 发动机转矩有效位
  Int8_t engine_torque_valid;
  /// 发动机转矩(N.m)
  Float32_t engine_torque;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    driving_mode = VEH_DRIVING_MODE_INVALID;
    e_stop = VEH_E_STOP_INVALID;

    eps_status = VEH_EPS_STATUS_INVALID;
    throttle_sys_status = VEH_THROTTLE_SYS_STATUS_INVALID;
    ebs_status = VEH_EBS_STATUS_INVALID;

    steering_wheel_angle_valid = 0;
    steering_wheel_angle = 0;
    steering_wheel_speed_valid = false;
    steering_wheel_speed = 0;
    steering_wheel_torque_valid = false;
    steering_wheel_torque = 0;

    v_valid = 0;
    v = 0;
    a_valid = 0;
    a = 0;
    yaw_rate_valid = 0;
    yaw_rate = 0;
    ax_valid = false;
    ax = 0;
    ay_valid = false;
    ay = 0;

    wheel_speed_fl_valid = false;
    wheel_speed_fl = 0;
    wheel_speed_fr_valid = false;
    wheel_speed_fr = 0;
    wheel_speed_rl_valid = false;
    wheel_speed_rl = 0;
    wheel_speed_rr_valid = false;
    wheel_speed_rr = 0;
    wheel_speed_rl2_valid = false;
    wheel_speed_rl2 = 0;
    wheel_speed_rr2_valid = false;
    wheel_speed_rr2 = 0;

    epb_status = VEH_EPB_STATUS_INVALID;
    gear = VEH_GEAR_INVALID;
    gear_number = 0;
    signal_turning_indicator = VEH_TURNING_INDICATOR_INVALID;
    signal_turn_lamp = VEH_TURN_LAMP_INVALID;
    signal_brake_lamp = VEH_LAMP_INVALID;
    brake_pedal_value = 0;
    acc_pedal_value = 0;

    engine_speed_valid = false;
    engine_speed = 0;
    engine_torque_valid = false;
    engine_torque = 0;
  }

  /**
   * @brief 构造函数
   */
  Chassis() {
    Clear();
  }
};


/**
 * @struct ChassisCtlCmd
 * @brief 向车身发送的控制指令
 */
struct ChassisCtlCmd {
  /// 消息头
  MsgHead msg_head;
  /// 开始自动驾驶命令, 0 ~ 无请求, 1 ~ 关闭, 2 ~ 开启
  Int8_t start_robotic_ctl;
  /// 使能转向控制系统
  Int8_t enable_eps;
  /// 使能油门控制系统
  Int8_t enable_throttle_sys;
  /// 使能制动控制系统
  Int8_t enable_ebs;

  /// 使能远程控制
  Int8_t enable_remote_ctl;
  /// 直接控制车身
  Int8_t enable_direct_ctl;
  /// 使能速度控制
  Int8_t enable_acc;
  /// 释放油门控制
  Int8_t release_throttle;

  /// 方向盘角度指令(rad)
  Float32_t steering_wheel_angle;
  /// 方向盘角速度指令(rad/s)
  Float32_t steering_wheel_speed;
  /// 方向盘扭矩指令(N/m)
  Float32_t steering_wheel_torque;
  /// 速度(m/s)
  Float32_t velocity;
  /// 加速度(m/s2)
  Float32_t acceleration;
  /// 加速量
  Float32_t acc_value;
  /// 刹车量
  Float32_t brake_value;
  /// 档位
  Int8_t gear;
  /// 转向灯信号
  Int8_t turn_lamp;
  /// 制动灯信号
  Int8_t brake_lamp;
  /// 雨刮器状态
  Int8_t wiper;
  /// 驻车系统状态
  Int8_t epb_status;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    start_robotic_ctl = 0;

    enable_eps = 0;
    enable_throttle_sys = 0;
    enable_ebs = 0;

    enable_remote_ctl = 0;
    enable_direct_ctl = 0;
    enable_acc = 0;
    release_throttle = 0;

    steering_wheel_angle = 0;
    steering_wheel_speed = 0;
    steering_wheel_torque = 0;

    velocity = 0;
    acceleration = 0;
    acc_value = 0;
    brake_value = 0;

    gear = VEH_GEAR_INVALID;
    turn_lamp = VEH_TURN_LAMP_INVALID;
    brake_lamp = VEH_LAMP_INVALID;
    wiper = VEH_WIPER_INVALID;
    epb_status = VEH_EPB_STATUS_INVALID;
  }

  /**
   * @brief 构造函数
   */
  ChassisCtlCmd() {
    Clear();
  }
};


/**
 * @struct ChassisFtAuman
 * @brief 具体车型的特殊信息 (FT-Auman)
 */
struct ChassisFtAuman {
  /// TJA功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_tja;
  /// HWA功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_hwa;
  /// I-Drive功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_i_drive;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    switch_tja = 0;
    switch_hwa = 0;
    switch_i_drive = 0;
  }

  /**
   * @brief 构造函数
   */
  ChassisFtAuman() {
    Clear();
  }
};

/**
 * @struct SpecialChassisInfo
 * @brief 从车身获取的其它信息
 */
struct SpecialChassisInfo {
  /// 消息头
  MsgHead msg_head;
  /// 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  Int8_t start_adas;

  /// CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  Int8_t cnt_stu_frame_loss_can0;
  Int8_t cnt_stu_frame_loss_can1;
  Int8_t cnt_stu_frame_loss_can2;
  Int8_t cnt_stu_frame_loss_can3;

  /// 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  Int8_t cnt_stu_gtw_to_veh_can0;
  Int8_t cnt_stu_gtw_to_veh_can1;
  Int8_t cnt_stu_gtw_to_veh_can2;
  Int8_t cnt_stu_gtw_to_veh_can3;

  /// 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  Int8_t cnt_stu_ctl_to_gtw_can0;
  Int8_t cnt_stu_ctl_to_gtw_can1;
  Int8_t cnt_stu_ctl_to_gtw_can2;
  Int8_t cnt_stu_ctl_to_gtw_can3;

  /// 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  Int8_t cnt_stu_ctl_to_gtw;

  /// 具体车型的特殊信息 (FT-Auman)
  ChassisFtAuman ft_auman;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
    start_adas = 0;

    // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
    cnt_stu_frame_loss_can0 = 0;
    cnt_stu_frame_loss_can1 = 0;
    cnt_stu_frame_loss_can2 = 0;
    cnt_stu_frame_loss_can3 = 0;

    // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
    cnt_stu_gtw_to_veh_can0 = 0;
    cnt_stu_gtw_to_veh_can1 = 0;
    cnt_stu_gtw_to_veh_can2 = 0;
    cnt_stu_gtw_to_veh_can3 = 0;

    // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
    cnt_stu_ctl_to_gtw_can0 = 0;
    cnt_stu_ctl_to_gtw_can1 = 0;
    cnt_stu_ctl_to_gtw_can2 = 0;
    cnt_stu_ctl_to_gtw_can3 = 0;

    // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
    cnt_stu_ctl_to_gtw = 0;

    // 具体车型的特殊信息 (FT-Auman)
    ft_auman.Clear();
  }

  /**
   * @brief 构造函数
   */
  SpecialChassisInfo() {
    Clear();
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_CHASSIS_H_


