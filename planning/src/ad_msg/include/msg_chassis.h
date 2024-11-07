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
 * @enum
 * @brief 带挂的状态
 */
enum {
  /// 没有带挂
  VEH_TRAILER_STATUS_NOT_CONNECTED = 0,
  /// 带挂
  VEH_TRAILER_STATUS_CONNECTED = 1
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

  /// 总质量有效位
  Int8_t gross_weight_valid;
  /// 总质量 (kg)
  Float32_t gross_weight;

  /// 挂车状态
  Int8_t trailer_status;

  /// 车辆参数
  struct {
    /// 车长有效位
    Int8_t vehicle_length_valid;
    /// 车长，单位：米
    Float32_t vehicle_length;
    /// 车宽有效位
    Int8_t vehicle_width_valid;
    /// 车宽，单位：米
    Float32_t vehicle_width;
    /// 车高有效位
    Int8_t vehicle_height_valid;
    /// 车高，单位：米
    Float32_t vehicle_height;

    /// 挂车长度有效位
    Int8_t trailer_length_valid;
    /// 挂车长度，单位：米
    Float32_t trailer_length;
    /// 挂车宽度有效位
    Int8_t trailer_width_valid;
    /// 挂车宽度，单位：米
    Float32_t trailer_width;
    /// 挂车高度有效位
    Int8_t trailer_height_valid;
    /// 挂车高度，单位：米
    Float32_t trailer_height;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      vehicle_length_valid = 0;
      vehicle_length = 0.0F;
      vehicle_width_valid = 0;
      vehicle_width = 0.0F;
      vehicle_height_valid = 0;
      vehicle_height = 0.0F;

      trailer_length_valid = 0;
      trailer_length = 0.0F;
      trailer_width_valid = 0;
      trailer_width = 0.0F;
      trailer_height_valid = 0;
      trailer_height = 0.0F;
    }
  } param;
  Float32_t trailer_l;
  Float32_t trailer_w;
  Float32_t trailer_h;

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

    gross_weight_valid = false;
    gross_weight = 0.0F;

    trailer_status = VEH_TRAILER_STATUS_NOT_CONNECTED;

    trailer_l = 0.0F;
    trailer_w = 0.0F;
    trailer_h = 0.0F;
		param.Clear();
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

struct ChassisDfD17 {
  /// BCM状态   0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
  Int8_t BCM_state;  
  /// VECU状态  0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
  Int8_t VECU_state;
  /// 主转向状态   0:Not Ready / 1:Ready / 2:Reserve / 3:ADU engaged / 4:ADU engaged Degrade / 5:Error
  Int8_t EPS_state;
  /// 冗余转向状态1  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  Int8_t CEPS1_state;
  /// 冗余转向状态2  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  Int8_t CEPS2_state;
  /// 主刹车状态   0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  Int8_t EBS_state;
  /// 冗余刹车状态  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  Int8_t CEBS_state;
  /// For AD-ECU
  /// AD-ECU Driving mode   1:Power off / 2:Not Ready / 3:Ready / 4:Engage / 5:Fallback
  Int8_t ADCU_Mode_Sts;
  /// 降级是否生效
  Int8_t enable_fallback;
  /// 目标降级是否有效
  Int8_t target_fallback_valid;
  /// 降级状态 0：No fallbcak / 1: A / 2: B / 4: C / 8: D
  Int8_t AD_fallback_level;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    BCM_state = 0;
    VECU_state = 0;
    EPS_state = 0;
    CEPS1_state = 0;
    CEPS2_state = 0;
    EBS_state = 0;
    CEBS_state = 0;
    ADCU_Mode_Sts = 0;
    enable_fallback = 0;
    target_fallback_valid = 0;
    AD_fallback_level = 0;
  }

  /**
   * @brief 构造函数
   */
  ChassisDfD17() {
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

  /// LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_lka;
  /// ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_acc;
  /// AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_aeb;

  /// 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_alc;
  /// 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_isl;

  /// Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_ngp;

  /// 目标车速[m/s]
  Int8_t target_velocity_valid;
  Float32_t target_velocity;
  /// 加速度[m/s^2]
  Int8_t target_acc_valid;
  Float32_t target_acc;
  /// 时距[s]
  Int8_t target_time_gap_valid;
  Float32_t target_time_gap;

  /// 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  Int32_t changing_lane_req;

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
  ChassisDfD17 dfcv_d17;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
    start_adas = 0;

    /// LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_lka = 0;
    /// ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_acc = 0;
    /// AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_aeb = 0;

    /// 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_alc = 0;
    /// 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_isl = 0;

    /// Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
    enable_ngp = 0;

    /// 目标车速[m/s]
    target_velocity_valid = 0;
    target_velocity = 0;
    /// 加速度[m/s^2]
    target_acc_valid = 0;
    target_acc = 0;
    /// 时距[s]
    target_time_gap_valid = 0;
    target_time_gap = 0;

    /// 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
    changing_lane_req = 0;

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

    dfcv_d17.Clear();
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


