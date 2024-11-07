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

#ifndef PHOENIX_AD_MSG_MSG_CHASSIS_C_H_
#define PHOENIX_AD_MSG_MSG_CHASSIS_C_H_

#include "utils/macros.h"
#include "utils/com_utils_c.h"
#include "msg_common_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief 底盘控制的状态
 */
enum {
  VEH_CHASSIS_CTL_STATUS_MANUAL = 0,
  VEH_CHASSIS_CTL_STATUS_REQ_ROBOTIC_CTL,
  VEH_CHASSIS_CTL_STATUS_WAITING_ROBOTIC_CTL_ACK,
  VEH_CHASSIS_CTL_STATUS_ROBOTIC,
  VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL,
  VEH_CHASSIS_CTL_STATUS_WAITING_MANUAL_CTL_ACK
};


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
  /// 处于人工驾驶模式
  VEH_DRIVING_MODE_MANUAL,
  /// 处于自动驾驶模式
  VEH_DRIVING_MODE_ROBOTIC
};

/**
 * @enum
 * @brief 方向盘控制状态
 */
enum {
  /// 无效的状态
  VEH_EPS_STATUS_INVALID = 0,
  /// 处于人工驾驶模式
  VEH_EPS_STATUS_MANUAL,
  /// 处于自动驾驶模式
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
  /// 处于人工驾驶模式
  VEH_THROTTLE_SYS_STATUS_MANUAL,
  /// 处于自动驾驶模式
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
  /// 处于人工驾驶模式
  VEH_EBS_STATUS_MANUAL,
  /// 处于自动驾驶模式
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
  VEH_TRAILER_STATUS_CONNECTED
};


/**
 * @struct Chassis
 * @brief 车身信息
 */
typedef struct _Chassis_t Chassis_t;
struct _Chassis_t {
  /// 消息头
  MsgHead_t msg_head;
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
  /// 选择档位 Number
  Int8_t selected_gear_number;
  
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

  Float32_t trailer_l;
  Float32_t trailer_w;
  Float32_t trailer_h;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearChassis(Chassis_t* const data);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
/**
 * @struct DFCV_SpecialChassisInfo_t
 * @brief DFCV控制新增信号
 */
typedef struct _DFCV_SpecialChassisInfo_t DFCV_SpecialChassisInfo_t;
struct _DFCV_SpecialChassisInfo_t {
  //DFCV新增底盘信号字段
  //制动设备源地址
  Uint8_t  source_address_brake_control_device;
  //整车总质量（含挂车）
  Float64_t  vehicle_mass;
  //左前轮制动压力
  Float64_t brake_pressure_lf;
  //当前档位传动比
  Float64_t current_gear_ratio;
  //变速箱选择的档位
  Int16_t transmission_selected_gear;
  //离合器状态
  Uint8_t clutch_switch;
  //发动机名义摩擦扭矩百分比
  Int16_t nominal_fricton_troque_percent;
  //附件损失扭矩百分比
  Int16_t estimated_lossed_torque_percent;
  //f发动机实际扭矩百分比
  Int16_t actual_engine_torque_percent;
  //驾驶员请求的扭矩百分比
  Int16_t driver_damand_torque_percent;
  //发动机设备控制源地址.
  Uint8_t source_address_engine_control_device;
  //变速箱换档状态
  Uint8_t transmission_shift_status;
  //变速箱结合状态
  Uint8_t transmission_engage_status;
  //变速箱TSC1扭矩控制模式
  Uint8_t tcu_engine_control_mode;
  //挂车状态
  Uint8_t trailer_connected_status;

  Uint8_t trans_ready_for_brake_release;
  
  Float32_t tcu_output_shaft_speed;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearDFCVSpecialChassisInfo(DFCV_SpecialChassisInfo_t* const data);

#endif

/**
 * @brief debug information of Control
 */

/**
 * @struct ChassisCtlCmd
 * @brief 向车身发送的控制指令
 */
typedef struct _ChassisCtlCmd_t ChassisCtlCmd_t;
struct _ChassisCtlCmd_t {
  /// 消息头
  MsgHead_t msg_head;
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

  /// 坡度信号
  Float32_t ramp_slope_value;
  /// 制动器控制模式
  Int8_t ebs_mode_enable;
  /// EBI控制模式
  Int8_t ebi_control_mode;
  /// EBS ungercy
  double ebs_ungercy;

  Int8_t tar_type;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearChassisCtlCmd(ChassisCtlCmd_t* const data);


/**
 * @struct ChassisFtAuman_t
 * @brief 具体车型的特殊信息 (FT-Auman)
 */
typedef struct _ChassisFtAuman_t ChassisFtAuman_t;
struct _ChassisFtAuman_t {
  /// TJA功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_tja;
  /// HWA功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_hwa;
  /// I-Drive功能开关, 0 ~ OFF, 1 ~ ON
  Int8_t switch_i_drive;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearChassisFtAuman(ChassisFtAuman_t* const data);

/**
 * @struct ChassisDfD17_t
 * @brief 具体车型的特殊信息 (DF-D17)
 */
typedef struct _ChassisDfD17_t ChassisDfD17_t;
struct _ChassisDfD17_t {
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
  /// 降级使能, 系统决策
  Int8_t enable_fallback;
  /// 目标降级是否有效
  Int8_t target_fallback_valid;
  /// 降级状态 0：No fallbcak / 1: A / 2: B / 4: C / 8: D
  Int8_t AD_fallback_level;  
    /// ECUControlInformation2_EHPS
  Uint16_t AHPSFailureCode;
  Uint8_t SteeringErrorEvent1ActivePS;
  Uint8_t SteeringErrorEvent1PS;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearChassisDFD17(ChassisDfD17_t* const data);


typedef struct _ChassisDfD17_adrecord ChassisDfD17_adrecord;

struct _ChassisDfD17_adrecord
{
  Uint32_t ad_mode;

  Uint32_t tar_vel_set;
  
  Uint32_t tar_trajectory_type;
  
  Uint32_t tar_vel_type;

  double vel_obj_dist;
  
  double vel_obj_vel;

  float planning_send_vel;

  float planning_send_ax;
};

void Phoenix_AdMsg_ClearChassisDfD17adrecord(ChassisDfD17_adrecord* const data);

/**
 * @struct SpecialChassisInfo
 * @brief 从车身获取的其它信息
 */
typedef struct _SpecialChassisInfo_t SpecialChassisInfo_t;
struct _SpecialChassisInfo_t {
  /// 消息头
  MsgHead_t msg_head;
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

  /// 目标车速有效位
  Int8_t target_velocity_valid;
  /// 目标车速[m/s]
  Float32_t target_velocity;
  /// 目标加速度有效位
  Int8_t target_acc_valid;
  /// 目标加速度[m/s^2]
  Float32_t target_acc;
  /// 目标时距有效位
  Int8_t target_time_gap_valid;
  /// 目标时距[s]
  Float32_t target_time_gap;

  /// 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  Int8_t changing_lane_req;


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
  ChassisFtAuman_t ft_auman;
  /// 具体车型的特殊信息 (DF-D17)
  ChassisDfD17_t df_d17;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearSpecialChassisInfo(SpecialChassisInfo_t* const data);


/**
 * @brief 判断是否处于自动控制模式
 * @param[in] status 当前底盘的控制状态
 * @return 0 ~ 不处于自动控制模式, 1 ~ 处于自动控制模式
 */
Int32_t Phoenix_AdMsg_IsChassisInRoboticMode(Int32_t status);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_AD_MSG_MSG_CHASSIS_C_H_


