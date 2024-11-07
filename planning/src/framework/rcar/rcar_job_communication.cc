/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_communication.cc
 * @brief      跟外部模块通信
 * @details    接收外部模块发送的消息，向外部模块发送消息
 *
 * @author     pengc
 * @date       2021.07.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "rcar/rcar_job_communication.h"

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "math/math_utils.h"
#include "common/module_status.h"
#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif
#include "data_serialization.h"
#include "communication/hmi_msg_sender.h"
#include "communication/shared_data.h"
#include "work/work_monitor.h"
#include "sensor_calibration.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#ifdef __cplusplus
extern "C" {
#endif
#include "kotei_vehicle_platform_data_api.h"
#ifdef __cplusplus
}
#endif
#endif


/******************************************************************************/
/* 类型定义                                                                    */
/******************************************************************************/
// 传感器节点类型
#define SENSING_NODE_UNDEFINED (0)
#define SENSING_NODE_RCAR (1)
#define SENSING_NODE_PC (2)
#define SENSING_NODE SENSING_NODE_PC

// 控制节点类型
#define CONTROL_NODE_UNDEFINED (0)
#define CONTROL_NODE_TC397 (1)
#define CONTROL_NODE_PC (2)
#define CONTROL_NODE CONTROL_NODE_TC397


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 指示初始化是否完成
static bool s_initialize_flag = false;
phoenix::common::Matrix<Float32_t, 3, 3> mat_calibration_radar;
phoenix::common::Matrix<Float32_t, 3, 3> mat_calibration_camera;

#if (ENABLE_UDP_NODE)
// 用来进行UDP通讯
static phoenix::framework::UdpNode s_udp_node;
#endif

// 模块状态列表
static phoenix::ad_msg::ModuleStatusList s_module_status_list;
// 事件通知列表
static phoenix::ad_msg::EventReportingList s_event_reporting_list;
// GNSS数据
static phoenix::ad_msg::Gnss s_gnss_info;
// IMU数据
static phoenix::ad_msg::Imu s_imu_info;
// 车身数据
static phoenix::ad_msg::Chassis s_chassis_info;
// 车身控制指令数据
static phoenix::ad_msg::ChassisCtlCmd s_chassis_ctl_info;
// 车辆平台相关的车身数据
static phoenix::ad_msg::SpecialChassisInfo s_special_chassis_info;
// 相机识别的车道线数据
static phoenix::ad_msg::LaneMarkCameraList s_lane_mark_camera_list;
// 相机识别的障碍物信息
static phoenix::ad_msg::ObstacleCameraList s_obstacle_camera_list;
// ESR识别的障碍物信息
static phoenix::ad_msg::ObstacleRadarList s_obstacle_esr_list;
// 交通标志及交通信号数据
static phoenix::ad_msg::TrafficSignalList s_traffic_segnal_list;
// 相机识别的车道线数据
static phoenix::ad_msg::LaneMarkCameraList s_lane_mark_camera_list_to_tc397;
// camera lane line information
static phoenix::ad_msg::LaneInfoCameraList s_lane_info_camera_list_to_tc397;
// lateral control information
static phoenix::ad_msg::LateralControlInfo s_lateral_control_info;
// control module status
static phoenix::ad_msg::ModuleStatusList s_control_module_status_list;

// 规划结果
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
static Vpf_PlanningResult s_vpf_planning_result;
#endif

// 用来发送HMI显示类消息
static phoenix::framework::HmiMsgSender s_hmi_msg_sender(Nullptr_t);
// 用来保存HMI设置消息
static phoenix::ad_msg::PlanningSettings s_msg_planning_settings;

// 模块监控
static phoenix::framework::WorkMonitor* s_module_monitor = Nullptr_t;


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/

/*
 * @brief 清除内部数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void ClearMsgData() {
  s_gnss_info.Clear();
  s_imu_info.Clear();
  s_chassis_info.Clear();
  s_chassis_ctl_info.Clear();
  s_special_chassis_info.Clear();
  s_lane_mark_camera_list.Clear();
  s_obstacle_camera_list.Clear();
  s_obstacle_esr_list.Clear();
  s_traffic_segnal_list.Clear();

  s_msg_planning_settings.Clear();
  
  // 规划结果
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  phoenix::common::com_memset(&s_vpf_planning_result,
                              0, sizeof(s_vpf_planning_result));
#endif

  s_lateral_control_info.Clear();
}

// 从中间层接收消息的回调函数
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
/*
 * @brief 接收GNSS数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvGnss(Vpf_Gnss* data) {
  // get message
  s_gnss_info.latitude = data->latitude;
  s_gnss_info.longitude = data->longitude;
  s_gnss_info.altitude = data->altitude;
  s_gnss_info.heading_gnss = phoenix::common::com_deg2rad(
        phoenix::common::ConvGpsHeading(data->heading_gnss));
  s_gnss_info.x_utm = data->x_utm;
  s_gnss_info.y_utm = data->y_utm;
  s_gnss_info.z_utm = data->z_utm;
  /// TODO: UTM定位的航向角超出了-pi到+pi的范围，后期需要修改
  s_gnss_info.heading_utm =
      phoenix::common::NormalizeAngle(data->heading_utm);
  s_gnss_info.x_odom = data->x_odom;
  s_gnss_info.y_odom = data->y_odom;
  s_gnss_info.z_odom = data->z_odom;
  s_gnss_info.heading_odom =
      phoenix::common::NormalizeAngle(data->heading_odom);
  s_gnss_info.pitch = data->pitch;
  s_gnss_info.roll = data->roll;
  s_gnss_info.v_e = data->v_e;
  s_gnss_info.v_n = data->v_n;
  s_gnss_info.v_u = data->v_u;
  s_gnss_info.v_x_utm = data->v_x_utm;
  s_gnss_info.v_y_utm = data->v_y_utm;
  s_gnss_info.v_z_utm = data->v_z_utm;
  s_gnss_info.v_x_odom = data->v_x_odom;
  s_gnss_info.v_y_odom = data->v_y_odom;
  s_gnss_info.v_z_odom = data->v_z_odom;

  s_gnss_info.gnss_status = data->gnss_status;
  s_gnss_info.utm_status = data->utm_status;
  s_gnss_info.odom_status = data->odom_status;

  if (com_isnan(s_gnss_info.latitude) || com_isinf(s_gnss_info.latitude) ||
      com_isnan(s_gnss_info.longitude) || com_isinf(s_gnss_info.longitude) ||
      com_isnan(s_gnss_info.heading_gnss) || com_isinf(s_gnss_info.heading_gnss)) {
    s_gnss_info.gnss_status = phoenix::ad_msg::Gnss::STATUS_INVALID;

    s_gnss_info.latitude = 0;
    s_gnss_info.longitude = 0;
    s_gnss_info.heading_gnss = 0;
  }

  if (com_isnan(s_gnss_info.x_utm) || com_isinf(s_gnss_info.x_utm) ||
      com_isnan(s_gnss_info.y_utm) || com_isinf(s_gnss_info.y_utm) ||
      com_isnan(s_gnss_info.heading_utm) || com_isinf(s_gnss_info.heading_utm)) {
    s_gnss_info.utm_status = phoenix::ad_msg::Gnss::STATUS_INVALID;

    s_gnss_info.x_utm = 0;
    s_gnss_info.y_utm = 0;
    s_gnss_info.heading_utm = 0;
  }

  // Update message head
  s_gnss_info.msg_head.valid = true;
  s_gnss_info.msg_head.UpdateSequenceNum();
  s_gnss_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetGnss(s_gnss_info);

  // monitor
  s_module_monitor->FeedDog_RecvGnss();
  
  return (0);
}

/*
 * @brief 接收IMU数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvImu(Vpf_imu* data) {
  // get message
  s_imu_info.yaw_rate = data->yaw_rate;
  s_imu_info.pitch_rate = data->pitch_rate;
  s_imu_info.roll_rate = data->roll_rate;
  s_imu_info.accel_x = data->accel_x;
  s_imu_info.accel_y = data->accel_y;
  s_imu_info.accel_z = data->accel_z;

  // Update message head
  s_imu_info.msg_head.valid = true;
  s_imu_info.msg_head.UpdateSequenceNum();
  s_imu_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetImu(s_imu_info);

#if (CONTROL_NODE == CONTROL_NODE_PC)
 #if (ENABLE_UDP_NODE)
  if (s_initialize_flag) {
    Uint8_t serialization_data_buff[512];
    Int32_t max_buff_size = sizeof(serialization_data_buff) - 1;

    // send planning result
    Int32_t data_size = sizeof(phoenix::ad_msg::Imu);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      Int32_t data_len = phoenix::data_serial::EncodeImuArray(
            serialization_data_buff, 0, max_buff_size, &s_imu_info, 1);
      s_udp_node.Publish("localization/imu",
                         serialization_data_buff, data_len);
    }
  }
 #endif // #if (ENABLE_UDP_NODE)
#endif // #if (CONTROL_NODE == CONTROL_NODE_PC)

  // monitor
  s_module_monitor->FeedDog_RecvImu();
  
  return (0);
}

/*
 * @brief 接收车身数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvChassis(Vpf_phoenix_ad_msg_Chassis* data) {
  // get message
  // 驾驶模式
  s_chassis_info.driving_mode = data->driving_mode;
  // 紧急停止信号
  s_chassis_info.e_stop = data->e_stop;

  // 方向盘控制状态
  s_chassis_info.eps_status = data->eps_status;
  // 油门系统控制状态
  s_chassis_info.throttle_sys_status = data->throttle_sys_status;
  // 制动系统控制状态
  s_chassis_info.ebs_status = data->ebs_status;

  // 方向盘角度有效位
  s_chassis_info.steering_wheel_angle_valid = data->steering_wheel_angle_valid;
  // 方向盘角度 (rad)
  s_chassis_info.steering_wheel_angle = data->steering_wheel_angle;
  // 方向盘转速有效位
  s_chassis_info.steering_wheel_speed_valid = data->steering_wheel_speed_valid;
  // 方向盘转速 (rad/s)
  s_chassis_info.steering_wheel_speed = data->steering_wheel_speed;
  // 实际转向扭矩有效位
  s_chassis_info.steering_wheel_torque_valid = data->steering_wheel_torque_valid;
  // 实际转向扭矩(N.m)
  s_chassis_info.steering_wheel_torque = data->steering_wheel_torque;

  // 车速有效位
  s_chassis_info.v_valid = data->v_valid;
  // 车速(m/s)
  s_chassis_info.v = data->v;
  // 加速度有效位
  s_chassis_info.a_valid = data->a_valid;
  // 加速度
  s_chassis_info.a = data->a;
  // Yaw Rate有效位
  s_chassis_info.yaw_rate_valid = data->yaw_rate_valid;
  // Yaw Rate(rad/s)
  s_chassis_info.yaw_rate = data->yaw_rate;
  // AX 有效位
  s_chassis_info.ax_valid = data->ax_valid;
  // AX(m/s^2)
  s_chassis_info.ax = data->ax;
  // AY 有效位
  s_chassis_info.ay_valid = data->ay_valid;
  // AY(m/s^2)
  s_chassis_info.ay = data->ay;

  // 左前轮速有效位
  s_chassis_info.wheel_speed_fl_valid = data->wheel_speed_fl_valid;
  // 左前轮速(m/s)
  s_chassis_info.wheel_speed_fl = data->wheel_speed_fl;
  // 右前轮速有效位
  s_chassis_info.wheel_speed_fr_valid = data->wheel_speed_fr_valid;
  // 右前轮速(m/s)
  s_chassis_info.wheel_speed_fr = data->wheel_speed_fr;
  // 左后轮速有效位
  s_chassis_info.wheel_speed_rl_valid = data->wheel_speed_rl_valid;
  // 左后轮速(m/s)
  s_chassis_info.wheel_speed_rl = data->wheel_speed_rl;
  // 右后轮速有效位
  s_chassis_info.wheel_speed_rr_valid = data->wheel_speed_rr_valid;
  // 右后轮速(m/s)
  s_chassis_info.wheel_speed_rr = data->wheel_speed_rr;
  // 左后2#轮速有效位
  s_chassis_info.wheel_speed_rl2_valid = data->wheel_speed_rl2_valid;
  // 左后2#轮速(m/s)
  s_chassis_info.wheel_speed_rl2 = data->wheel_speed_rl2;
  // 右后2#轮速有效位
  s_chassis_info.wheel_speed_rr2_valid = data->wheel_speed_rr2_valid;
  // 右后2#轮速(m/s)
  s_chassis_info.wheel_speed_rr2 = data->wheel_speed_rr2;

  // 驻车系统状态
  s_chassis_info.epb_status = data->epb_status;
  // 档位
  s_chassis_info.gear = data->gear;
  // 档位 Number
  s_chassis_info.gear_number = data->gear_number;
  // 转向拨杆信号
  s_chassis_info.signal_turning_indicator = data->signal_turning_indicator;
  // 转向灯信号
  s_chassis_info.signal_turn_lamp = data->signal_turn_lamp;
  // 制动灯信号
  s_chassis_info.signal_brake_lamp = data->signal_brake_lamp;

  // 制动踏板深度, PH=[百分比][0,100]
  s_chassis_info.brake_pedal_value = data->brake_pedal_value;
  // 油门踏板深度, PH=[百分比][0,100]
  s_chassis_info.acc_pedal_value = data->acc_pedal_value;

  // 发动机转速有效位
  s_chassis_info.engine_speed_valid = data->engine_speed_valid;
  // 发动机转速
  s_chassis_info.engine_speed = data->engine_speed;
  // 发动机转矩有效位
  s_chassis_info.engine_torque_valid = data->engine_torque_valid;
  // 发动机转矩(N.m)
  s_chassis_info.engine_torque = data->engine_torque;

  // Update message head
  s_chassis_info.msg_head.valid = true;
  s_chassis_info.msg_head.UpdateSequenceNum();
  s_chassis_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetChassis(s_chassis_info);

  // monitor
  s_module_monitor->FeedDog_RecvChassis();
  
  return (0);
}

/*
 * @brief 接收车身控制指令数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvChassisCtlCmd(Vpf_phoenix_ad_msg_ChassisCtlCmd_t* data) {
  // get message
  // 开始自动驾驶命令
  s_chassis_ctl_info.start_robotic_ctl = data->start_robotic_ctl;
  // 使能转向控制系统
  s_chassis_ctl_info.enable_eps = data->enable_eps;
  // 使能油门控制系统
  s_chassis_ctl_info.enable_throttle_sys = data->enable_throttle_sys;
  // 使能制动控制系统
  s_chassis_ctl_info.enable_ebs = data->enable_ebs;
  /// TODO: 使能远程控制
  s_chassis_ctl_info.enable_remote_ctl = 0;
  // 使能直接控制模式
  s_chassis_ctl_info.enable_direct_ctl = data->enable_debug_mode;
  // 方向盘角度 (rad)
  s_chassis_ctl_info.steering_wheel_angle = data->steering_wheel_angle;
  // 方向盘转速 (rad/s)
  s_chassis_ctl_info.steering_wheel_speed = data->steering_wheel_speed;
  // 方向盘扭矩指令(N/m)
  s_chassis_ctl_info.steering_wheel_torque = data->steering_wheel_torque;
  // 车速(m/s)
  s_chassis_ctl_info.velocity = data->velocity;
  // 加速度
  s_chassis_ctl_info.acceleration = data->acceleration;
  // 加速度量
  s_chassis_ctl_info.acc_value = data->acc_value;
  // 刹车量
  s_chassis_ctl_info.brake_value = data->brake_value;

  // 档位
  s_chassis_ctl_info.gear = data->gear;
  // 转向灯信号
  s_chassis_ctl_info.turn_lamp = data->turn_lamp;
  // 制动灯信号
  s_chassis_ctl_info.brake_lamp = data->brake_lamp;
  // 驻车系统状态
  s_chassis_ctl_info.epb_status = data->epb_status;
  // 雨刮器状态
  s_chassis_ctl_info.wiper = data->wiper;
  // 驻车系统状态
  s_chassis_ctl_info.epb_status = data->epb_status;

  // Update message head
  s_chassis_ctl_info.msg_head.valid = true;
  s_chassis_ctl_info.msg_head.UpdateSequenceNum();
  s_chassis_ctl_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetChassisCtlCmd(
        s_chassis_ctl_info);

  // monitor
  s_module_monitor->FeedDog_RecvChassisCtlCmd();
  
  return (0);
}

/*
 * @brief 接收车辆平台相关的车身数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvSpecialChassisInfo(
    Vpf_phoenix_ad_msg_SpecialChassisInfo* data) {
  // get message
  s_special_chassis_info.start_adas = data->start_adas;

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  s_special_chassis_info.cnt_stu_frame_loss_can0 =
      data->cnt_stu_timeout_can1;
  s_special_chassis_info.cnt_stu_frame_loss_can1 =
      data->cnt_stu_timeout_can2;
  s_special_chassis_info.cnt_stu_frame_loss_can2 =
      data->cnt_stu_timeout_can3;
  s_special_chassis_info.cnt_stu_frame_loss_can3 = 0;

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  s_special_chassis_info.cnt_stu_gtw_to_veh_can0 =
      !data->cnt_stu_gtw_to_veh_can1;
  s_special_chassis_info.cnt_stu_gtw_to_veh_can1 =
      !data->cnt_stu_gtw_to_veh_can2;
  s_special_chassis_info.cnt_stu_gtw_to_veh_can2 =
      !data->cnt_stu_gtw_to_veh_can3;
  s_special_chassis_info.cnt_stu_gtw_to_veh_can3 = 0;

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  s_special_chassis_info.cnt_stu_ctl_to_gtw_can0 =
      !data->cnt_stu_ctl_to_gtw_can1;
  s_special_chassis_info.cnt_stu_ctl_to_gtw_can1 =
      !data->cnt_stu_ctl_to_gtw_can2;
  s_special_chassis_info.cnt_stu_ctl_to_gtw_can2 =
      !data->cnt_stu_ctl_to_gtw_can3;
  s_special_chassis_info.cnt_stu_ctl_to_gtw_can3 = 0;

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  s_special_chassis_info.cnt_stu_ctl_to_gtw =
      !data->cnt_stu_ctl_gtw;

  // Update message head
  s_special_chassis_info.msg_head.valid = true;
  s_special_chassis_info.msg_head.UpdateSequenceNum();
  s_special_chassis_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetSpecialChassisInfo(
        s_special_chassis_info);

  // Monitor
  s_module_monitor->FeedDog_RecvSpecialChassisInfo();
		
  return (0);
}

/*
 * @brief 接收相机识别的车道线数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvLaneMarkCameraList(Vpf_LaneMarkCameraList* data) {
  // get message
  s_lane_mark_camera_list.lane_mark_num = data->lane_mark_num;
  if (s_lane_mark_camera_list.lane_mark_num < 0) {
    s_lane_mark_camera_list.lane_mark_num = 0;
  }
  if (s_lane_mark_camera_list.lane_mark_num >
      phoenix::ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM) {
    s_lane_mark_camera_list.lane_mark_num =
        phoenix::ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM;
  }

  for (Int32_t i = 0; i < s_lane_mark_camera_list.lane_mark_num; ++i) {
    const Vpf_LaneMarkCamera& lane_mark = data->lane_marks[i];
    phoenix::ad_msg::LaneMarkCamera& dst =
        s_lane_mark_camera_list.lane_marks[i];

    dst.id = lane_mark.id;
    switch (lane_mark.lane_mark_type) {
    case (Vpf_LANE_MARK_TYPE_UNKNOWN):
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
      break;
    case (Vpf_LANE_MARK_TYPE_DASHED):
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
      break;
    case (Vpf_LANE_MARK_TYPE_SOLID):
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
      break;
    case (Vpf_LANE_MARK_TYPE_DOUBLE_LANE_MARK):
      dst.lane_mark_type =
          phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
      break;
    case (Vpf_LANE_MARK_TYPE_BOTTS_DOTS):
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
      break;
    case (Vpf_LANE_MARK_TYPE_ROAD_EDGE):
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
      break;
    default:
      dst.lane_mark_type =
            phoenix::ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
      break;
    }
    dst.quality = lane_mark.quality;
    dst.view_range_valid = lane_mark.view_range_valid;
    dst.mark_width = lane_mark.mark_width;
    dst.view_range_start = lane_mark.view_range_start;
    dst.view_range_end = lane_mark.view_range_end;
    dst.c0 = lane_mark.c0;
    dst.c1 = lane_mark.c1;
    dst.c2 = lane_mark.c2;
    dst.c3 = lane_mark.c3;
  }

  // Update message head
  s_lane_mark_camera_list.msg_head.valid = true;
  s_lane_mark_camera_list.msg_head.UpdateSequenceNum();
  s_lane_mark_camera_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetLaneMarkCameraList(
        s_lane_mark_camera_list);

  // monitor
  s_module_monitor->FeedDog_RecvLaneMarkCameraList();
  
  return (0);
}

/*
 * @brief 接收相机识别的障碍物信息的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvObstacleCameraList(Vpf_ObstacleCameraList* data) {
  // get message
  // Update each of lane mark information
  Int32_t obstacle_num = data->obstacle_num;
  phoenix::common::Matrix<Float32_t, 2, 1> point_conv;
  phoenix::ad_msg::Chassis chassis;

  phoenix::framework::SharedData::instance()->GetChassis(&chassis);
  if (obstacle_num < 0) {
    obstacle_num = 0;
  }
  if (obstacle_num > phoenix::ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM) {
    obstacle_num = phoenix::ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const Vpf_ObstacleCamera& obj = data->obstacles[i];
    phoenix::ad_msg::ObstacleCamera& dst = s_obstacle_camera_list.obstacles[i];

    dst.id = obj.id;

    switch (obj.type) {
    case (Vpf_OBJ_TYPE_UNKNOWN):
      dst.type = phoenix::ad_msg::OBJ_TYPE_UNKNOWN;
      dst.length = 1.0F;
      break;
    case (Vpf_OBJ_TYPE_PASSENGER_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      dst.length = 5.0F;
      break;
    case (Vpf_OBJ_TYPE_COMMERCIAL_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      dst.length = 5.0F;
      break;
    case (Vpf_OBJ_TYPE_SPECIAL_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      dst.length = 5.0F;
      break;
    case (Vpf_OBJ_TYPE_OTHER_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      dst.length = 5.0F;
      break;
    case (Vpf_OBJ_TYPE_PEDESTRIAN):
      dst.type = phoenix::ad_msg::OBJ_TYPE_PEDESTRIAN;
      dst.length = 1.0F;
      break;
    case (Vpf_OBJ_TYPE_BICYCLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_BICYCLE;
      dst.length = 2.0F;
      break;
    case (Vpf_OBJ_TYPE_ANIMAL):
      dst.type = phoenix::ad_msg::OBJ_TYPE_ANIMAL;
      dst.length = 1.0F;
      break;
    case (Vpf_OBJ_TYPE_DISCARD):
      dst.type = phoenix::ad_msg::OBJ_TYPE_DISCARD;
      dst.length = 1.0F;
      break;
    case (Vpf_OBJ_TYPE_CURB):
      dst.type = phoenix::ad_msg::OBJ_TYPE_CURB;
      dst.length = 1.0F;
      break;
    default:
      dst.type = phoenix::ad_msg::OBJ_TYPE_UNKNOWN;
      dst.length = 1.0F;
      break;
    }

    switch (obj.status) {
    case (Vpf_OBJ_STATUS_UNKNOWN):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    case (Vpf_OBJ_STATUS_STANDING):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_STANDING;
      break;
    case (Vpf_OBJ_STATUS_STOPPED):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
      break;
    case (Vpf_OBJ_STATUS_MOVING):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_MOVING;
      break;
    case (Vpf_OBJ_STATUS_ONCOMING):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_ONCOMING;
      break;
    case (Vpf_OBJ_STATUS_PARKED):
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_PARKED;
      break;
    default:
      dst.status = phoenix::ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    }

    switch (obj.cut_in) {
    case (Vpf_CUT_IN_TYPE_UNKNOWN):
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    case (Vpf_CUT_IN_TYPE_IN_HOST_LANE):
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE;
      break;
    case (Vpf_CUT_IN_TYPE_OUT_HOST_LANE):
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_OUT_HOST_LANE;
      break;
    case (Vpf_CUT_IN_TYPE_CUT_IN):
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_IN;
      break;
    case (Vpf_CUT_IN_TYPE_CUT_OUT):
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_OUT;
      break;
    default:
      dst.cut_in = phoenix::ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    }

    switch (obj.blinker) {
    case (Vpf_BLINKER_UNKNOWN):
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    case (Vpf_BLINKER_OFF):
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_OFF;
      break;
    case (Vpf_BLINKER_LEFT):
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_LEFT;
      break;
    case (Vpf_BLINKER_RIGHT):
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_RIGHT;
      break;
    case (Vpf_BLINKER_BOTH):
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_BOTH;
      break;
    default:
      dst.blinker = phoenix::ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    }

    dst.brake_lights = obj.brake_lights;
    dst.age = obj.age;
    dst.lane = obj.lane;
    // dst.length = obj.length;
    dst.width = obj.width;
    point_conv(0) = obj.x;
    point_conv(1) = obj.y;
    phoenix::common::TransformVert_2D(mat_calibration_camera, &point_conv);
    dst.x = point_conv(0);
    dst.y = point_conv(1);
    dst.v_x = obj.rel_v_x + chassis.v;
    dst.accel_x = obj.accel_x;
    dst.yaw_rate = obj.yaw_rate;
    dst.scale_change = obj.scale_change;
  }
  // Update number of obstacles
  s_obstacle_camera_list.obstacle_num = obstacle_num;

  // Update message head
  s_obstacle_camera_list.msg_head.valid = true;
  s_obstacle_camera_list.msg_head.UpdateSequenceNum();
  s_obstacle_camera_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetObstacleCameraList(
        s_obstacle_camera_list);

  // monitor
  s_module_monitor->FeedDog_RecvObstacleCameraList();
  
  return (0);
}

/*
 * @brief 接收ESR识别的障碍物信息的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvObstacleEsrList(Vpf_Obstacle_FWRadarList* data) {
  // get message
  // Update each of lane mark information
  Int32_t obstacle_num = data->obstacle_num;
  phoenix::ad_msg::Chassis chassis;
  phoenix::common::Matrix<Float32_t, 2, 1> point_conv;

  phoenix::framework::SharedData::instance()->GetChassis(&chassis);
  if (obstacle_num < 0) {
    obstacle_num = 0;
  }
  if (obstacle_num > phoenix::ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM) {
    obstacle_num = phoenix::ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const Vpf_Obstacle_FWRadar& obj = data->obstacles[i];
    phoenix::ad_msg::ObstacleRadar& dst = s_obstacle_esr_list.obstacles[i];

    dst.id = obj.id;

    switch (obj.type) {
    case (Vpf_OBJ_TYPE_UNKNOWN):
      dst.type = phoenix::ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    case (Vpf_OBJ_TYPE_PASSENGER_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case (Vpf_OBJ_TYPE_COMMERCIAL_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case (Vpf_OBJ_TYPE_SPECIAL_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case (Vpf_OBJ_TYPE_OTHER_VEHICLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      break;
    case (Vpf_OBJ_TYPE_PEDESTRIAN):
      dst.type = phoenix::ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case (Vpf_OBJ_TYPE_BICYCLE):
      dst.type = phoenix::ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case (Vpf_OBJ_TYPE_ANIMAL):
      dst.type = phoenix::ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case (Vpf_OBJ_TYPE_DISCARD):
      dst.type = phoenix::ad_msg::OBJ_TYPE_DISCARD;
      break;
    case (Vpf_OBJ_TYPE_CURB):
      dst.type = phoenix::ad_msg::OBJ_TYPE_CURB;
      break;
    default:
      dst.type = phoenix::ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    }

    switch (obj.track_status) {
    case (Vpf_TRACK_STATUS_NO_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    case (Vpf_TRACK_STATUS_NEW_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
      break;
    case (Vpf_TRACK_STATUS_NEW_UPDATED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET;
      break;
    case (Vpf_TRACK_STATUS_UPDATED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET;
      break;
    case (Vpf_TRACK_STATUS_COASTED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_COASTED_TARGET;
      break;
    case (Vpf_TRACK_STATUS_MERGED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_MERGED_TARGET;
      break;
    case (Vpf_TRACK_STATUS_INVALID_COASTED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET;
      break;
    case (Vpf_TRACK_STATUS_NEW_COASTED_TARGET):
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET;
      break;
    default:
      dst.track_status =
          phoenix::ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    }

    switch (obj.merged_status) {
    case (Vpf_MERGED_STATUS_NO_TARGET):
      dst.merged_status =
          phoenix::ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    case (Vpf_MERGED_STATUS_MR_TARGET):
      dst.merged_status =
          phoenix::ad_msg::ObstacleRadar::MERGED_STATUS_MR_TARGET;
      break;
    case (Vpf_MERGED_STATUS_LR_TARGET):
      dst.merged_status =
          phoenix::ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
      break;
    case (Vpf_MERGED_STATUS_MR_LR_TARGET):
      dst.merged_status =
          phoenix::ad_msg::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET;
      break;
    default:
      dst.merged_status =
          phoenix::ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    }

    dst.oncomming = obj.oncomming;
    dst.bridge = obj.bridge;
    dst.range = obj.range;
    dst.angle = obj.angle;
    dst.range_rate = obj.range_rate;
    dst.range_acceleration = obj.range_acceleration;
    dst.lateral_rate = obj.lateral_rate;
    dst.width = obj.width;
    if (dst.width < 0.1F) {
      dst.width = 0.1F;
    }
    dst.length = obj.length;

    point_conv(0) = obj.x;
    point_conv(1) = obj.y;
    phoenix::common::TransformVert_2D(mat_calibration_radar, &point_conv);
    dst.x = point_conv(0);
    dst.y = point_conv(1);
    dst.v_x = obj.rel_v_x + chassis.v;
    dst.v_y = obj.rel_v_y;
    dst.accel_x = obj.accel_x;
    dst.accel_y = obj.accel_y;
    dst.yaw_rate = obj.yaw_rate;
  }

  // Update number of obstacles
  s_obstacle_esr_list.obstacle_num = obstacle_num;

  // Update message head
  s_obstacle_esr_list.msg_head.valid = true;
  s_obstacle_esr_list.msg_head.UpdateSequenceNum();
  s_obstacle_esr_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetObstacleRadarFrontList(
        s_obstacle_esr_list);

  // monitor
  s_module_monitor->FeedDog_RecvObstacleRadarFrontList();
  
  return (0);
}

/*
 * @brief 接收交通标志及交通信号数据的回调函数
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
//static Int32_t VPF_RecvTrafficSignal(Vpf_TrafficSignal* data) {

//  phoenix::framework::SharedData::instance()->SetTrafficSignal(s_traffic_segnal);
//}

/*
 * @brief 接收横向控制调试信息
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvLateralControlInfo(Vpf_LateralControlInfo* data) {
  // get message
  s_lateral_control_info.lat_ctl_result = data->lat_ctl_result;
  T.B.D

  // Update message head
  s_lateral_control_info.msg_head.valid = true;
  s_lateral_control_info.msg_head.UpdateSequenceNum();
  s_lateral_control_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetLateralControlInfo(
        s_lateral_control_info);

  return (0);
}

/*
 * @brief 接收横向控制调试信息
 * @param[in] data 消息存储地址
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Int32_t VPF_RecvControlModuleStatus(Vpf_ModuleStatus* data) {
  // get message
  s_control_module_status_list.module_status_num = data->module_status_num;
  if (s_control_module_status_list.module_status_num >
      phoenix::ad_msg::ModuleStatusList::MAX_MODULE_STATUS_NUM) {
    s_control_module_status_list.module_status_num =
        phoenix::ad_msg::ModuleStatusList::MAX_MODULE_STATUS_NUM;
  }
  for (Int32_t i = 0; i < s_control_module_status_list.module_status_num; ++i) {
  T.B.D
  }

  // Update message head
  s_control_module_status_list.msg_head.valid = true;
  s_control_module_status_list.msg_head.UpdateSequenceNum();
  s_control_module_status_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

  // save message
  phoenix::framework::SharedData::instance()->SetControlModuleStatusList(
        s_control_module_status_list);

  return (0);
}
#endif

/*
 * @brief 接收HMI设置消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvHmiSettings(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodePlanningSettingsArray(
          buf, 0, buf_len, &s_msg_planning_settings, 1);

    phoenix::framework::SharedData::instance()->SetRemotePlanningSettings(
          s_msg_planning_settings);
  }
}

/*
 * @brief 接收LaneMarkCameraList消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvLaneMarkCamera(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeLaneMarkCameraListArray(
          buf, 0, buf_len, &s_lane_mark_camera_list, 1);

    // Update message head
    s_lane_mark_camera_list.msg_head.valid = true;
    s_lane_mark_camera_list.msg_head.UpdateSequenceNum();
    s_lane_mark_camera_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetLaneMarkCameraList(
          s_lane_mark_camera_list);

    // monitor
    s_module_monitor->FeedDog_RecvLaneMarkCameraList();
  }
}

/*
 * @brief 接收ObstacleCameraList消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvObstacleCamera(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeObstacleCameraListArray(
          buf, 0, buf_len, &s_obstacle_camera_list, 1);

    // Update message head
    s_obstacle_camera_list.msg_head.valid = true;
    s_obstacle_camera_list.msg_head.UpdateSequenceNum();
    s_obstacle_camera_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetObstacleCameraList(
          s_obstacle_camera_list);

    // monitor
    s_module_monitor->FeedDog_RecvObstacleCameraList();
  }
}

/*
 * @brief 接收ObstacleCameraList消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvObstacleRadarFront(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &s_obstacle_esr_list, 1);

    // Update message head
    s_obstacle_esr_list.msg_head.valid = true;
    s_obstacle_esr_list.msg_head.UpdateSequenceNum();
    s_obstacle_esr_list.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetObstacleRadarFrontList(
          s_obstacle_esr_list);

    // monitor
    s_module_monitor->FeedDog_RecvObstacleRadarFrontList();
  }
}

/*
 * @brief 接收Chassis消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvChassis(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeChassisArray(
          buf, 0, buf_len, &s_chassis_info, 1);

    // Update message head
    s_chassis_info.msg_head.valid = true;
    s_chassis_info.msg_head.UpdateSequenceNum();
    s_chassis_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetChassis(s_chassis_info);

    // monitor
    s_module_monitor->FeedDog_RecvChassis();
  }
}

/*
 * @brief 接收ChassisCtlCmd消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvChassisCtlCmd(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeChassisCtlCmdArray(
          buf, 0, buf_len, &s_chassis_ctl_info, 1);

    // Update message head
    s_chassis_ctl_info.msg_head.valid = true;
    s_chassis_ctl_info.msg_head.UpdateSequenceNum();
    s_chassis_ctl_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetChassisCtlCmd(
          s_chassis_ctl_info);

    // monitor
    s_module_monitor->FeedDog_RecvChassisCtlCmd();
  }
}

/*
 * @brief 接收SpecialChassisInfo消息的回调函数
 * @param[in] channel 消息通道
 * @param[in] buf 消息存储地址
 * @param[in] buf_len 消息长度
 * @param[in] user 用户数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UDP_RecvSpecialChassisInfo(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  if ((Nullptr_t != buf) && (buf_len > 0)) {
    phoenix::data_serial::DecodeSpecialChassisInfoArray(
          buf, 0, buf_len, &s_special_chassis_info, 1);

    // Update message head
    s_special_chassis_info.msg_head.valid = true;
    s_special_chassis_info.msg_head.UpdateSequenceNum();
    s_special_chassis_info.msg_head.timestamp = phoenix::common::GetClockNowMs();

    // save message
    phoenix::framework::SharedData::instance()->SetSpecialChassisInfo(
          s_special_chassis_info);

    // Monitor
    s_module_monitor->FeedDog_RecvSpecialChassisInfo();
  }
}

/*
 * @brief 注册通讯用的回调函数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void RegisterVpfCommunicationCallbackFunc() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Vpf_SensorDataCallback callback;
  callback.RecvGnssCallback = &VPF_RecvGnss;
  callback.RecvImuCallback = &VPF_RecvImu;

#if (SENSING_NODE == SENSING_NODE_RCAR)
  callback.RecvLaneMarkCameraListCallback = &VPF_RecvLaneMarkCameraList;
  callback.RecvObstacleCameraListCallback = &VPF_RecvObstacleCameraList;
  callback.RecvObstacleEsrListCallback = &VPF_RecvObstacleEsrList;
#elif (SENSING_NODE == SENSING_NODE_PC)
  callback.RecvLaneMarkCameraListCallback = Nullptr_t;
  callback.RecvObstacleCameraListCallback = Nullptr_t;
  callback.RecvObstacleEsrListCallback = Nullptr_t;
#else
  ERROR: Please specify the sensing node.
#endif

#if (CONTROL_NODE == CONTROL_NODE_TC397)
  callback.RecvChassisCallback = &VPF_RecvChassis;
  callback.RecvChassisCtlCmdCallback = &VPF_RecvChassisCtlCmd;
  callback.RecvSpecialChassisInfoCallback = &VPF_RecvSpecialChassisInfo;
#elif (CONTROL_NODE == CONTROL_NODE_PC)
  callback.RecvChassisCallback = Nullptr_t;
  callback.RecvChassisCtlCmdCallback = Nullptr_t;
  callback.RecvSpecialChassisInfoCallback = Nullptr_t;
#else
  ERROR: Please specify the control node.
#endif

  kotei_set_sensor_data_callback(&callback);
#endif // #if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
}


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/
/*
 * @brief 初始化外部通信模块
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_Initialize(void* module_monitor) {
  bool ret = false;

  // 设置模块监控器
  s_module_monitor = (phoenix::framework::WorkMonitor*)module_monitor;

  // 初始化全局变量
  s_initialize_flag = false;

  //设置雷达传感器标定数据
  phoenix::framework::SensorOnVehicle sensor_param;
  sensor_param.SensorOnVehicleInit();
  phoenix::framework::Calibration calibration = sensor_param.calibration_radar[0];
  phoenix::common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_calibration_radar.SetIdentity();
  phoenix::common::Rotate_2D<Float32_t>(
        rotate_center, calibration.yaw_offset, &mat_calibration_radar);
  phoenix::common::Translate_2D(
        calibration.x_offset, calibration.y_offset, &mat_calibration_radar);

  //设置相机传感器标定数据
  mat_calibration_camera.SetIdentity();  
  calibration = sensor_param.calibration_cam[0];
  phoenix::common::Rotate_2D<Float32_t>(
        rotate_center, calibration.yaw_offset, &mat_calibration_camera);
  phoenix::common::Translate_2D(
        calibration.x_offset, calibration.y_offset, &mat_calibration_camera);

  // 清除内部数据
  ClearMsgData();

  // 注册通讯用的回调函数
  RegisterVpfCommunicationCallbackFunc();

#if (ENABLE_UDP_NODE)
  s_hmi_msg_sender.SetUdpNode(&s_udp_node);
#endif

  // 启动UDP节点
#if (ENABLE_UDP_NODE)
  phoenix::framework::UdpNode::UdpParam udp_param;
  udp_param.enable_recv = true;
  udp_param.enable_send = true;
  Int32_t mode = 0;
  if (0 == mode) {
    udp_param.rcv_port = 9500;
    udp_param.snd_port = 9501;
    udp_param.mode = phoenix::framework::UdpNode::UdpParam::MODE_UNICAST;
    strncpy(udp_param.snd_addr, "192.168.1.100", sizeof(udp_param.snd_addr)-1);
  } else if (1 == mode) {
    udp_param.rcv_port = 9500;
    udp_param.snd_port = 9500;
    udp_param.mode = phoenix::framework::UdpNode::UdpParam::MODE_GROUP;
    strncpy(udp_param.snd_addr, "224.10.10.2", sizeof(udp_param.snd_addr)-1);
  } else {
    udp_param.rcv_port = 9500;
    udp_param.snd_port = 9500;
    udp_param.mode = phoenix::framework::UdpNode::UdpParam::MODE_BROADCAST;
    //strncpy(udp_param.snd_addr, "192.168.10.102", sizeof(udp_param.snd_addr)-1);
    strncpy(udp_param.snd_addr, "192.168.1.255", sizeof(udp_param.snd_addr)-1);
  }

  ret = s_udp_node.Start(udp_param);
  if (false == ret) {
    LOG_ERR << "Failed to start UDP node.";
    return;
  }
#endif

#if (ENABLE_UDP_NODE)
  // 启动HMI消息发送功能
  ret = s_hmi_msg_sender.Start();
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to start hmi message sender.";
    return;
  }
#endif
  // 从UDP节点中订阅消息
#if (ENABLE_UDP_NODE)
  ret = s_udp_node.Subscribe("hmi/planning/remote_settings",
                             &UDP_RecvHmiSettings, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"hmi/planning/settings\" from UDP Node.";
    return;
  }

#if (SENSING_NODE == SENSING_NODE_PC)
  ret = s_udp_node.Subscribe("perception/lane_mark_camera",
                             &UDP_RecvLaneMarkCamera, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"perception/lane_mark_camera\" from UDP Node.";
    return;
  }

  ret = s_udp_node.Subscribe("perception/obstacle_camera",
                             &UDP_RecvObstacleCamera, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"perception/obstacle_camera\" from UDP Node.";
    return;
  }

  ret = s_udp_node.Subscribe("perception/obstacle_radar_0",
                             &UDP_RecvObstacleRadarFront, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"perception/obstacle_radar_0\" from UDP Node.";
    return;
  }
#endif // #if (SENSING_NODE == SENSING_NODE_PC)

 #if (CONTROL_NODE == CONTROL_NODE_PC)
  ret = s_udp_node.Subscribe("control/chassis",
                             &UDP_RecvChassis, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"control/chassis\" from UDP Node.";
    return;
  }

  ret = s_udp_node.Subscribe("control/chassis_ctl_cmd",
                             &UDP_RecvChassisCtlCmd, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"control/chassis_ctl_cmd\" from UDP Node.";
    return;
  }

  ret = s_udp_node.Subscribe("control/special_chassis_info",
                             &UDP_RecvSpecialChassisInfo, Nullptr_t);
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to subscribe \"control/special_chassis_info\" from UDP Node.";
    return;
  }
 #endif // #if (CONTROL_NODE == CONTROL_NODE_PC)
#endif // #if (ENABLE_UDP_NODE)

  // 启动UDP节点消息接收功能
#if (ENABLE_UDP_NODE)
  /// TODO: 这里将启动一个Task用来接收UDP消息，可能与RCAR系统存在冲突!!!
  ret = s_udp_node.StartReceiving();
  if (false == ret) {
    s_udp_node.Stop();
    LOG_ERR << "Failed to start UDP receiving.";
    return;
  }
#endif

  // 初始化成功
  s_initialize_flag = true;
}

/*
 * @brief 发送规划的结果
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_SendPlanningResult(const void* planning_ret) {
  const phoenix::ad_msg::PlanningResult& planning_result =
      *((const phoenix::ad_msg::PlanningResult*)planning_ret);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // message head
  s_vpf_planning_result.msg_head.valid = planning_result.msg_head.valid;
  s_vpf_planning_result.msg_head.sequence = planning_result.msg_head.sequence;
  s_vpf_planning_result.msg_head.timestamp = planning_result.msg_head.timestamp;
  // 当前决策模块的状态
  s_vpf_planning_result.planning_status[0] = planning_result.planning_status[0];
  s_vpf_planning_result.planning_status[1] = planning_result.planning_status[1];
  s_vpf_planning_result.planning_status[2] = planning_result.planning_status[2];
  s_vpf_planning_result.planning_status[3] = planning_result.planning_status[3];

  // 驾驶模式请求
  s_vpf_planning_result.tar_driving_mode = planning_result.tar_driving_mode;
  // 使能转向系统
  s_vpf_planning_result.enable_eps = planning_result.enable_eps;
  // 使能油门系统
  s_vpf_planning_result.enable_throttle_sys = planning_result.enable_throttle_sys;
  // 使能制动系统
  s_vpf_planning_result.enable_ebs = planning_result.enable_ebs;

  // 保持当前方向盘角度不变
  s_vpf_planning_result.hold_steering_wheel = planning_result.hold_steering_wheel;
  // 释放油门
  s_vpf_planning_result.release_throttle = planning_result.release_throttle;

  // 档位请求
  s_vpf_planning_result.tar_gear = planning_result.tar_gear;
  // 转向指示灯请求
  s_vpf_planning_result.tar_turn_lamp = planning_result.tar_turn_lamp;
  // 制动灯请求
  s_vpf_planning_result.tar_brake_lamp = planning_result.tar_brake_lamp;

  // 速度请求
  s_vpf_planning_result.tar_v = planning_result.tar_v;
  // 加速度请求
  s_vpf_planning_result.tar_a = planning_result.tar_a;

  // 目标轨迹
  s_vpf_planning_result.tar_trj.timestamp = planning_result.tar_trj.timestamp;
  s_vpf_planning_result.tar_trj.curr_pos.x = planning_result.tar_trj.curr_pos.x;
  s_vpf_planning_result.tar_trj.curr_pos.y = planning_result.tar_trj.curr_pos.y;
  s_vpf_planning_result.tar_trj.curr_pos.h = planning_result.tar_trj.curr_pos.h;
  s_vpf_planning_result.tar_trj.curr_pos.c = planning_result.tar_trj.curr_pos.c;
  s_vpf_planning_result.tar_trj.curr_pos.s = planning_result.tar_trj.curr_pos.s;
  s_vpf_planning_result.tar_trj.curr_pos.l = planning_result.tar_trj.curr_pos.l;
  s_vpf_planning_result.tar_trj.leading_pos.x = planning_result.tar_trj.leading_pos.x;
  s_vpf_planning_result.tar_trj.leading_pos.y = planning_result.tar_trj.leading_pos.y;
  s_vpf_planning_result.tar_trj.leading_pos.h = planning_result.tar_trj.leading_pos.h;
  s_vpf_planning_result.tar_trj.leading_pos.c = planning_result.tar_trj.leading_pos.c;
  s_vpf_planning_result.tar_trj.leading_pos.s = planning_result.tar_trj.leading_pos.s;
  s_vpf_planning_result.tar_trj.leading_pos.l = planning_result.tar_trj.leading_pos.l;

  s_vpf_planning_result.tar_trj.lat_err.moving_flag =
      planning_result.tar_trj.lat_err.moving_flag;
  s_vpf_planning_result.tar_trj.lat_err.samples[0].lat_err =
      planning_result.tar_trj.lat_err.samples[0].lat_err;
  s_vpf_planning_result.tar_trj.lat_err.samples[0].lat_err_v =
      planning_result.tar_trj.lat_err.samples[0].lat_err_chg_rate;
  s_vpf_planning_result.tar_trj.lat_err.samples[0].yaw_err =
      planning_result.tar_trj.lat_err.samples[0].yaw_err;
  s_vpf_planning_result.tar_trj.lat_err.samples[0].yaw_err_v =
      planning_result.tar_trj.lat_err.samples[0].yaw_err_chg_rate;
  s_vpf_planning_result.tar_trj.lat_err.samples[1].lat_err =
      planning_result.tar_trj.lat_err.samples[1].lat_err;
  s_vpf_planning_result.tar_trj.lat_err.samples[1].lat_err_v =
      planning_result.tar_trj.lat_err.samples[1].lat_err_chg_rate;
  s_vpf_planning_result.tar_trj.lat_err.samples[1].yaw_err =
      planning_result.tar_trj.lat_err.samples[1].yaw_err;
  s_vpf_planning_result.tar_trj.lat_err.samples[1].yaw_err_v =
      planning_result.tar_trj.lat_err.samples[1].yaw_err_chg_rate;


  s_vpf_planning_result.tar_trj.trj_direction =
      planning_result.tar_trj.trj_direction;

  s_vpf_planning_result.tar_trj.points_num = planning_result.tar_trj.points_num;
  if (s_vpf_planning_result.tar_trj.points_num > Vpf_MAX_TRAJECTORY_POINT_NUM) {
    s_vpf_planning_result.tar_trj.points_num = Vpf_MAX_TRAJECTORY_POINT_NUM;
  }
  for (Int32_t i = 0; i < s_vpf_planning_result.tar_trj.points_num; ++i) {
    s_vpf_planning_result.tar_trj.points[i].x = planning_result.tar_trj.points[i].x;
    s_vpf_planning_result.tar_trj.points[i].y = planning_result.tar_trj.points[i].y;
    s_vpf_planning_result.tar_trj.points[i].h = planning_result.tar_trj.points[i].h;
    s_vpf_planning_result.tar_trj.points[i].c = planning_result.tar_trj.points[i].c;
    s_vpf_planning_result.tar_trj.points[i].s = planning_result.tar_trj.points[i].s;
  }


   //轨迹曲线拟合系数
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv0 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv0;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv1 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv1;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv2 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv2;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv3 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv3;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv4 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv4;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv5 =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Cv5;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Dist2PathZero =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_Dist2PathZero;
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid =
      planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;

  for (Int32_t i = 0; i < 6; ++i) {
    s_vpf_planning_result.tar_trj.PlanningTraj_Cv.coeffs[i]= planning_result.tar_trj.PlanningTraj_Cv.coeffs[i];
  }

  //车道中心线系数
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv0 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv0;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv1 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv1;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv2 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv2;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv3 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv3;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv4 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv4;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv5 =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_Cv5;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneDist2PathZero =
      planning_result.tar_trj.LaneTraj_Cv.LaneDist2PathZero;
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.LaneTraj_IsValid =
      planning_result.tar_trj.LaneTraj_Cv.LaneTraj_IsValid;

  for (Int32_t i = 0; i < 6; ++i) {
    s_vpf_planning_result.tar_trj.LaneTraj_Cv.c_center[i]= planning_result.tar_trj.LaneTraj_Cv.c_center[i];
  }

    //left车道线系数
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv0 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv0;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv1 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv1;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv2 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv2;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv3 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv3;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv4 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv4;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv5 =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_Cv5;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneDist2PathZero =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneDist2PathZero;
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_IsValid =
      planning_result.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_IsValid;

  for (Int32_t i = 0; i < 6; ++i) {
    s_vpf_planning_result.tar_trj.LeftLaneTraj_Cv.c_left[i]= planning_result.tar_trj.LeftLaneTraj_Cv.c_left[i];
  }

      //right车道线系数
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv0 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv0;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv1 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv1;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv2 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv2;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv3 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv3;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv4 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv4;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv5 =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_Cv5;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneDist2PathZero =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneDist2PathZero;
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_IsValid =
      planning_result.tar_trj.RightLaneTraj_Cv.RightLaneTraj_IsValid;

  for (Int32_t i = 0; i < 6; ++i) {
    s_vpf_planning_result.tar_trj.RightLaneTraj_Cv.c_right[i]= planning_result.tar_trj.RightLaneTraj_Cv.c_right[i];
  }


  // send planning result
  kotei_set_planning_result(&s_vpf_planning_result);
#endif

#if (CONTROL_NODE == CONTROL_NODE_PC)
 #if (ENABLE_UDP_NODE)
  if (s_initialize_flag) {
    Uint8_t serialization_data_buff[512];
    Int32_t max_buff_size = sizeof(serialization_data_buff) - 1;

    // send planning result
    Int32_t data_size = sizeof(phoenix::ad_msg::PlanningResult);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      Int32_t data_len = phoenix::data_serial::EncodePlanningResultArray(
            serialization_data_buff, 0, max_buff_size, &planning_result, 1);
      s_udp_node.Publish("planning/planning_result",
                         serialization_data_buff, data_len);
    }
  }
 #endif // #if (ENABLE_UDP_NODE)
#endif // #if (CONTROL_NODE == CONTROL_NODE_PC)
}

/*
 * @brief 发送模块状态
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/03/20  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_SendModuleStatus() {
  phoenix::framework::SharedData *shared_data =
      phoenix::framework::SharedData::instance();

  shared_data->GetModuleStatusList(&s_module_status_list);

  /// TODO: Send to TC397
}

/*
 * @brief 发送事件通知
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/03/20  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_SendEventReporting() {
  phoenix::framework::SharedData *shared_data =
      phoenix::framework::SharedData::instance();

  shared_data->GetEventReportingList(&s_event_reporting_list);

  /// TODO: Send to TC397
}

/*
 * @brief 发送驾驶地图的状态
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/03/20  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_SendDrivingMapStatus() {
  phoenix::framework::SharedData *shared_data =
      phoenix::framework::SharedData::instance();

  // Camera lane mark
  shared_data->GetLaneMarkCameraList(&s_lane_mark_camera_list_to_tc397);
  // Camera lane
  shared_data->GetLaneInfoCameraList(&s_lane_info_camera_list_to_tc397);

  /// TODO: Send to TC397
}

/*
 * @brief 向HMI模块发送相关的消息（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Communication_SendHmiMsg() {
  if (s_initialize_flag) {
    // 发送HMI相关的消息
    s_hmi_msg_sender.SendHmiMsg();
  }
}

