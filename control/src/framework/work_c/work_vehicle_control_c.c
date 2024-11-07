/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_vehicle_control.c
 * @brief      车身控制
 * @details    封装了车身控制功能的接口
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
#include "work_c/work_vehicle_control_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "math/math_utils_c.h"
#include "communication_c/shared_data_c.h"
#include "longitudinal_control_wrapper_c.h"
#include "vehicle_control_wrapper_c.h"
#include "vehicle_model_c.h"


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化车身控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_Initialize(WorkVehCtlInstance_t* const instance) {
  Phoenix_AdMsg_ClearChassisCtlCmd(&(instance->veh_ctl_request));

  Phoenix_AdMsg_ClearChassis(&(instance->chassis));
  Phoenix_AdMsg_ClearSpecialChassisInfo(&(instance->special_chassis));
  Phoenix_AdMsg_ClearPlanningResult(&(instance->planning_result));
  phoenix_com_memset(&(instance->module_status_list), 0,
                     sizeof(instance->module_status_list));

  Phoenix_AdMsg_ClearChassisCtlCmd(&(instance->chassis_ctl_cmd));

  Phoenix_Common_Os_Mutex_Init(&(instance->lock_veh_ctl_info));

  Phoenix_LonCtl_Initialize();
  Phoenix_VehCtl_Initialize();
}


/*
 * @brief 请求开启车身控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_StartRobotCtl(
    WorkVehCtlInstance_t* const instance) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.start_robotic_ctl = 2;
  instance->veh_ctl_request.turn_lamp = VEH_TURN_LAMP_OFF;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求关闭车身控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_StopRobotCtl(
    WorkVehCtlInstance_t* const instance) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.start_robotic_ctl = 1;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 允许/拒绝远程控制
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_EnableRemoteCtl(
    WorkVehCtlInstance_t* const instance, Int8_t enable) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.enable_remote_ctl = enable;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求开启/关闭调试模式
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_EnableDirectCtl(
    WorkVehCtlInstance_t* const instance, Int8_t enable) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.enable_direct_ctl = enable;

  if (!enable) {
    // enable_test_mode = false;
    instance->veh_ctl_request.enable_acc = 1;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求开启/关闭转向系统控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_EnableEps(
    WorkVehCtlInstance_t* const instance, Int8_t enable) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.enable_eps = enable;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求开启/关闭加速系统控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_EnableThrottleSys(
    WorkVehCtlInstance_t* const instance, Int8_t enable) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.enable_throttle_sys = enable;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求开启/关闭制动系统控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_EnableEbs(
    WorkVehCtlInstance_t* const instance, Int8_t enable) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.enable_ebs = enable;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求方向盘角度控制
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_TurnSteeringWheel(
    WorkVehCtlInstance_t* const instance,
    Float32_t angle, Float32_t speed) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.steering_wheel_angle =
        Phoenix_VehModel_ClampMaxSteeringAngle(angle);
    instance->veh_ctl_request.steering_wheel_speed = speed;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

void Phoenix_Work_VehicleCtl_TurnSteeringWheelDirectly(
    WorkVehCtlInstance_t* const instance,
    Float32_t angle, Float32_t speed) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (!instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.steering_wheel_angle =
        Phoenix_VehModel_ClampMaxSteeringAngle(angle);
    instance->veh_ctl_request.steering_wheel_speed = speed;

    ChassisCtlCmd_t cmd;
    Phoenix_SharedData_GetChassisCtlCmd(&cmd);

    cmd.steering_wheel_angle = instance->veh_ctl_request.steering_wheel_angle;
    cmd.steering_wheel_speed = instance->veh_ctl_request.steering_wheel_speed;

    Phoenix_SharedData_SetChassisCtlCmd(&cmd);
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求速度控制
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_SpeedUp(
    WorkVehCtlInstance_t* const instance, Float32_t spd, Float32_t acc) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.velocity = spd;
    instance->veh_ctl_request.acceleration = acc;

    instance->veh_ctl_request.enable_acc = 1;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求加速(加油门)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_Accelerate(
    WorkVehCtlInstance_t* const instance, Float32_t value) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.acc_value = value;
    instance->veh_ctl_request.enable_acc = 0;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求制动(踩刹车)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_Brake(
    WorkVehCtlInstance_t* const instance, Float32_t value) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.brake_value = value;

  instance->veh_ctl_request.enable_direct_ctl = 1;
  instance->veh_ctl_request.enable_acc = 0;
  // enable_test_mode = false;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求变档
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_ChangeGear(
    WorkVehCtlInstance_t* const instance, Int8_t value) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (instance->veh_ctl_request.enable_direct_ctl) {
    instance->veh_ctl_request.gear = value;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求转向灯控制
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_SetTurnLamp(
    WorkVehCtlInstance_t* const instance, Int8_t value) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.turn_lamp = value;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}

/*
 * @brief 请求雨刮器控制
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_VehicleCtl_SetWiper(
    WorkVehCtlInstance_t* const instance, Int8_t value) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  instance->veh_ctl_request.wiper = value;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));
}


/*
 * @brief 开始一次车身控制功能（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Work_VehicleCtl_DoWork(WorkVehCtlInstance_t* const instance) {
  Int8_t enable_handle_internal_error = 1;

  Int64_t curr_timestamp = 0;
  Int32_t chassis_ctl_status = 0;
  VehCtlDataSource_t veh_ctl_data_src;
  const ChassisCtlCmd_t* ctl_cmd = Nullptr_t;

  Int32_t tar_type_mp = 0;

  /// 读取所需的数据
  Phoenix_SharedData_GetChassisCtlStatus(&chassis_ctl_status);
  Phoenix_SharedData_GetChassis(&(instance->chassis));
  Phoenix_SharedData_GetSpecialChassisInfo(&(instance->special_chassis));
  Phoenix_SharedData_GetPlanningResult(&(instance->planning_result));
  Phoenix_SharedData_GetModuleStatusList(&(instance->module_status_list));

  /// 读取时间戳
  curr_timestamp = Phoenix_Common_GetClockNowMs();

  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(instance->lock_veh_ctl_info));

  if (!instance->veh_ctl_request.enable_direct_ctl) {
    // 正常工作模式 (非直接控制模式)

    // 驾驶模式请求
    switch (instance->planning_result.tar_driving_mode) {
    case (VEH_DRIVING_MODE_MANUAL):
      instance->veh_ctl_request.start_robotic_ctl = 1;
      break;
    case (VEH_DRIVING_MODE_ROBOTIC):
      instance->veh_ctl_request.start_robotic_ctl = 2;
      break;
    default:
      // don't change start robotic request
      break;
    }
    // 使能转向系统
    instance->veh_ctl_request.enable_eps = instance->planning_result.enable_eps;
    // 使能油门系统
    instance->veh_ctl_request.enable_throttle_sys =
        instance->planning_result.enable_throttle_sys;
    // 使能制动系统
    instance->veh_ctl_request.enable_ebs = instance->planning_result.enable_ebs;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N) && \
    (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
    // AD-ECU Driving mode   1:Power off / 2:Not Ready / 3:Ready / 4:Engage / 5:Fallback
#if 0
    // AD-ECU平台下D17-B2车型暂时只能从车身侧启动
    if (Phoenix_AdMsg_IsChassisInRoboticMode(chassis_ctl_status)) {
      // 处于自动模式下
      if ((4 != instance->special_chassis.df_d17.ADCU_Mode_Sts) &&
          (5 != instance->special_chassis.df_d17.ADCU_Mode_Sts)) {
        // 请求退出自动模式
        instance->veh_ctl_request.start_robotic_ctl = 1;
      }
    } else {
      // 处于手动模式下
      if ((4 == instance->special_chassis.df_d17.ADCU_Mode_Sts) ||
          (5 == instance->special_chassis.df_d17.ADCU_Mode_Sts)) {
        // 请求进入自动模式
        instance->veh_ctl_request.start_robotic_ctl = 2;
      }
    }
#endif
    if ((4 == instance->special_chassis.df_d17.ADCU_Mode_Sts) ||
        (5 == instance->special_chassis.df_d17.ADCU_Mode_Sts)) {
      // AD-ECU 处于自动模式下
      // 使能转向系统
      instance->veh_ctl_request.enable_eps = 1;
      // 使能油门系统
      instance->veh_ctl_request.enable_throttle_sys = 1;
      // 使能制动系统
      instance->veh_ctl_request.enable_ebs = 1;
    } else {
      // AD-ECU 处于手动模式下
      // 使能转向系统
      instance->veh_ctl_request.enable_eps = 0;
      // 使能油门系统
      instance->veh_ctl_request.enable_throttle_sys = 0;
      // 使能制动系统
      instance->veh_ctl_request.enable_ebs = 0;
    }
#endif

    // 档位请求
    if (VEH_GEAR_INVALID != instance->planning_result.tar_gear) {
      instance->veh_ctl_request.gear = instance->planning_result.tar_gear;
    }

    instance->veh_ctl_request.enable_acc = 1;
    instance->veh_ctl_request.release_throttle =
        instance->planning_result.release_throttle;

    instance->veh_ctl_request.velocity = instance->planning_result.tar_v;
    instance->veh_ctl_request.acceleration = instance->planning_result.tar_a;

    instance->veh_ctl_request.turn_lamp =
        instance->planning_result.tar_turn_lamp;
  }

  /// TODO: 远程控制进入/退出自动驾驶
#if 0
  if (instance->veh_ctl_request.enable_remote_ctl) {
    // 车身控制按钮进入/退出自动驾驶
    // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
    if (1 == instance->special_chassis.start_adas) {
      // 退出自动驾驶
      instance->veh_ctl_request.start_robotic_ctl = 1;
    } else if (2 == instance->special_chassis.start_adas) {
      // 进入自动驾驶
      instance->veh_ctl_request.start_robotic_ctl = 2;
    } else {
      // nothing to do
    }
  }
#endif

  // Check module status
  if (!instance->veh_ctl_request.enable_direct_ctl &&
      enable_handle_internal_error) {
    for (Int32_t i = 0;
         i < instance->module_status_list.module_status_num; ++i) {
      const ModuleStatus_t* module =
          &(instance->module_status_list.module_status_list[i]);
      Int8_t module_exception = 0;
      switch (module->sub_module_id) {
#if (!ENTER_PLAYBACK_MODE)
      case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_0):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
      case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_1):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
      case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_2):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
      case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_3):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
#endif
      case (SUB_MODULE_ID_CONTROL_RECV_MSG_PLANNING_RESULT):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
      case (SUB_MODULE_ID_CONTROL_LAT_CTL):
        if (module->timeout || (MODULE_STATUS_OK != module->status)) {
          module_exception = 1;
        }
        break;
      default:
        break;
      }
      if (module_exception) {
        if (instance->veh_ctl_request.velocity > 0) {
          instance->veh_ctl_request.velocity = 0;
        }
        if (instance->veh_ctl_request.acceleration > -1.0F) {
          instance->veh_ctl_request.acceleration = -1.0F;
        }
        break;
      }
    }
  }

  // 填充数据
  veh_ctl_data_src.timestamp = curr_timestamp;
  veh_ctl_data_src.chassis_ctl_status = chassis_ctl_status;

  instance->veh_ctl_request.ramp_slope_value = instance->planning_result.ramp_slope_value; // 20221226 guanj
  tar_type_mp = Phoenix_AdMsg_GetVelPlanngingTarType(instance->planning_result.planning_status);
  if (tar_type_mp == 7) { // AEB
    instance->veh_ctl_request.ebs_ungercy = 100.0f;
  } else {
    instance->veh_ctl_request.ebs_ungercy = 0.0f;
  }

  veh_ctl_data_src.request = &(instance->veh_ctl_request);
  veh_ctl_data_src.chassis = &(instance->chassis);

  // 车身控制
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_GOYU)
  Phoenix_VehCtl_CalcVehCtlValue(&veh_ctl_data_src);
#elif (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  Phoenix_VehCtl_CalcVehCtlValue(&veh_ctl_data_src,&(instance->lon_ctl_info));
#endif

  ctl_cmd = Phoenix_VehCtl_GetVehCtlCmd();

  // 写入控制指令
  phoenix_com_memcpy(&(instance->chassis_ctl_cmd),
                     ctl_cmd, sizeof(ChassisCtlCmd_t));

  // 开启/停止自动驾驶后清除相应的请求状态
  instance->veh_ctl_request.start_robotic_ctl = 0;

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(instance->lock_veh_ctl_info));

  // 保存控制指令
  Phoenix_SharedData_SetChassisCtlCmd(ctl_cmd);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  // Add longtitude debug information
  Phoenix_SharedData_SetLongtitudeControlInfo(&(instance->lon_ctl_info));
#endif

  return (0);
}
