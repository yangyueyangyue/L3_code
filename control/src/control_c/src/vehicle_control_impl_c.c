/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vehicle_control_impl.c
 * @brief      车身控制器
 * @details    实现车身控制器
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
#include "vehicle_control_impl_c.h"
#include "longitudinal_control_wrapper_c.h"

#include "utils/macros.h"

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
#include "Vehicle_Definiton.h"
#include "LgtCtrl.h"
#include "communication_c/shared_data_c.h"
#endif

/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
double CoChs_aBrkThd_C = 0.05F;

/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化车身控制器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_VehCtlImpl_Initialize(
    VehCtlImplInstance_t* const instance) {
  Phoenix_AdMsg_ClearChassisCtlCmd(&(instance->ctl_cmd));

  Phoenix_LonCtl_Initialize();
}

/*
 * @brief 计算车身控制指令
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_GOYU)
Int32_t Phoenix_VehCtlImpl_CalcVehCtlValue(
    VehCtlImplInstance_t* const instance,
    const VehCtlDataSource_t* data_source) {

  Float32_t cur_v = data_source->chassis->v;
  Float32_t tar_v = data_source->request->velocity;
  Float32_t tar_a = data_source->request->acceleration;

  // 启动/停止自动驾驶
  instance->ctl_cmd.start_robotic_ctl = data_source->request->start_robotic_ctl;

  // 使能转向/油门/制动控制系统
  instance->ctl_cmd.enable_eps =
      data_source->request->enable_eps;
  instance->ctl_cmd.enable_throttle_sys =
      data_source->request->enable_throttle_sys;
  instance->ctl_cmd.enable_ebs = data_source->request->enable_ebs;

  // 使能直接控制模式
  instance->ctl_cmd.enable_direct_ctl =
      data_source->request->enable_direct_ctl;
  // 使能速度控制
  instance->ctl_cmd.enable_acc = data_source->request->enable_acc;

  // 方向盘转角
  instance->ctl_cmd.steering_wheel_angle =
      data_source->request->steering_wheel_angle;
  instance->ctl_cmd.steering_wheel_speed =
      data_source->request->steering_wheel_speed;
  // 灯光控制
  instance->ctl_cmd.turn_lamp = data_source->request->turn_lamp;
  instance->ctl_cmd.brake_lamp = data_source->request->brake_lamp;
  // 驻车控制
  instance->ctl_cmd.epb_status = data_source->request->epb_status;

  // 档位控制
  switch (data_source->request->gear) {
  case (VEH_GEAR_P):
    if ((cur_v < 0.1F) &&
        ((instance->ctl_cmd.brake_value > 5) ||
         (data_source->chassis->brake_pedal_value > 5))) {
      instance->ctl_cmd.gear = data_source->request->gear;
    }
    break;
  case (VEH_GEAR_N):
    instance->ctl_cmd.gear = data_source->request->gear;
    break;
  case (VEH_GEAR_R):
    if ((cur_v < 0.1F) &&
        /*((instance->ctl_cmd.brake_value > 5) ||
         (data_source->chassis->brake_pedal_value > 5)) &&*/
        (VEH_GEAR_N == data_source->chassis->gear)) {
      instance->ctl_cmd.gear = data_source->request->gear;
    }
    break;
  case (VEH_GEAR_D):
    if (/*(cur_v < 0.1) && (cmd.brake_value > 5) &&*/
        (VEH_GEAR_N == data_source->chassis->gear)) {
      instance->ctl_cmd.gear = data_source->request->gear;
    } else if (VEH_GEAR_D == data_source->chassis->gear) {
      instance->ctl_cmd.gear = VEH_GEAR_D;
    }
    break;
  default:
    break;
  }

  instance->ctl_cmd.velocity = tar_v;
  instance->ctl_cmd.acceleration = tar_a;
  instance->ctl_cmd.acc_value = data_source->request->acc_value;
  instance->ctl_cmd.brake_value = data_source->request->brake_value;

  // 方向盘人工介入后，退出自动驾驶
  Int32_t enable_steering_wheel_interrupt = 1;
  if (enable_steering_wheel_interrupt) {
#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
    static Int32_t prev_eps_status = VEH_EPS_STATUS_MANUAL;
    static Int32_t eps_interrupt_count = 0;
    if (VEH_EPS_STATUS_ROBOTIC == prev_eps_status) {
      if (VEH_EPS_STATUS_MANUAL == data_source->chassis->eps_status ||
          VEH_EPS_STATUS_INVALID == data_source->chassis->eps_status) {
        eps_interrupt_count = 1;
      }
    }
    if ((VEH_EPS_STATUS_MANUAL == data_source->chassis->eps_status ||
         VEH_EPS_STATUS_INVALID == data_source->chassis->eps_status) &&
        (eps_interrupt_count > 0)) {
      eps_interrupt_count++;
      if (eps_interrupt_count > 10) {
        eps_interrupt_count = 0;

        instance->ctl_cmd.start_robotic_ctl = 1;
      }
    } else {
      eps_interrupt_count = 0;
    }
    prev_eps_status = data_source->chassis->eps_status;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
    static Int32_t prev_eps_status = VEH_EPS_STATUS_MANUAL;
    if ((VEH_EPS_STATUS_ROBOTIC == prev_eps_status) &&
        ((VEH_EPS_STATUS_MANUAL == data_source->chassis->eps_status) ||
         (VEH_EPS_STATUS_INVALID == data_source->chassis->eps_status)) &&
        instance->ctl_cmd.enable_eps) {
      printf("===== 方向盘人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
    prev_eps_status = data_source->chassis->eps_status;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
    static Int32_t prev_eps_status = VEH_EPS_STATUS_MANUAL;
    if ((VEH_EPS_STATUS_ROBOTIC == prev_eps_status) &&
        (VEH_EPS_STATUS_MANUAL_INTERRUPT == data_source->chassis->eps_status)) {
      printf("===== 方向盘人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
    prev_eps_status = data_source->chassis->eps_status;
#endif
  }

  // 踩刹车踏板，退出自动驾驶
  Int32_t enable_brake_interrupt = 1;
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  /// TODO: 西电蚂蚁需要在自动驾驶启动后，踩刹车解除驻车
  if (enable_brake_interrupt) {
    if ((data_source->chassis->brake_pedal_value > 5) &&
        ((VEH_GEAR_D == data_source->chassis->gear) ||
         (VEH_GEAR_R == data_source->chassis->gear)) &&
        (data_source->chassis->v > 3.0F/3.6F) &&
        instance->ctl_cmd.enable_throttle_sys) {
      // instance->ctl_cmd.gear = ad_msg::VEH_GEAR_N;
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
  }
#else
  if (enable_brake_interrupt) {
    if ((data_source->chassis->brake_pedal_value > 5)) {
      printf("===== 刹车人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
  }
#endif

  // 进入/退出自动驾驶后，关闭转向灯
  if (1 == instance->ctl_cmd.start_robotic_ctl) {
    instance->ctl_cmd.turn_lamp = VEH_TURN_LAMP_OFF;
  } else if (2 == instance->ctl_cmd.start_robotic_ctl) {
    instance->ctl_cmd.turn_lamp = VEH_TURN_LAMP_OFF;
  } else {
    // nothing to do
  }

  // 车速控制
  if (instance->ctl_cmd.enable_acc &&
      Phoenix_AdMsg_IsChassisInRoboticMode(data_source->chassis_ctl_status)) {
    LonCtlDataSource_t data_src;
    data_src.timestamp = data_source->timestamp;
    data_src.cur_v = data_source->chassis->v;
    data_src.cur_a = data_source->chassis->a;
    data_src.tar_v = instance->ctl_cmd.velocity;
    data_src.tar_a = instance->ctl_cmd.acceleration;
    // 底盘信息
    data_src.chassis.gross_weight = data_source->chassis->gross_weight;
    data_src.chassis.gear_number = data_source->chassis->gear_number;

    Phoenix_LonCtl_CalcLonCtlValue(
          &data_src, &instance->ctl_cmd.acc_value,
          &instance->ctl_cmd.brake_value);

    // 不处于前进或后退档，则不加速
    if ((VEH_GEAR_D != data_source->chassis->gear) &&
        (VEH_GEAR_R != data_source->chassis->gear)) {
      Phoenix_LonCtl_ResetAccValue(0.0F);
      instance->ctl_cmd.acc_value = 0;
    }
    // 决策模块请求松油门减速
    if (data_source->request->release_throttle) {
      Phoenix_LonCtl_ResetAccValue(0.0F);
      instance->ctl_cmd.acc_value = 0;
    }
    // 踩油门踏板退出制动状态，并根据油门踏板加速
    if (data_source->chassis->acc_pedal_value > 5) {
      instance->ctl_cmd.acc_value =
          data_source->chassis->acc_pedal_value;
      if (instance->ctl_cmd.brake_value > 5) {
        if (instance->ctl_cmd.acc_value > 20) {
          instance->ctl_cmd.acc_value = 20;
        }
      } else {
        if (instance->ctl_cmd.acc_value > 50) {
          instance->ctl_cmd.acc_value = 50;
        }
      }

      Phoenix_LonCtl_ResetBrakeValue(0.0F);
      instance->ctl_cmd.brake_value = 0;
    }

    if (0 == instance->ctl_cmd.enable_throttle_sys) {
      Phoenix_LonCtl_ResetAccValue(0.0F);
    }
    if (0 == instance->ctl_cmd.enable_ebs) {
      Phoenix_LonCtl_ResetBrakeValue(0.0F);
    }
  } else {
    Phoenix_LonCtl_ResetAccValue(0.0F);
    Phoenix_LonCtl_ResetBrakeValue(0.0F);
  }

  // 踩刹车踏板，不加油门
  if (data_source->chassis->brake_pedal_value > 5) {
    instance->ctl_cmd.acc_value = 0;
    Phoenix_LonCtl_ResetAccValue(0.0F);
  }
  // 正在制动时，不加油门
  if (instance->ctl_cmd.brake_value > 0) {
    instance->ctl_cmd.acc_value = 0;
  }

  /// TODO: E-Stop
  if (VEH_E_STOP_ON == data_source->chassis->e_stop) {
    instance->ctl_cmd.acc_value = 0;
    instance->ctl_cmd.brake_value = 20;
    instance->ctl_cmd.turn_lamp = VEH_TURN_LAMP_EMERGENCY;

    Phoenix_LonCtl_ResetAccValue(0.0F);
    Phoenix_LonCtl_ResetBrakeValue(0.0F);
  }

  return (0);
}
 
#elif (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
Int32_t Phoenix_VehCtlImpl_CalcVehCtlValue(VehCtlImplInstance_t* const instance, const VehCtlDataSource_t* data_source, LongtitudeControlInfo_t* const status_info) {
  Float32_t cur_v = data_source->chassis->v;

  // 启动/停止自动驾驶
  instance->ctl_cmd.start_robotic_ctl = data_source->request->start_robotic_ctl;

  // 使能转向/油门/制动控制系统
  instance->ctl_cmd.enable_eps = data_source->request->enable_eps;
  instance->ctl_cmd.enable_throttle_sys = data_source->request->enable_throttle_sys;
  instance->ctl_cmd.enable_ebs = data_source->request->enable_ebs;

  // 使能直接控制模式
  instance->ctl_cmd.enable_direct_ctl = data_source->request->enable_direct_ctl;
  // 使能速度控制
  instance->ctl_cmd.enable_acc = data_source->request->enable_acc;

  // 方向盘转角
  instance->ctl_cmd.steering_wheel_angle = data_source->request->steering_wheel_angle;
  instance->ctl_cmd.steering_wheel_speed = data_source->request->steering_wheel_speed;
  // 灯光控制
  instance->ctl_cmd.turn_lamp = data_source->request->turn_lamp;
  instance->ctl_cmd.brake_lamp = data_source->request->brake_lamp;
  // 驻车控制
  instance->ctl_cmd.epb_status = data_source->request->epb_status;

  instance->ctl_cmd.enable_remote_ctl = data_source->request->enable_remote_ctl;
  instance->ctl_cmd.release_throttle = data_source->request->release_throttle;
  instance->ctl_cmd.steering_wheel_torque = data_source->request->steering_wheel_torque;
  instance->ctl_cmd.wiper = data_source->request->wiper;

  // 档位控制
  switch (data_source->request->gear) {
  case (VEH_GEAR_P):
    if ((cur_v < 0.1F) &&
        ((instance->ctl_cmd.brake_value > 5) ||
         (data_source->chassis->brake_pedal_value > 5))) {
      instance->ctl_cmd.gear = data_source->request->gear;
    }
    break;
  case (VEH_GEAR_N):
    instance->ctl_cmd.gear = data_source->request->gear;
    break;
  case (VEH_GEAR_R):
    if ((cur_v < 0.1F) &&
        /*((instance->ctl_cmd.brake_value > 5) ||
         (data_source->chassis->brake_pedal_value > 5)) &&*/
        (VEH_GEAR_N == data_source->chassis->gear)) {
      instance->ctl_cmd.gear = data_source->request->gear;
    }
    break;
  case (VEH_GEAR_D):
    if (/*(cur_v < 0.1) && (cmd.brake_value > 5) &&*/
        (VEH_GEAR_N == data_source->chassis->gear)) {
      instance->ctl_cmd.gear = data_source->request->gear;
    } else if (VEH_GEAR_D == data_source->chassis->gear) {
      instance->ctl_cmd.gear = VEH_GEAR_D;
    }
    break;
  default:
    break;
  }
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  ChassisDfD17_adrecord ad_record_chassis;
  Phoenix_SharedData_GetChassisDfD17adrecordInfo(&ad_record_chassis);
  /*
  printf("\n share_ad_mode is %d, \
           share_tar_vel_set is s%d \
          , share_tar_trajectory_type is %d \
          , share_tar_vel_type is %d \
          , share_vel_obj_dist is %lf \
          , share_vel_obj_vel is %lf \n",
           ad_record_chassis.ad_mode,
           ad_record_chassis.tar_vel_set,
           ad_record_chassis.tar_trajectory_type,
           ad_record_chassis.tar_vel_type,
           ad_record_chassis.vel_obj_dist,
           ad_record_chassis.vel_obj_vel);
   */
  #endif
  instance->ctl_cmd.velocity = data_source->request->velocity;
  instance->ctl_cmd.acceleration = data_source->request->acceleration;
  instance->ctl_cmd.acc_value = data_source->request->acc_value;
  instance->ctl_cmd.brake_value = data_source->request->brake_value;

  // 方向盘人工介入后，退出自动驾驶
  Int32_t enable_steering_wheel_interrupt = 0;
  if (enable_steering_wheel_interrupt) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
    static Int32_t prev_eps_status = VEH_EPS_STATUS_MANUAL;
    if ((VEH_EPS_STATUS_ROBOTIC == prev_eps_status) &&
        ((VEH_EPS_STATUS_MANUAL == data_source->chassis->eps_status) ||
         (VEH_EPS_STATUS_INVALID == data_source->chassis->eps_status)) &&
        instance->ctl_cmd.enable_eps) {
      printf("===== 方向盘人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
    prev_eps_status = data_source->chassis->eps_status;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
    static Int32_t prev_eps_status = VEH_EPS_STATUS_MANUAL;
    if ((VEH_EPS_STATUS_ROBOTIC == prev_eps_status) &&
        (VEH_EPS_STATUS_MANUAL_INTERRUPT == data_source->chassis->eps_status)) {
      printf("===== 方向盘人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
    prev_eps_status = data_source->chassis->eps_status;
#endif
  }

  // 踩刹车踏板，退出自动驾驶
  Int32_t enable_brake_interrupt = 1;
  if (enable_brake_interrupt) {
    if ((data_source->chassis->brake_pedal_value > 5)) {
      printf("===== 刹车人工接管，退出自动驾驶模式 ======\n");
      instance->ctl_cmd.start_robotic_ctl = 1;
    }
  }

  // 进入/退出自动驾驶后，关闭转向灯
  if (1 == instance->ctl_cmd.start_robotic_ctl) {
    instance->ctl_cmd.turn_lamp = VEH_TURN_LAMP_OFF;
  } else if (2 == instance->ctl_cmd.start_robotic_ctl) {
    instance->ctl_cmd.turn_lamp = VEH_TURN_LAMP_OFF;
  } else {
    // nothing to do
  }

  SpdPln_vTrgSpd_mp = instance->ctl_cmd.velocity;
  SpdPln_aTrgAcc_mp = instance->ctl_cmd.acceleration;
  VehDa_rSlop_mp = data_source->request->ramp_slope_value;
  double final_target_speed;
  if (ACC_RXFlag == 0) {
    final_target_speed = SpdPln_vTrgSpd_mp;
  } else {
    final_target_speed = SpdPln_vTrgSpd;
  }

  if (instance->ctl_cmd.enable_acc &&
      Phoenix_AdMsg_IsChassisInRoboticMode(data_source->chassis_ctl_status)) {
    if(final_target_speed < 0.25f && VehDa_vEgoSpd_mp < ACCS_vStpThd_C) {
      BhvCrdn_numBhvID_mp = 11;
    } else if (BhvCrdn_numBhvID_mp == 11 && final_target_speed > 0.5f) {
      BhvCrdn_numBhvID_mp = 11;
    } else {
      BhvCrdn_numBhvID_mp = 2;
    }
  } else {
    BhvCrdn_numBhvID_mp = 0;
  }

  DFCV_SpecialChassisInfo_t dfcv_special_chassis;
  Phoenix_SharedData_GetDFCVSpecialChassisInfo(&dfcv_special_chassis);

  VehDa_rBrkPedl_mp = data_source->chassis->brake_pedal_value;
  VehDa_mWght_mp = (real32_T)dfcv_special_chassis.vehicle_mass;
  VehDa_vEgoSpd_mp = data_source->chassis->v * 3.6;
  VehDa_aEgoAcc_mp = data_source->chassis->a;
  VehDa_stTraCurGear_mp =data_source->chassis->gear_number;
  VehDa_rTraCurGear_mp = (real32_T)dfcv_special_chassis.current_gear_ratio;
  VehDa_stCluSwt_mp = dfcv_special_chassis.clutch_switch;
  VehDa_prcTrqEngNomFric_mp = dfcv_special_chassis.nominal_fricton_troque_percent;
  VehDa_prcTrqEstimdLoss_mp = dfcv_special_chassis.estimated_lossed_torque_percent;
  VehDa_stSrcBrk_mp = dfcv_special_chassis.source_address_brake_control_device;
  VehDa_prcActuTrq_mp = dfcv_special_chassis.actual_engine_torque_percent;
  VehDa_prcDrvrDmdTrq_mp = dfcv_special_chassis.driver_damand_torque_percent;
  VehDa_stSrcEngCtrl_mp = dfcv_special_chassis.source_address_engine_control_device;
  VehDa_pFrontLeft_mp = (real32_T)dfcv_special_chassis.brake_pressure_lf;
  VehDa_nEngSpd_mp = data_source->chassis->engine_speed;
  VehDa_stTrlrCnctn_mp = dfcv_special_chassis.trailer_connected_status;
  VehDa_stTraSht_mp = dfcv_special_chassis.transmission_shift_status;
  VehDa_stTraEgd_mp = dfcv_special_chassis.transmission_engage_status;
  VehDa_stTraSelGear_mp = dfcv_special_chassis.transmission_selected_gear;
  VehDa_stTraTrqLim_mp = dfcv_special_chassis.tcu_engine_control_mode;
  VehDa_rAccrPedl_mp = data_source->chassis->acc_pedal_value;

  VehDa_nOutpShaft_mp = dfcv_special_chassis.tcu_output_shaft_speed;
  // TCU.ETC7_TCU.4A03.TransReadyForBrakeRelease
  VehDa_stBrkReady4Rls_mp = dfcv_special_chassis.trans_ready_for_brake_release;

  if (instance->ctl_cmd.enable_ebs || instance->ctl_cmd.enable_throttle_sys) {
    Sys_stADMd_mp = 3;
  } else {
    Sys_stADMd_mp = 0;
  }

  ACCS_vSetFrmSys_mp = ADCU_set_speed_C;

  if ((VehDa_mWght_mp > 20000) && (VehDa_mWght_mp < 600000)) {
    VehDa_stTrlrCnctn_mp = 1;
  } else {
    VehDa_stTrlrCnctn_mp = 0;
  }

  DRMC_lCurr = 0;

  if (VehDa_stSrcEngCtrl_mp == 3) {
    VehDa_stTraTrqLim_mp = 3;
  } else {
    VehDa_stTraTrqLim_mp = 0;
  }

  SpdPln_lTrgLngErr_mp = 0;
  EnvDetn_idCllsnTar = 0;

  PlanningResult_t planning_result;
  Phoenix_SharedData_GetPlanningResult(&planning_result);
  
  instance->ctl_cmd.tar_type = ((planning_result.planning_status[0]>>20)&0x0F);
  instance->ctl_cmd.release_throttle = planning_result.release_throttle;

  EnvDetn_lCllsnTarLgt = (real32_T)planning_result.front_nearest_obj_distance;
  EnvDetn_vCllsnTarRel = (real32_T)VehDa_vEgoSpd_mp / 3.6f - planning_result.front_nearest_obj_speed;
  float tar_throttle = planning_result.tar_throttle;
  int vel_plan_type = ((planning_result.planning_status[0] >> 20) & 0x0F);

  VehDa_agPitch_mp = (real32_T)planning_result.pitch;
  VehDa_varPitch_mp = (real32_T)planning_result.pitch_variance;

  SpdPln_stTarType_mp = instance->ctl_cmd.tar_type;
  SpdPln_stReleaseThrottle_mp = instance->ctl_cmd.release_throttle;
  SpdPln_pcc_tar_throttle_mp = tar_throttle;

  int32_T map_type = Phoenix_AdMsg_GetTrjPlanngingRoadTpye(planning_result.planning_status);
  if ((TRJ_PLAN_ROAD_BUILED_TYPE_HD_MAP == map_type) || (TRJ_PLAN_ROAD_BUILED_TYPE_MIXED_HD_MAP == map_type)) {
    VehDa_stMapIsAvail_mp = 1;
  } else {
    VehDa_stMapIsAvail_mp = 0;
  }

  // the main longitudinal control function
  LgtCtrl_step();

  // 驻车时不请求缓速器制动
  if (BhvCrdn_numBhvID_mp == 11) {
    instance->ctl_cmd.ebi_control_mode = 0;
  } else {
    instance->ctl_cmd.ebi_control_mode = 2;
  }

  if (SpdPln_aTrgAcc <= -1.0f || VehDa_vEgoSpd < 20.0f || EBS_aReq < -1.5f) {
    instance->ctl_cmd.ebs_ungercy = 100.0f;
  } else {
    instance->ctl_cmd.ebs_ungercy = 0.0f;
  }
  // TODO: 控制EBS最大减速度限制
  if (EBS_aReq < -2.0f) {
    EBS_aReq = -2.0f;
  }
  
  // 车速控制
  if (instance->ctl_cmd.enable_acc && Phoenix_AdMsg_IsChassisInRoboticMode(data_source->chassis_ctl_status)) {
    instance->ctl_cmd.acc_value = Eng_rAccrPedlReq;
    instance->ctl_cmd.brake_value = (- EBS_aReq) / 10.0 * 100;
    
    if (15 == vel_plan_type) { // PCC
      // instance->ctl_cmd.release_throttle = planning_result.release_throttle;
      // instance->ctl_cmd.acc_value = tar_throttle;
      instance->ctl_cmd.brake_value = 0.0F;
    }
  
    if (15 != vel_plan_type) {
      if (instance->ctl_cmd.acc_value > 90.0f) {
        instance->ctl_cmd.acc_value = 90.0f;
      }
    }
  
    if (CoChs_stCdn == 1 || (CoChs_stCdn == 3 && EBS_aReq <= CoChs_aBrkThd_C) || CoChs_stCdn == 5
      || CoChs_stCdn == 6 || CoChs_stCdn == 7 || CoChs_stCdn == 10 || (CoChs_stCdn == 2 && CoChs_stStarting == 1)) {
      instance->ctl_cmd.ebs_mode_enable = 2;
    } else {
      instance->ctl_cmd.ebs_mode_enable = 0;
    }
    
    if (instance->ctl_cmd.brake_value < -30) {
      instance->ctl_cmd.brake_value = -30;
    } else if (instance->ctl_cmd.brake_value > 100) {
      instance->ctl_cmd.brake_value = 100;
    }
  }

  // add Control topic's debug message
  status_info->debug_VecDa_aCalcd_mp = 0;
  status_info->debug_VDC_facDrvTrmJ_mp = VDC_facDrvTrmJ_mp;
  status_info->debug_speed_integ_value = VMC_aAccnGvnrI_mp;
  status_info->debug_speed_propo_value = VMC_aAccnGvnrP_mp;
  status_info->debug_speed_value_a_req = VMC_aReq;
  status_info->debug_speed_integ_freeze_status = VMC_stAccnGvnrIntglFrz_mp;
  status_info->debug_speed_inieg_init_status = VMC_stAccnGvnrIntglIni_mp;
  status_info->debug_speed_error = VMC_vDif_mp;
  status_info->debug_longitudinal_control_status = CoChs_stCdn;
  
  status_info->debug_engine_torque_raw = Eng_prcTrqRaw_mp2;
  status_info->debug_engine_torque = Eng_prcTrqReq;
  status_info->debug_engine_torque_nfm = Eng_trqReq;

  status_info->debug_Eng_rAccrPedlReq = Eng_rAccrPedlReq;
  status_info->debug_EBS_aReq = EBS_aReq;
  status_info->debug_vehicle_max_acce = 0;

  status_info->debug_fr_rolling_res_mp = VDC_frRollngRes_mp;
  status_info->debug_fr_air_drag_mp = VDC_frAirDrag_mp;
  status_info->debug_fr_slp_res_mp = VDC_frSlpRes_mp;
  status_info->debug_fr_acc_res_mp = VDC_frAccnRes_mp;
  status_info->debug_fr_cmp_mp = VDC_frCmp_mp;
  status_info->debug_fr_eng_fric_mp = VDC_frEngFric_mp;

  status_info->debug_speedup_shift_status = Tra_stShiftWthSpdUp;
  status_info->debug_tcu_torque_status = Tra_stTrqLimWthTCU;
  status_info->debug_cochs_stnegtrqreq_mp = 0;

  status_info->debug_AD_Status = BhvCrdn_numBhvID_mp;

  status_info->debug_Cochs_stStarting = CoChs_stStarting;
  status_info->debug_Vmc_aAccnGvnrD_mp = VMC_aAccnGvnrD_mp;
  status_info->debug_Vdc_aReqNom = VDC_aReqNom;
  status_info->debug_Tra_stShiftWhtSpdUp =Tra_stShiftWthSpdUp;

  status_info->debug_Tra_stTrqLimWthTcu = Tra_stTrqLimWthTCU;
  status_info->debug_Tra_trqReq = Tra_trqReq;
  // status_info->debug_acc_pid = VMC_Accnpid;

  // chassis info 
  status_info->debug_SpdPln_lTrgLngErr_mp = SpdPln_lTrgLngErr_mp;
  status_info->debug_VehDa_nOutpShaft_mp = VehDa_nOutpShaft_mp;
  status_info->debug_VehDa_stBrkReady4Rls_mp = VehDa_stBrkReady4Rls_mp; 

  status_info->debug_throttle_real = instance->ctl_cmd.acc_value;

  // adecu debug
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  status_info->adecu_debug_ad_mode = ad_record_chassis.ad_mode;
  status_info->adecu_debug_tar_vel_set = ad_record_chassis.tar_vel_set;
  status_info->adecu_debug_tar_trajectory_type = ad_record_chassis.tar_trajectory_type;
  status_info->adecu_debug_tar_vel_type = ad_record_chassis.tar_vel_type;
  status_info->adecu_debug_vel_obj_dist = ad_record_chassis.vel_obj_dist;
  status_info->adecu_debug_vel_obj_vel = ad_record_chassis.vel_obj_vel;
  status_info->adecu_debug_planning_send_vel = ad_record_chassis.planning_send_vel;
  status_info->adecu_debug_planning_send_ax = ad_record_chassis.planning_send_ax;
  #endif
  return (0);
}

#endif
