/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control_wrapper.c
 * @brief      横向控制器接口
 * @details    定义横向控制器接口
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
#include "lateral_control_wrapper_c.h"

#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
#include "LATCtrl.h"
#include "msg_chassis_c.h"
#elif (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
#include "lateral_control_pid_c.h"
#else
  Error: Invalid Lateral Controller Type.
#endif

/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 横向控制器的成员变量
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
static LatCtlPidInstance_t s_lat_ctl_pid_instance;
#endif

/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化横向控制器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtl_Initialize() {
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    // 横向控制初始化    matlab代码生成   2022/7/20   wm
    /* Initialize model */
  LATCtrl_initialize();
  
  //模型运行相关的状态参数设置
  LATC_swtADMod_C = 1;    // 自动驾驶进入选项
  LATC_stADMod_C = 0;
  LKA_swtAgSteerOfstSel_C = 1;
  LATC_agSteerOfst_C = 0;  //34#零位补偿5度
  //LATC_agSteerZeroOfst ;
  LATC_swtLQRMan_C = 1;
  LQR_Calc_flag = 1;          //LQR运算方式：0在线计算，1离线插值表
  if (LQR_Calc_flag==1) {
    LQR_CircleTime_C = 0;
  } else {
    LQR_CircleTime_C = 700;
  }

  // 空载
  LATC_Q1Wgt4LQR_C = 0.7;//1;
  LATC_Q2Wgt4LQR_C = 1;
  LATC_Q3Wgt4LQR_C = 0.008;
  LATC_Q4Wgt4LQR_C = 5;//8;
  //满载
  LATC_Q1Wgt4LQR_C = 0.6;//1;
  LATC_Q2Wgt4LQR_C = 1;
  LATC_Q3Wgt4LQR_C = 0.005;
  LATC_Q4Wgt4LQR_C = 0.3;//8;
  LATC_para4LQR_R_C = 200;

  LATC_swtPathSel_C = 1;
  LATC_facLaneSign_C = -1;
  LATC_facMPUSign_C = 1;

  //增加换道预瞄点的设置
  Prev_strategy_switch = 1;   //换道预瞄点是否要调整：0 默认；1调整
  LATC_tPrevtime4FB_C =2; //换道反馈预瞄点时长（s）默认2s
  LATC_tPrevtime4FF_C = 2;//换道前馈预瞄点时长（s）默认0.8s
  LC_PrevDistLimitFB =1;// 2;//换道反馈预瞄点距离限制（m）默认20m
  LC_PrevDistLimitFF = 1;//3;//10;//换道前馈预瞄点距离限制（m）默认5m  20m可能会超出换道轨迹出现意外
  LATC_tPrevtime4LQR_C = 0.3;//调整LKA预瞄点
  LCC_lPrevDistLimit = 5.0;//3.0;//调整LKA预瞄点距离限幅（m）默认2m
  LatHeadRateLmt_rad = 0.03;
  LatDistRateLmt = 0.3;//0.6;
  LatCvRateLmt = 0.003;//0.01
  DistErrRateLimit = 3.0;  //误差速度变化率
  //误差设置
  //LQR_Ele1Man_swt = 0;//0：默认选择计算值；1：手动设置值
  //LQR_Ele1Man_C = 0;//手动设置的err1值
#endif

#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
  Phoenix_LatCtlPid_Initialize(&s_lat_ctl_pid_instance);
#endif
}

/*
 * @brief 重置横向控制器内部状态
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtl_ResetLatCtlValue(Float32_t tar_steering_angle) {
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
  Phoenix_LatCtlPid_ResetLatCtlValue(&s_lat_ctl_pid_instance, tar_steering_angle);
#endif
}

/*
 * @brief 配置横向控制器(PID)的参数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtl_ConfigPIDController(const LatCtlPidConf_t* conf) {
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
  Phoenix_LatCtlPid_Configurate(&s_lat_ctl_pid_instance, conf);
#endif
}

/*
 * @brief 计算横向控制量using PID
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
Int32_t Phoenix_LatCtl_CalcLatCtlValue(
    const LatCtlDataSource_t* data_source,
    Float32_t* tar_yaw_rate, Float32_t* ctl_value,
    LateralControlInfo_t* const status_info) {
  return (Phoenix_LatCtlPid_CalcLatCtlValue(
            &s_lat_ctl_pid_instance,
            data_source, tar_yaw_rate, ctl_value,
            &(status_info->lat_ctl_pid_info)));
}
#endif

/*
 * @brief 计算横向控制量 using LQR
 *
 */
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
Int32_t DFCV_LatCtl_CalcLatCtlValue(Chassis_t *dfcvctrl_src_chassis, 
    DFCV_SpecialChassisInfo_t* dfcv_special_chassis,
    PlanningResult_t *planning_result,    Imu_t  *imu, Int32_t *CtrlStatus, 
    Float32_t* tar_yaw_rate, Float32_t* ctl_value,    LateralControlInfo_t* const status_info) {
//系统消息
 //  if (Phoenix_AdMsg_IsChassisInRoboticMode(*CtrlStatus)) {
if (*CtrlStatus == VEH_CHASSIS_CTL_STATUS_ROBOTIC) {
  CoST_stADMoEna = 3;            // 驾驶模式
} else {
  CoST_stADMoEna = 0 ;            // 驾驶模式 
}

if (VMP_LC_Flag>0){
  LQR_Ele3Man_swt = 1;
  LQR_Ele4Man_swt = 1;
} else {
  LQR_Ele3Man_swt = 0;
  LQR_Ele4Man_swt = 0;
}

  COEHPS_LKADemdRsp = 1;      //转向机构控制请求   （此处是否已转移到系统控制）
  VMP_LC_Flag =  planning_result->tar_trj.lat_err.moving_flag;              // 换道标志位
  ADCU_tTiDely4MCU2RCAR = 0;                                 //规控输入 ？？？
  VMP_lLatPosn4TgtPath  = 0;                                       //规控输入 ？？？
  VMP_agHeadAngle4TgtPath  = 0;                             //规控输入 ？？？
  VMP_cCv4TgtPath  = 0;                                                 //规控输入 ？？？
  // 目前测试换道轨迹切换在控制部分，后续应该全部在规控部分
  VMP_cTgtPathCv0 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[0];        
  VMP_cTgtPathCv1 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[1];        
  VMP_cTgtPathCv2 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[2];           
  VMP_cTgtPathCv3 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[3];        
  VMP_cTgtPathCv4 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[4];         
  VMP_cTgtPathCv5 = planning_result->tar_trj.PlanningTraj_Cv.coeffs[5];          
  VMP_lLgtDist2PathZero = 0;      //规控输入 ？？？

  //左右车道线系数输入
  VMP_stLane0Prob = 0;      //左车道线置信度
  VMP_lLaneDist0 = 0;          //左车道线拟合零次项系数
  VMP_Lane0Cv1 = 0;           //左车道线拟合一次项系数
  VMP_Lane0Cv2 = 0;           //左车道线拟合二次项系数
  VMP_Lane0Cv3 = 0;           //左车道线拟合三次项系数
  VMP_stLane1Prob = 0;     //右车道线置信度
  VMP_lLaneDist1 = 0;        //右车道线拟合零次项系数
  VMP_Lane1Cv1 = 0;         //右车道线拟合一次项系数
  VMP_Lane1Cv2 = 0;         //右车道线拟合二次项系数
  VMP_Lane1Cv3 = 0;         //右车道线拟合三次项系数
  //车体信息
  DE_mVehMass = dfcv_special_chassis->vehicle_mass;                  //车辆质量
  DE_stTrlr = dfcv_special_chassis->trailer_connected_status;     //挂车状态位
  DE_wVehYawRate =  dfcvctrl_src_chassis->yaw_rate;                  //车辆航向角速度
  DE_vVehSpdKph =  dfcvctrl_src_chassis->v * 3.6;                            //车辆速度
  DE_phiSteerAngle  =  dfcvctrl_src_chassis->steering_wheel_angle * 57.29578;  // 车体方向盘转角;        /* '<Root>/DE_phiSteerAngle' */
  DE_aLonAcc =  dfcvctrl_src_chassis->ax;                              // IMU加计信息    imu.accel_x
  DE_aLatAcc  =  dfcvctrl_src_chassis->ay;                              // IMU加计信息;   imu.accel_y
  DE_aVertAcc =  imu->accel_z;                         // IMU加计信息;   在哪里取？？？

  // 横向控制执行函数    matlab代码生成   2022/7/20   wm
  /* Step the model */
  LATCtrl_step();

  //换道切换时，清零控制量
  if( (planning_result->tar_trj.lat_err.moving_flag) < VMP_LC_Flag ) {
    VMC_phiTarAgSteer = 0;
  }

  printf("ADmode = %d \n",*CtrlStatus);
  status_info->lat_debug.ADmode = *CtrlStatus;
  status_info->lat_debug.CurSteerAngle = DE_phiSteerAngle;
  status_info->lat_debug.CurVehYawRate = DE_wVehYawRate;
  status_info->lat_debug.PrevLatDist = LCC_lPrevLatDist4LQR;
  status_info->lat_debug.PrevHeading = LCC_lPrevHeading4LQR;
  status_info->lat_debug.PrevCurve = LCC_lPrevCv4LQR;
  //status_info->lat_debug.PrevLatDist_Path =  0;//LCC_lPrevLatDist4LQR_Path;
  //status_info->lat_debug.PrevHeading_Path = 0;// LCC_lPrevHeading4LQR_Path;
  //status_info->lat_debug.PrevCurve_Path =  0;//LCC_lPrevCv4LQR_Path;

  status_info->lat_debug.CalculateLatDist = LATC_lDist4Cmr;
  status_info->lat_debug.CalculateLatHeading = LATC_agHead4Cmr;
  status_info->lat_debug.CalculateLatCurve = LATC_cCv4Cmr;
  status_info->lat_debug.ChangeLaneFlag = VMP_LC_Flag;
  status_info->lat_debug.TarAngleSteer = VMC_phiTarAgSteer;
  status_info->lat_debug.LATC_eleK1 = LATC_eleK1;
  status_info->lat_debug.LATC_eleK2 = LATC_eleK2;
  status_info->lat_debug.LATC_eleK3 = LATC_eleK3;
  status_info->lat_debug.LATC_eleK4 = LATC_eleK4;
  status_info->lat_debug.LATC_agFrTyreFBEle1 = LATC_agFrTyreFBEle1;
  status_info->lat_debug.LATC_agFrTyreFBEle2 = LATC_agFrTyreFBEle2;
  status_info->lat_debug.LATC_agFrTyreFBEle3 = LATC_agFrTyreFBEle3;
  status_info->lat_debug.LATC_agFrTyreFBEle4 = LATC_agFrTyreFBEle4;
  status_info->lat_debug.LATC_agFFSteerDeg = LATC_agFFSteerDeg ;
  status_info->lat_debug.LATC_agFBSteerDeg = LATC_agFBSteerDeg;

    // 状态信息
  status_info->lat_debug.CoST_stADMoEna = CoST_stADMoEna;
  status_info->lat_debug.LATC_swtPathSel_C = LATC_swtPathSel_C;
  status_info->lat_debug.LATC_Q1Wgt4LQR_C = LATC_Q1Wgt4LQR_C;
  status_info->lat_debug.LATC_Q2Wgt4LQR_C = LATC_Q2Wgt4LQR_C;
  status_info->lat_debug.LATC_Q3Wgt4LQR_C = LATC_Q3Wgt4LQR_C;
  status_info->lat_debug.LATC_Q4Wgt4LQR_C = LATC_Q4Wgt4LQR_C;
  status_info->lat_debug.LKA_swtAgSteerOfstSel_C = LKA_swtAgSteerOfstSel_C;
  status_info->lat_debug.LATC_agSteerOfst_C = LATC_agSteerOfst_C;
  status_info->lat_debug.LATC_agSteerZeroOfst = LATC_agSteerZeroOfst;
  status_info->lat_debug.LQR_IntNum = LQR_IntNum;
  status_info->lat_debug.LC_lPrevCv4FF = LC_lPrevCv4FF;
  status_info->lat_debug.LC_lPrevHeading4FB = LC_lPrevHeading4FB;
  status_info->lat_debug.LC_lPrevLatDist4FB = LC_lPrevLatDist4FB;
  status_info->lat_debug.LatCv4LQR = LatCv4LQR;
  status_info->lat_debug.LatDist4LQR = LatDist4LQR;
  status_info->lat_debug.LatHeading4LQR = LatHeading4LQR;
  
  status_info->lat_debug.DE_mVehMass = DE_mVehMass;
  status_info->lat_debug.DE_stTrlr = DE_stTrlr;
  status_info->lat_debug.LATC_vLatSpdFlt4LQR = LATC_vLatSpdFlt4LQR;
  status_info->lat_debug.LATC_vDistErrFlt4LQR = LATC_vDistErrFlt4LQR;
  status_info->lat_debug.LATC_agHeadFlt4LQR = LATC_agHeadFlt4LQR;
  status_info->lat_debug.LATC_wYRErrFlt4LQR = LATC_wYRErrFlt4LQR;
  
  *tar_yaw_rate = 0;
  *ctl_value = VMC_phiTarAgSteer * 0.01745329251994329577;
  
  return (0);
}
#endif
