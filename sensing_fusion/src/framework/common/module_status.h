/******************************************************************************
 ** 模块状态类型定义
 ******************************************************************************
 *
 *  定义各种模块状态
 *
 *  @file       mudule_status.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MODULE_STATUS_H_
#define PHOENIX_FRAMEWORK_MODULE_STATUS_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "container/static_vector.h"


namespace phoenix {
namespace framework {


enum InternalModuleId {
  INTERNAL_MODULE_ID_INVALID = 0,

  /// Module status
  INTERNAL_MODULE_ID_MSG_RECV_GNSS,
  INTERNAL_MODULE_ID_MSG_RECV_IMU,
  INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE,
  INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE_CURB,
  INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_OBJ,
  INTERNAL_MODULE_ID_MSG_RECV_ESR_FRONT,
  INTERNAL_MODULE_ID_MSG_RECV_CHASSIS,
  INTERNAL_MODULE_ID_MSG_RECV_MAXIEYE_CAMERA,
  INTERNAL_MODULE_ID_MSG_RECV_LIDAR_FRONT,
  INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_LEFT,
  INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_RIGHT,
  INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_LEFT,
  INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_RIGHT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_LEFT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_RIGHT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_LEFT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_RIGHT,
  INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR,
  INTERNAL_MODULE_ID_MSG_RECV_FUSION_OBJ,
  INTERNAL_MODULE_ID_MSG_RECV_MPU_STATE,

  INTERNAL_MAX_MODULE_NUM
};

enum {
  INTERNAL_MODULE_STATUS_OK = 0,
  INTERNAL_MODULE_STATUS_ERR,
  INTERNAL_MODULE_STATUS_ERR_POS_FILTER_FAULT,
  INTERNAL_MODULE_STATUS_ERR_DRIVING_MAP_FAULT,
  INTERNAL_MODULE_STATUS_ERR_OBJ_FILTER_FAULT,
  INTERNAL_MODULE_STATUS_ERR_ACTION_PLANNING_FAULT,
  INTERNAL_MODULE_STATUS_ERR_TRAJECTORY_PLANNING_FAULT,
  INTERNAL_MODULE_STATUS_ERR_VELOCITY_PLANNING_FAULT
};



/**
 * @brief 传感器数据状态
 * 
 */
enum {
  PERCEPTION_MODULE_DATA_STATUS_OK = 0, //传感器数据正常
  PERCEPTION_MODULE_DATA_BEATING_VIOLENTLY = 1, // 传感器数据跳动剧烈
  PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE = 1<<1, // 超过正常位置范围
  PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE = 1<<2, // 超过正常速度范围
  PERCEPTION_MODULE_DATA_LIST_EMPTY = 1<<3, //  传感器数据列表为空 
  // PERCEPTION_MODULE_DATA_ERR,

};


/**
 * @brief 以下是关于感知模块状态定义
 * 
 */
enum {
  /// 正常
  PERCEPTION_MODULE_STATUS_OK = 0,
  /// 异常
  PERCEPTION_MODULE_STATUS_ERR,
  /// 位置滤波异常
  PERCEPTION_MODULE_STATUS_ERR_POS_FILTER_FAULT,
  /// 驾驶地图异常
  PERCEPTION_MODULE_STATUS_ERR_DRIVING_MAP_FAULT,

  ///更新之前跟踪的障碍物出现错误
  PERCEPTION_MODULE_STATUS_PREPROCESS_TRACKED_OBJS_FAULT,

  /// 从相对位置列表中获取当前位置出现错误
  PERCEPTION_MODULE_STATUS_ERR_GET_CUR_POS_FAULT,

  /// 相对位置列表信息为空
  PERCEPTION_MODULE_STATUS_ERR_EMPTY_REL_POS_LIST_FAULT,

  /// 传感器都没有有效障碍物输入
  PERCEPTION_MODULE_STATUS_ERR_NO_SENSOR_OBJ_INPUT_FAULT,

  /// 传感器有输入却没有融合结果
  PERCEPTION_MODULE_STATUS_ERR_NO_PERCEPTION_OBJ_RESULT_FAULT,

  /// 感知1V1R都有输入但是无多传感器融合结果,该设置有待商榷
  PERCEPTION_MODULE_STATUS_ERR_NO_1V1R_PERCEPTION_FAULT

/**
 * @brief 
 * 其他可能状态：
 * 1. 传感器障碍物方向、位置、速度、明显异常，比如速度过大，方向反向，位置超限制；
 * 2. 传感器同一ID障碍物跳动过于剧烈；
 * 3. 输出融合结果与相应的传感器结果差别过于剧烈，比如方向相反，速度差别太大。
 * 
 */

};

struct PerceptionModuleStatus {
  Int64_t timestamp;          //时间戳
  Int32_t module_status;      //感知模块整体状态，一般是模块的返回值
  Uint32_t perception_data_status; //融合结果的数据状态, 保留字段，先不处理
  Uint32_t fw_radar_data_status; //前毫米波雷达数据状态
  Uint32_t main_camera_data_status; //前视一体机数据状态

  Uint32_t fw_lidar_data_status; //激光雷达数据状态

  Uint32_t left_fw_radar_data_status; //左前毫米波雷达数据状态
  Uint32_t right_fw_radar_data_status; //右前毫米波雷达数据状态
  Uint32_t left_bw_radar_data_status; //左后毫米波雷达数据状态
  Uint32_t right_bw_radar_data_status; //右后毫米波雷达数据状态

  Uint32_t left_side_camera_data_status;//左前相机数据状态
  Uint32_t right_side_camera_data_status;//右前相机数据状态
  Uint32_t left_bw_camera_data_status;//左后相机数据状态
  Uint32_t right_bw_camera_data_status;//右后相机数据状态
  Uint32_t front_center_camera_data_status;//前环视数据状态

  Uint32_t reserved0;//保留字段
  Uint32_t reserved1;//保留字段
  Uint32_t reserved2;//保留字段
  Uint32_t reserved3;//保留字段

  void Clear(){
    timestamp = 0;
    module_status = 0;
    perception_data_status = 0;
    fw_radar_data_status = 0;
    main_camera_data_status = 0;
    fw_lidar_data_status = 0;

    left_fw_radar_data_status = 0;
    right_fw_radar_data_status = 0;
    left_bw_radar_data_status = 0;
    right_bw_radar_data_status = 0;

    left_side_camera_data_status = 0;
    right_side_camera_data_status = 0;
    left_bw_camera_data_status = 0;
    right_bw_camera_data_status = 0;

    reserved0 = 0;
    reserved1 = 0;
    reserved2 = 0;
    reserved3 = 0;

  }


};





}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MODULE_STATUS_H_

