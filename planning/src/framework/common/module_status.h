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


enum {
  /// 正常
  PLANNING_MODULE_STATUS_OK = 0,
  /// 异常
  PLANNING_MODULE_STATUS_ERR,
  /// 位置滤波异常
  PLANNING_MODULE_STATUS_ERR_POS_FILTER_FAULT,
  /// 驾驶地图异常
  PLANNING_MODULE_STATUS_ERR_DRIVING_MAP_FAULT,
  /// OBJ滤波异常
  PLANNING_MODULE_STATUS_ERR_OBJ_FILTER_FAULT,
  /// 行为规划异常
  PLANNING_MODULE_STATUS_ERR_ACTION_PLANNING_FAULT,
  /// 轨迹规划异常
  PLANNING_MODULE_STATUS_ERR_TRAJECTORY_PLANNING_FAULT,
  /// 速度规划异常
  PLANNING_MODULE_STATUS_ERR_VELOCITY_PLANNING_FAULT
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MODULE_STATUS_H_

