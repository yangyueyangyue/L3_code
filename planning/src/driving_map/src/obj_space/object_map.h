/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       object_map.h
 * @brief      内部障碍物
 * @details    定义了内部障碍物的相关数据结构体
 *
 * @author     boc
 * @date       2020.06.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_DRIVING_MAP_OBJECT_MAP_H_
#define PHOENIX_DRIVING_MAP_OBJECT_MAP_H_

#include "utils/macros.h"
#include "geometry/obbox2d.h"
#include "container/static_vector.h"
#include "curve/path.h"
#include "driving_map.h"
#include "ad_msg.h"

namespace phoenix {
namespace driv_map {

/**
 * @struct ObjectRefLineInfo
 * @brief 障碍物与参考线的关联信息
 */
struct ObjectRefLineInfo {
  /// 参考线信息是否有效
  bool valid;
  /// 参考线的索引
  Int32_t ref_line_index;
  /// 障碍物中心坐标在参考线上的投影点
  common::PathPoint proj_on_ref;

  /**
   * @brief 清除内部信息
   */
  void Clear() {
    valid = false;
    ref_line_index = -1;
    proj_on_ref.Clear();
  }

  /**
   * @brief 构造函数
   */
  ObjectRefLineInfo() {
    valid = false;
    ref_line_index = -1;
  }
};

/**
 * @struct Object
 * @brief 障碍物信息(Internal)
 */
struct Object {
  /// 障碍物ID
  Int32_t id;
  /// 障碍物位置
  common::Vec2d pos;
  /// 障碍物包围盒
  common::OBBox2d obb;
  /// Heading
  map_var_t heading;
  /// 障碍物高度
  map_var_t height;
  /// 离地面高度
  map_var_t height_to_ground;
  /// 障碍物类型
  Int8_t type;
  /// 是否是动态障碍物
  Int8_t dynamic;
  /// 障碍物存在的置信度
  Int8_t confidence;
  /// 感知类别
  Int8_t perception_type;
  /// 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
  /// 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
  bool ignore;
  /// 标记不确定的障碍物
  bool uncertain;
  /// 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
  map_var_t v_x;
  /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
  map_var_t v_y;
  /// 障碍物绝对速度，单位：米/秒
  map_var_t v;
  /// 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
  map_var_t a_x;
  /// 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
  map_var_t a_y;
  /// 障碍物绝对加速度，单位：米^2/秒
  map_var_t a;
  /// 角速度
  map_var_t yaw_rate;
  /// 与参考线之间的关联信息
  ObjectRefLineInfo ref_line;
  /// 保存对动态障碍物预测的轨迹
  common::StaticVector<common::StaticVector<common::TrajectoryPoint,
      MAX_OBJ_PRED_TRAJ_POINT_NUM>, MAX_OBJ_PRED_TRAJ_NUM> pred_trajectory;

  /**
   * @brief 清除障碍物预测的轨迹
   */
  void ClearPredTrj() {
    for (Int32_t i = 0; i < pred_trajectory.Size(); ++i) {
      pred_trajectory[i].Clear();
    }
    pred_trajectory.Clear();
  }

  /**
   * @brief 清除内部信息
   */
  void Clear() {
    id = -1;
    pos.Clear();
    obb.Clear();
    heading = 0.0F;
    height = 0.0F;
    height_to_ground = 0.0F;
    type = static_cast<Int8_t>(ad_msg::OBJ_TYPE_UNKNOWN);
    dynamic = 0;
    confidence = 0;
    perception_type = static_cast<Int8_t>(ad_msg::OBJ_PRCP_TYPE_UNKNOWN);
    ignore = false;
    uncertain = false;
    v_x = 0.0F;
    v_y = 0.0F;
    v = 0.0F;
    a_x = 0.0F;
    a_y = 0.0F;
    a = 0.0F;
    yaw_rate = 0.0F;
    ref_line.Clear();
    ClearPredTrj();
  }

  /**
   * @brief 构造函数
   */
  Object() {
    Clear();
  }
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_OBJECT_MAP_H_
