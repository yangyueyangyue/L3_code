/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       object_hierarchical_grids.h
 * @brief      碰撞分析的基类
 * @details    碰撞分析的基类
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_DRIVING_MAP_COLLISION_TESTING_BASE_H_
#define PHOENIX_DRIVING_MAP_COLLISION_TESTING_BASE_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "curve/path.h"
#include "geometry/obbox2d.h"
#include "geometry/geometry_utils.h"
#include "container/static_vector.h"
#include "driving_map.h"
#include "obj_space/object_map.h"


namespace phoenix {
namespace driv_map {


/**
 * @class CollisionTestingBase
 * @brief 碰撞分析的基类
 */
class CollisionTestingBase {
public:
  /**
   * @struct RiskTestResult
   * @brief 碰撞分析的结果信息
   */
  struct RiskTestResult {
    /// 分层网格中的障碍物列表信息obj_list_对应的索引
    Int32_t obj_index;
    /// 此障碍物对应的风险值。静态障碍物根据距离确定，动态障碍物根据距离
    /// 以及S-T的分析结果确定，其中0和100为特殊值，定义0为肯定不会碰撞，
    /// 定义100为肯定会碰撞，其它值为风险值（处于0和100之间）。
    Int32_t risk_value;
    /// 与障碍物之间的距离，单位：米
    map_var_t dynamic_distance;
    map_var_t static_distance;
    /// 与此障碍物有碰撞风险的目标物体在轨迹上的路径长
    map_var_t collision_s;

    /**
     * @brief 比较两个碰撞风险测试结果中的风险大小（重载小于操作符号）
     * @param[in] right 小于操作符右侧的碰撞风险测试结果
     * @return true ~ 操作符左侧风险小，false ~ 操作符右侧的风险小
     */
    bool operator <(const RiskTestResult& right) const {
      if (risk_value < right.risk_value) {
        return true;
      } else if (risk_value == right.risk_value) {
        if (right.dynamic_distance < (dynamic_distance-0.1F)) {
          return true;
        } else if ((dynamic_distance-0.11F) <= right.dynamic_distance &&
                   right.dynamic_distance <= (dynamic_distance+0.11F)) {
          if (right.static_distance < static_distance) {
            return true;
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      obj_index = -1;
      risk_value = 0;
      dynamic_distance = 0;
      static_distance = 0;
      collision_s = 0;
    }

    /**
     * @brief 构造函数
     */
    RiskTestResult() {
      Clear();
    }
  };

  /**
   * @struct ObjectAssociation
   * @brief 用来将分层网格中存储的障碍物信息与上层模块存储的障碍物信息进行关联
   */
  struct ObjectAssociation {
    /// 用来和ObjectMap中的object_list_进行关联
    Int32_t obj_list_index;
    /// 是否是动态的障碍物
    bool dynamic;
    /// 动态障碍物采样点在其预测轨迹中的索引（属于第几条轨迹）
    Int32_t pred_trj_index;
    /// 动态障碍物采样点在其预测轨迹中的索引（属于轨迹中的哪个点）
    Int32_t pred_trj_point_index;
    /// 障碍物采样点信息
    common::TrajectoryPoint trj_point;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      obj_list_index = -1;
      dynamic = false;
      pred_trj_index = -1;
      pred_trj_point_index = -1;
      trj_point.Clear();
    }

    /**
     * @brief 构造函数
     */
    ObjectAssociation() {
      obj_list_index = -1;
      dynamic = false;
      pred_trj_index = -1;
      pred_trj_point_index = -1;
    }
  };


  /*
   * @struct ObjInfoBase
   * @brief 内部使用的障碍物信息
   */
  struct ObjInfoBase {
    // 用来和上层模块的障碍物信息进行关联
    ObjectAssociation obj_association;
    // 障碍物的包围盒
    common::OBBox2d obb;
    common::AABBox2d aabb;
    // 障碍物的包围盒的外接圆的半径
    map_var_t obb_radius;
    // 障碍物在内部列表中的索引
    Int32_t obj_index;

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      obj_association.Clear();
      obb.Clear();
      aabb.Clear();
      obb_radius = 0;
      obj_index = -1;
    }

    /*
     * @brief 构造函数
     */
    ObjInfoBase() {
      Clear();
    }
  };

  class CallbackGetObject {
  public:
    CallbackGetObject() {}
    virtual ~CallbackGetObject() {}
    virtual const Object* Get(Int32_t obj_index) const {
      LOG_WARN << "Unexpected to call this function.";
      return (Nullptr_t);
    }
  };

  class CallbackIfObjHasRisk {
  public:
    CallbackIfObjHasRisk() {}
    virtual ~CallbackIfObjHasRisk() {}
    virtual void Handle(const RiskTestResult& risky_obj) {
      LOG_WARN << "Unexpected to call this function.";
    }
  };

public:
  /**
   * @brief 构造函数
   */
  CollisionTestingBase();

  /**
   * @brief 析构函数
   */
  virtual ~CollisionTestingBase();

  /**
   * @brief 清除内部数据
   */
  virtual void Clear();

  /**
   * @brief 向网格中添加一个障碍物
   * @param[in] new_obj 要添加的新的障碍物的信息
   * @param[in] new_obj_list_index 新的障碍物在用户障碍物列表中的索引
   * @return true 成功 \n
   *         false 失败（通常是内部存储空间不足）
   */
  virtual bool AddObject(
      const Object& new_obj,
      Int32_t new_obj_list_index);

  /**
   * @brief 告知已经完成了所有障碍物的添加，用于碰撞分析模块的进一步处理
   */
  virtual void CompleteAddingObject();

  /**
   * @brief 根据索引返回障碍物关联信息
   * @param[in] obj_index 障碍物索引
   * @return 障碍物关联信息
   */
  virtual const ObjectAssociation* GetObjectAssociation(
      Int32_t obj_index) const;

  /**
   * @brief 测试输入的目标物与网格中的障碍物的碰撞风险
   * @param[in] test_obj 测试的输入目标物
   * @param[out] risky_obj_list 碰撞测试时有风险的障碍物的列表
   * @return 碰撞的风险值
   */
  virtual Int32_t TestCollision(
      const CollisionTestObj& test_obj,
      const CallbackGetObject& callback_get_obj,
      CallbackIfObjHasRisk* callback_if_has_risk) const;

protected:
  /*
   * @brief 测试输入的目标物与某个具体障碍物的碰撞风险
   * @param[in] test_obj 输入目标物
   * @param[in] obj_in_space 障碍物网格空间中的障碍物
   * @param[out] result 碰撞结果信息
   * @return 碰撞的风险值
   */
  Int32_t TestCollisionPairwise(
      const CollisionTestObj& test_obj, const ObjInfoBase& obj_in_space,
      const CallbackGetObject& callback_get_obj,
      RiskTestResult* result) const;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_COLLISION_TESTING_BASE_H_
