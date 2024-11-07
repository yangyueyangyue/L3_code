/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       object_map_impl.h
 * @brief      障碍物空间
 * @details    构建障碍物空间，保存了障碍物相关信息，提供碰撞分析的功能
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

#ifndef PHOENIX_DRIVING_MAP_OBJECT_MAP_IMPL_H_
#define PHOENIX_DRIVING_MAP_OBJECT_MAP_IMPL_H_

#include "container/static_vector.h"
#include "obj_space/object_map.h"
#include "obj_space/collision_testing_hierarchical_grids.h"
#include "driving_map.h"


namespace phoenix {
namespace driv_map {


/**
 * @class ObjectMapImpl
 * @brief 构建障碍物空间
 */
class ObjectMapImpl {
public:
  /**
   * @brief 构造函数
   */
  ObjectMapImpl();

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 获取障碍物列表
   * @return true 障碍物列表
   */
  inline const common::StaticVector<Object, MAX_OBJECT_NUM_IN_OBJ_SPACE>&
      object_list() const {
    return (object_list_);
  }

  /**
   * @brief 添加一个障碍物
   * @param[in] new_obj 要添加的新的障碍物的信息
   * @param[in] new_obj_list_index 新的障碍物在用户障碍物列表中的索引
   * @return true 成功 \n
   *         false 失败（通常是内部存储空间不足）
   */
  bool AddObject(const Object& new_obj);

  /**
   * @brief 告知已经完成了所有障碍物的添加，用于碰撞分析模块的进一步处理
   */
  void CompleteAddingObject() {
    object_space_.CompleteAddingObject();
  }

  /**
   * @brief 通过索引获取障碍物信息
   * @param[in] obj_index 障碍物索引
   * @return 障碍物信息
   */
  const Object& GetObject(Int32_t obj_index) const;

  bool IsUncertainObject(Int32_t obj_index) const;
  bool IsUncertainObject(const Object& obj) const;

  /**
   * @brief 测试输入的目标物与网格中的障碍物的碰撞风险
   * @param[in] test_obj 输入目标物
   * @param[out] risky_obj_list 碰撞测试时有风险的障碍物的列表
   * @return 碰撞的风险值
   */
  Int32_t TestCollision(const CollisionTestParam& param,
      const CollisionTestObj& test_obj,
      CollisionTestResult* const result) const;

  Int32_t TestCollisionOnPath(
      const CollisionTestOnPathObj& obj_in,
      const common::Path& path,
      common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> path_sample_points,
      CollisionTestOnPathResult* const result) const;

  /**
   * @brief 忽略正后方的动态障碍物
   * @param[in] test_obj 输入目标物
   */
  void IgnoreRightBehindDynamicObject(const CollisionTestObj& test_obj);

private:
  // 将类ObjectMapImpl设置为类CallbackGetObject的友元
  class CallbackGetObject;
  friend class CallbackGetObject;
  class CallbackGetObject : public CollisionTestingBase::CallbackGetObject {
  public:
    CallbackGetObject(const ObjectMapImpl* obj_map) { obj_map_ = obj_map; }
    virtual const Object* Get(Int32_t obj_index) const {
      if ((0 <= obj_index) && (obj_index < obj_map_->object_list_.Size())) {
        return (&(obj_map_->object_list_[obj_index]));
      }
      return (Nullptr_t);
    }

  private:
    // 构建障碍物空间类实例的指针
    const ObjectMapImpl* obj_map_;
  };

  // 将类ObjectMapImpl设置为类CallbackAddRiskyObject的友元
  class CallbackAddRiskyObject;
  friend class CallbackAddRiskyObject;
  /*
   * @class CallbackAddRiskyObject
   * @brief 添加有风险的障碍物时的回调类
   */
  class CallbackAddRiskyObject :
      public CollisionTestingBase::CallbackIfObjHasRisk {
  public:
    /*
     * @brief 构造函数
     * @param[in] obj_map 构建障碍物空间类实例的指针
     * @param[in] risky_static_obj_list 有风险的静态障碍物列表
     * @param[in] risky_dynamic_obj_list 有风险的动态障碍物列表
     */
    CallbackAddRiskyObject(
        const ObjectMapImpl* obj_map,
        const CollisionTestParam* prm,
        common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM> * const risky_static_obj_list,
        common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM> * const risky_dynamic_obj_list,
        common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM> * const uncertain_obj_list) :
      obj_map_(obj_map),
      param_(prm),
      risky_static_obj_list_(risky_static_obj_list),
      risky_dynamic_obj_list_(risky_dynamic_obj_list),
      uncertain_obj_list_(uncertain_obj_list) {
    }

    /*
     * @brief 对可能有风险的障碍物进行处理
     * @param[in] risky_obj 有风险的障碍物
     * @note 1、当有风险的障碍物列表未满时，将risky_obj添加
     *          到有风险的障碍物列表中；
     *       2、当有风险的障碍物列表已满，且risky_obj的碰撞
     *          风险比有风险的障碍物列表中碰撞风险最小的障碍物
     *          的碰撞风险大时，将有风险的障碍物列表中碰撞最小
     *          的障碍物替换为risky_obj。
     */
    virtual void Handle(const CollisionTestingBase::RiskTestResult& risky_obj);

  private:
    // 构建障碍物空间类实例的指针
    const ObjectMapImpl* obj_map_;
    const CollisionTestParam* param_;
    // 有风险的静态障碍物列表
    common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM>* risky_static_obj_list_;
    // 有风险的动态障碍物列表
    common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM>* risky_dynamic_obj_list_;
    // 不确定的且有风险的障碍物列表
    common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM>* uncertain_obj_list_;
  };

  // 将类ObjectMapImpl设置为类CallbackIgnoreRightBehindDynamicObject的友元
  class CallbackIgnoreRightBehindDynamicObject;
  friend class CallbackIgnoreRightBehindDynamicObject;
  /*
   * @class CallbackIgnoreRightBehindDynamicObject
   * @brief 忽略正后方的动态障碍物时的回调类
   */
  class CallbackIgnoreRightBehindDynamicObject :
      public CollisionTestingBase::CallbackIfObjHasRisk {
  public:
    /*
     * @brief 构造函数
     * @param[in] test_obj 自动驾驶车辆的有向矩形包围盒、速度等信息
     * @param[in] obj_map 构建障碍物空间类实例的指针
     */
    CallbackIgnoreRightBehindDynamicObject(
        const CollisionTestObj& test_obj, ObjectMapImpl * const obj_map) :
      /*test_obj_(test_obj), */obj_map_(obj_map) {
      test_obj_heading_ = test_obj.obb.unit_direction_x().Angle();
    }

    /*
     * @brief 将自动驾驶车辆正后方的障碍物设置为可忽略障碍物
     * @param[in] risky_obj 有风险的障碍物
     */
    virtual void Handle(const CollisionTestingBase::RiskTestResult& risky_obj);

  private:
    // 自动驾驶车辆的有向矩形包围盒、速度等信息
    // const CollisionRiskTestObj& test_obj_;
    // 自动驾驶车辆的航向角
    map_var_t test_obj_heading_;
    // 构建障碍物空间类实例的指针
    ObjectMapImpl* obj_map_;
  };

private:
  void AddCollisionInfoToList(
      const CollisionTestResult::ObjInfo& coll_info,
      common::StaticVector<CollisionTestOnPathResult::ObjInfo,
      CollisionTestOnPathResult::MAX_RISKY_OBJ_NUM>* risky_obj_list,
      common::UnorderedMap<Int32_t, Int32_t,
      CollisionTestOnPathResult::MAX_RISKY_OBJ_NUM>* risky_obj_lookup_tab) const;

  void ClassifyCollisionInfo(
      const CollisionTestOnPathObj& obj_in,
      const common::Path& path,
      const common::PathPoint& start_point,
      CollisionTestOnPathResult::ObjInfo* coll_info) const;

private:
  // 保存输入的障碍物列表
  common::StaticVector<Object, MAX_OBJECT_NUM_IN_OBJ_SPACE> object_list_;
  // 构建障碍物空间
  CollisionTestingHierarchicalGrids object_space_;
  // CollisionTestingKdTree object_space_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_OBJECT_MAP_IMPL_H_
