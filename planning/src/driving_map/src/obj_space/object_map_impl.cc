/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       object_map_impl.cc
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

#include "obj_space/object_map_impl.h"
#include "geometry/geometry_utils.h"

namespace phoenix {
namespace driv_map {


ObjectMapImpl::ObjectMapImpl() {
}

void ObjectMapImpl::Clear() {
  object_list_.Clear();
  object_space_.Clear();
}

bool ObjectMapImpl::AddObject(const Object& new_obj) {
  // Allocate a space from object list.
  Int32_t obj_index = -1;
  Object* obj = object_list_.Allocate(&obj_index);
  if (Nullptr_t == obj) {
    LOG_ERR << "Storage is full and cann't save objects anymore.";
    return false;
  }

  // Save object information to object list.
  *obj = new_obj;
  obj->uncertain = IsUncertainObject(*obj);

  // Add object to object space.
  if (!object_space_.AddObject(new_obj, obj_index)) {
    // Failed to add object to space.
    object_list_.PopBack();
    LOG_ERR << "Cann't add object to object space.";
    return false;
  }

  return (true);
}

const Object& ObjectMapImpl::GetObject(Int32_t obj_index) const {
  COM_CHECK((obj_index >= 0) && (obj_index < object_list_.Size()));

  return (object_list_[obj_index]);
}

bool ObjectMapImpl::IsUncertainObject(Int32_t obj_index) const {
  if (obj_index < 0 || obj_index >= object_list_.Size()) {
    return false;
  }

  return (IsUncertainObject(object_list_[obj_index]));
}

bool ObjectMapImpl::IsUncertainObject(const Object& obj) const {
  if (obj.perception_type == ad_msg::OBJ_PRCP_TYPE_UNKNOWN) {
    return true;
  }
  // if ((obj.perception_type == ad_msg::OBJ_PRCP_TYPE_RADAR) && (!obj.dynamic)) {
  //   return true;
  // }
  if (obj.confidence < 60) {
    return true;
  }

  return false;
}

Int32_t ObjectMapImpl::TestCollision(
    const CollisionTestParam& param,
    const CollisionTestObj& test_obj,
    CollisionTestResult* const result) const {
  result->Clear();

  Int32_t risk_value = 0;
  common::StaticVector<CollisionTestingBase::RiskTestResult,
      CollisionTestResult::MAX_RISKY_OBJ_NUM> risky_static_obj_list;
  common::StaticVector<CollisionTestingBase::RiskTestResult,
      CollisionTestResult::MAX_RISKY_OBJ_NUM> risky_dynamic_obj_list;
  common::StaticVector<CollisionTestingBase::RiskTestResult,
      CollisionTestResult::MAX_RISKY_OBJ_NUM> uncertain_obj_list;

  CallbackAddRiskyObject callback_add_risky_obj(
        this, &param,
        &risky_static_obj_list, &risky_dynamic_obj_list,
        &uncertain_obj_list);
  CallbackGetObject callback_get_obj(this);
  risk_value = object_space_.TestCollision(
        test_obj, callback_get_obj, &callback_add_risky_obj);

  for (Int32_t i = 0; i < risky_static_obj_list.Size(); ++i) {
    CollisionTestResult::ObjInfo* risky_obj_out = result->risky_obj_list.Allocate();
    if (Nullptr_t != risky_obj_out) {
      const CollisionTestingBase::RiskTestResult& risky_info =
          risky_static_obj_list[i];
      const CollisionTestingBase::ObjectAssociation* obj_ass =
          object_space_.GetObjectAssociation(risky_info.obj_index);
      const Object& obj = object_list_[obj_ass->obj_list_index];
      risky_obj_out->obj_list_index = obj_ass->obj_list_index;
      risky_obj_out->risk_value = risky_info.risk_value;
      risky_obj_out->dynamic_distance = risky_info.dynamic_distance;
      risky_obj_out->static_distance = risky_info.static_distance;
      risky_obj_out->obj_s_ref = obj.ref_line.proj_on_ref.s;
      risky_obj_out->obj_l_ref = obj.ref_line.proj_on_ref.l;
      risky_obj_out->collision_s = risky_info.collision_s;
      risky_obj_out->obj_traj_point = obj_ass->trj_point;
    } else {
      break;
    }
  }

  for (Int32_t i = 0; i < risky_dynamic_obj_list.Size(); ++i) {
    CollisionTestResult::ObjInfo* risky_obj_out = result->risky_obj_list.Allocate();
    if (Nullptr_t != risky_obj_out) {
      const CollisionTestingBase::RiskTestResult& risky_info =
          risky_dynamic_obj_list[i];
      const CollisionTestingBase::ObjectAssociation* obj_ass =
          object_space_.GetObjectAssociation(risky_info.obj_index);
      const Object& obj = object_list_[obj_ass->obj_list_index];
      risky_obj_out->obj_list_index = obj_ass->obj_list_index;
      risky_obj_out->risk_value = risky_info.risk_value;
      risky_obj_out->dynamic_distance = risky_info.dynamic_distance;
      risky_obj_out->static_distance = risky_info.static_distance;
      risky_obj_out->obj_s_ref = obj.ref_line.proj_on_ref.s;
      risky_obj_out->obj_l_ref = obj.ref_line.proj_on_ref.l;
      risky_obj_out->collision_s = risky_info.collision_s;
      risky_obj_out->obj_traj_point = obj_ass->trj_point;
    } else {
      break;
    }
  }

  for (Int32_t i = 0; i < uncertain_obj_list.Size(); ++i) {
    CollisionTestResult::ObjInfo* risky_obj_out = result->uncertain_list.Allocate();
    if (Nullptr_t != risky_obj_out) {
      const CollisionTestingBase::RiskTestResult& risky_info =
          uncertain_obj_list[i];
      const CollisionTestingBase::ObjectAssociation* obj_ass =
          object_space_.GetObjectAssociation(risky_info.obj_index);
      const Object& obj = object_list_[obj_ass->obj_list_index];
      risky_obj_out->obj_list_index = obj_ass->obj_list_index;
      risky_obj_out->risk_value = risky_info.risk_value;
      risky_obj_out->dynamic_distance = risky_info.dynamic_distance;
      risky_obj_out->static_distance = risky_info.static_distance;
      risky_obj_out->obj_s_ref = obj.ref_line.proj_on_ref.s;
      risky_obj_out->obj_l_ref = obj.ref_line.proj_on_ref.l;
      risky_obj_out->collision_s = risky_info.collision_s;
      risky_obj_out->obj_traj_point = obj_ass->trj_point;
    } else {
      break;
    }
  }

  return (risk_value);
}

#define ENABLE_OUTPUT_TEST_COLLISION_LOG (0)

Int32_t ObjectMapImpl::TestCollisionOnPath(
    const CollisionTestOnPathObj& obj_in,
    const common::Path& path,
    common::StaticVector<common::PathPoint,
    common::Path::kMaxPathPointNum> path_sample_points,
    CollisionTestOnPathResult* const result) const {
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
  std::cout << "\n######  Test Collision on path. (Begin)  ######" << std::endl;
#endif

  result->Clear();
  Int32_t sample_size = path_sample_points.Size();
  if (sample_size < 1) {
    LOG_ERR << "Failed to sample path for collision testing.";
    return (0);
  }

  const common::PathPoint& start_point = path_sample_points.Front();

  CollisionTestObj test_obj;
  test_obj.obb.set_extents(obj_in.obj_half_length, obj_in.obj_half_width);
  test_obj.obb_circumradius = common::com_sqrt(
        common::Square(obj_in.obj_half_length) +
        common::Square(obj_in.obj_half_width));
  test_obj.v = obj_in.obj_v;
  if (test_obj.v < 0.1F) {
    test_obj.v = 0.1F;
  }

  CollisionTestResult coll_test_ret;

  for (Int32_t i = 0; i < sample_size; ++i) {
    const common::PathPoint& sample_point = path_sample_points[i];
    map_var_t heading = sample_point.heading;
    if (obj_in.is_backing) {
      // For backing mode of vehicle
      heading = common::NormalizeAngle(heading + COM_PI);
    }
    test_obj.obb.set_unit_direction(
          common::com_cos(heading), common::com_sin(heading));
    test_obj.obb.set_center(
          sample_point.point +
          obj_in.x_offset * test_obj.obb.unit_direction_x());
    test_obj.s = common::com_abs(sample_point.s - start_point.s) +
        obj_in.s_offset;
    test_obj.t = test_obj.s / test_obj.v + obj_in.t_offset;

    coll_test_ret.Clear();
    CollisionTestParam param;
    param.SetTestingIgnoredObj(true);
    param.SetReturnSingleRiskyObj(false);
    Int32_t risk_value = TestCollision(
          param, test_obj, &coll_test_ret);
    Int32_t risky_obj_list_size = coll_test_ret.risky_obj_list.Size();

#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
    std::cout << "\n>>> 采样点:" << i << ", s=" << test_obj.s << ", t=" << test_obj.t << std::endl;
    std::cout << "  碰撞风险值:" << risk_value << std::endl;
    std::cout << "  有风险的障碍物数量:" << risky_obj_list_size << std::endl;
#endif

    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      AddCollisionInfoToList(
            coll_test_ret.risky_obj_list[j],
            &(result->risky_obj_list), &(result->risky_obj_lookup_tab));
    }

    Int32_t uncertain_list_size = coll_test_ret.uncertain_list.Size();
    for (Int32_t j = 0; j < uncertain_list_size; ++j) {
      AddCollisionInfoToList(
            coll_test_ret.uncertain_list[j],
            &(result->uncertain_list), &(result->uncertain_lookup_tab));
    }
  }

#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
  std::cout << "\n有风险的障碍物是:" << std::endl;
#endif

  Int32_t max_risky_value = 0;
  for (Int32_t i = 0; i < result->risky_obj_list.Size(); ++i) {
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
    std::cout << "目标障碍物index: " << i << std::endl;
#endif

    CollisionTestOnPathResult::ObjInfo& rsk_obj_info =
        result->risky_obj_list[i];
    const CollisionTestResult::ObjInfo& test_ret = rsk_obj_info.test_ret;
    if (test_ret.risk_value > max_risky_value) {
      max_risky_value = test_ret.risk_value;
    }

    ClassifyCollisionInfo(obj_in, path, start_point, &rsk_obj_info);
  }

  for (Int32_t i = 0; i < result->uncertain_list.Size(); ++i) {
    ClassifyCollisionInfo(obj_in, path, start_point,
                          &(result->uncertain_list[i]));
  }

#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
  std::cout << "max_risky_value=" << max_risky_value << std::endl;
  std::cout << "######  Test Collision on path. (End)  ######" << std::endl;
#endif

  return (max_risky_value);
}

void ObjectMapImpl::IgnoreRightBehindDynamicObject(
    const CollisionTestObj& test_obj) {

  // Int32_t risk_value = 0;
  CallbackIgnoreRightBehindDynamicObject callback_ignore(test_obj, this);
  CallbackGetObject call_back_get_obj(this);

  // risk_value = object_space_.TestCollision(
  //       test_obj, &callback_ignore);
  object_space_.TestCollision(
        test_obj,call_back_get_obj, &callback_ignore);
}

void ObjectMapImpl::CallbackAddRiskyObject::Handle(
    const CollisionTestingBase::RiskTestResult& risky_obj) {
  const CollisionTestingBase::ObjectAssociation* risky_obj_ass =
      obj_map_->object_space_.GetObjectAssociation(risky_obj.obj_index);
  const Object& tar_obj = obj_map_->object_list_[risky_obj_ass->obj_list_index];

  if (tar_obj.ignore && (!param_->testing_ignored_object)) {
    return;
  }

  if (tar_obj.uncertain) {
    if (param_->return_uncertain_object &&
        (Nullptr_t != uncertain_obj_list_) &&
        (risky_obj.static_distance < 0.1F)) {
      common::StaticVector<CollisionTestingBase::RiskTestResult,
          CollisionTestResult::MAX_RISKY_OBJ_NUM>*
          risky_obj_list = uncertain_obj_list_;

      Int32_t risky_obj_list_size = risky_obj_list->Size();
      for (Int32_t i = 0; i < risky_obj_list_size; ++i) {
        CollisionTestingBase::RiskTestResult& obj =
            risky_obj_list->GetData(i);
        const CollisionTestingBase::ObjectAssociation* ass =
            obj_map_->object_space_.GetObjectAssociation(obj.obj_index);
        if (risky_obj_ass->obj_list_index == ass->obj_list_index) {
          map_var_t static_dist = common::Min(
                obj.static_distance, risky_obj.static_distance);
          if (obj < risky_obj) {
            obj = risky_obj;
          }
          obj.static_distance = static_dist;
          return;
        }
      }

      if (risky_obj_list->Full()) {
        const CollisionTestingBase::RiskTestResult* min_risky_obj =
            &(risky_obj_list->Front());
        Int32_t min_risky_obj_index = 0;
        for (Int32_t i = 1; i < risky_obj_list_size; ++i) {
          const CollisionTestingBase::RiskTestResult& tmp_obj =
              risky_obj_list->GetData(i);
          if (tmp_obj < (*min_risky_obj)) {
            min_risky_obj = &tmp_obj;
            min_risky_obj_index = i;
          }
        }
        // 当输入目标的碰撞风险比列表中碰撞风险最小的目标的碰撞风险大时，
        // 才将列表中碰撞风险最小的目标替换为当前输入目标。
        if ((*min_risky_obj) < risky_obj) {
          risky_obj_list->GetData(min_risky_obj_index) = risky_obj;
        }
      } else {
        risky_obj_list->PushBack(risky_obj);
      }
    }

    return;
  }

  if (param_->return_single_risky_object) {
    if (risky_obj_ass->dynamic) {
      if (risky_dynamic_obj_list_->Empty()) {
        risky_dynamic_obj_list_->PushBack(risky_obj);
      } else {
        CollisionTestingBase::RiskTestResult& front =
            risky_dynamic_obj_list_->Front();
        const CollisionTestingBase::ObjectAssociation* ass =
            obj_map_->object_space_.GetObjectAssociation(front.obj_index);
        if (risky_obj_ass->obj_list_index == ass->obj_list_index) {
          map_var_t static_dist = common::Min(
                front.static_distance, risky_obj.static_distance);
          if (front < risky_obj) {
            front = risky_obj;
          }
          front.static_distance = static_dist;
        } else {
          if (front < risky_obj) {
            front = risky_obj;
          }
        }
      }
    } else {
      if (risky_static_obj_list_->Empty()) {
        risky_static_obj_list_->PushBack(risky_obj);
      } else {
        CollisionTestingBase::RiskTestResult& front =
            risky_static_obj_list_->Front();
        if (front < risky_obj) {
          front = risky_obj;
        }
      }
    }
  } else {
    common::StaticVector<CollisionTestingBase::RiskTestResult,
        CollisionTestResult::MAX_RISKY_OBJ_NUM>* risky_obj_list = risky_static_obj_list_;
    if (risky_obj_ass->dynamic) {
      risky_obj_list = risky_dynamic_obj_list_;
    }

    Int32_t risky_obj_list_size = risky_obj_list->Size();
    for (Int32_t i = 0; i < risky_obj_list_size; ++i) {
      CollisionTestingBase::RiskTestResult& obj = risky_obj_list->GetData(i);
      const CollisionTestingBase::ObjectAssociation* ass =
          obj_map_->object_space_.GetObjectAssociation(obj.obj_index);
      if (risky_obj_ass->obj_list_index == ass->obj_list_index) {
        map_var_t static_dist = common::Min(
              obj.static_distance, risky_obj.static_distance);
        if (obj < risky_obj) {
          obj = risky_obj;
        }
        obj.static_distance = static_dist;
        return;
      }
    }

    bool full = false;
    if (risky_obj_ass->dynamic) {
      full = risky_obj_list->Size() >=
          param_->max_num_of_dynamic_obj_to_return ? true : false;
    } else {
      full = risky_obj_list->Size() >=
          param_->max_num_of_static_obj_to_return ? true : false;
    }

    if (full) {
      const CollisionTestingBase::RiskTestResult* min_risky_obj =
          &(risky_obj_list->Front());
      Int32_t min_risky_obj_index = 0;
      for (Int32_t i = 1; i < risky_obj_list_size; ++i) {
        const CollisionTestingBase::RiskTestResult& tmp_obj =
            risky_obj_list->GetData(i);
        if (tmp_obj < (*min_risky_obj)) {
          min_risky_obj = &tmp_obj;
          min_risky_obj_index = i;
        }
      }
      // 当输入目标的碰撞风险比列表中碰撞风险最小的目标的碰撞风险大时，
      // 才将列表中碰撞风险最小的目标替换为当前输入目标。
      if ((*min_risky_obj) < risky_obj) {
        risky_obj_list->GetData(min_risky_obj_index) = risky_obj;
      }
    } else {
      risky_obj_list->PushBack(risky_obj);
    }
  }
}

void ObjectMapImpl::CallbackIgnoreRightBehindDynamicObject::Handle(
    const CollisionTestingBase::RiskTestResult& risky_obj) {
  const CollisionTestingBase::ObjectAssociation* ass =
      obj_map_->object_space_.GetObjectAssociation(risky_obj.obj_index);

  if (ass->dynamic) {
    if (common::com_abs(
          common::AngleDiff(
            test_obj_heading_, ass->trj_point.path_point.heading)) <
        common::com_deg2rad(90.0F)) {
      if ((ass->trj_point.relative_time > 0.5F) &&
          (risky_obj.static_distance < 0.5F)) {
        obj_map_->object_list_[ass->obj_list_index].ignore = true;
      }
    }
  }
}

void ObjectMapImpl::AddCollisionInfoToList(
    const CollisionTestResult::ObjInfo& coll_info,
    common::StaticVector<CollisionTestOnPathResult::ObjInfo,
    CollisionTestOnPathResult::MAX_RISKY_OBJ_NUM>* risky_obj_list,
    common::UnorderedMap<Int32_t, Int32_t,
    CollisionTestOnPathResult::MAX_RISKY_OBJ_NUM>* risky_obj_lookup_tab) const {
  const Object& tar_obj = GetObject(coll_info.obj_list_index);

#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
  std::cout << "  *** 目标障碍物index:" << j << std::endl;
  std::cout << "      ID:" << tar_obj.id << std::endl;
  std::cout << "      碰撞信息: "
            << "风险值(" << coll_info.risk_value
            << ") 静态距离(" << coll_info.static_distance
            << ") 动态距离(" << coll_info.dynamic_distance
            << ") 碰撞路径长(" << coll_info.collision_s
            << ") 采样点{t=" << coll_info.obj_traj_point.relative_time
            << ",v=" << coll_info.obj_traj_point.v*3.6F
            << "} "
            << std::endl;
#endif

  if (coll_info.static_distance > 2.0F) {
    return;
  }

  const Int32_t* risky_obj_index =
      risky_obj_lookup_tab->Find(coll_info.obj_list_index);
  if (Nullptr_t == risky_obj_index) {
    // 还未添加到列表中
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
    std::cout << "      还未添加到列表中" << std::endl;
#endif
    if (risky_obj_list->Full()) {
      // 列表已经满了
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
      std::cout << "      列表已经满了" << std::endl;
#endif
      // 找到列表中风险最小的，若其风险小于当前的，则替换它
      Int32_t min_risk_obj_index = 0;
      for (Int32_t i = 1; i < risky_obj_list->Size(); ++i) {
        if (risky_obj_list->GetData(i).test_ret <
            risky_obj_list->GetData(min_risk_obj_index).test_ret) {
          min_risk_obj_index = i;
        }
      }
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
      std::cout << "      列表中风险最小的,index=" << min_risk_obj_index << std::endl;
#endif
      CollisionTestOnPathResult::ObjInfo& min_risk_obj =
          risky_obj_list->GetData(min_risk_obj_index);
      if (min_risk_obj.test_ret < coll_info) {
        // 当前碰撞风险大
        // 从索引表中删除旧的索引
        risky_obj_lookup_tab->Erase(min_risk_obj.test_ret.obj_list_index);
        // 替换为新的障碍物信息
        min_risk_obj.min_static_distance = coll_info.static_distance;
        min_risk_obj.test_ret = coll_info;
        // 添加新的索引信息到索引表
        risky_obj_lookup_tab->Insert(
              coll_info.obj_list_index, min_risk_obj_index);
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
        std::cout << "      列表中风险最小的 风险小于 当前, 替换它" << std::endl;
#endif
      }
    } else {
      // 列表未满
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
      std::cout << "      列表未满" << std::endl;
#endif
      Int32_t obj_idx = -1;
      CollisionTestOnPathResult::ObjInfo* rsk_obj_info =
          risky_obj_list->Allocate(&obj_idx);
      if (Nullptr_t != rsk_obj_info) {
        // 添加新的障碍物信息
        rsk_obj_info->min_static_distance = coll_info.static_distance;
        rsk_obj_info->test_ret = coll_info;
        // 添加新的索引信息到索引表
        risky_obj_lookup_tab->Insert(
              coll_info.obj_list_index, obj_idx);
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
        std::cout << "      添加到列表中，列表索引" << obj_idx << std::endl;
#endif
      } else {
        // 这个错误是不应当发生的
        LOG_ERR << "Can't add risky obj to list.";
      }
    }
  } else {
    // 已经添加到列表中了
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
    std::cout << "      已经添加到列表中了" << std::endl;
#endif
    CollisionTestOnPathResult::ObjInfo& rsk_obj_info =
        risky_obj_list->GetData(*risky_obj_index);
    // 若当前风险大，则更新为新的风险信息
    if (coll_info.static_distance < rsk_obj_info.min_static_distance) {
      rsk_obj_info.min_static_distance = coll_info.static_distance;
    }
    if (rsk_obj_info.test_ret < coll_info) {
      // 当前碰撞风险大
      rsk_obj_info.test_ret = coll_info;
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
      std::cout << "      当前碰撞风险大, 替换碰撞信息" << std::endl;
#endif
    }
  }
}

void ObjectMapImpl::ClassifyCollisionInfo(
    const CollisionTestOnPathObj& obj_in,
    const common::Path& path,
    const common::PathPoint& start_point,
    CollisionTestOnPathResult::ObjInfo* coll_info) const {
  // obj_in 自车 ，coll_info 障碍物
  const CollisionTestResult::ObjInfo& test_ret = coll_info->test_ret;
  const Object& tar_obj = GetObject(test_ret.obj_list_index);

  path.FindProjection(tar_obj.pos, &(coll_info->tar_obj_proj));
  coll_info->dist_to_tar_obj = //后轴
      coll_info->tar_obj_proj.s - start_point.s + obj_in.s_offset;

  coll_info->cut_in = false;
  map_var_t static_dist_to_tar_obj = 0.0F;
  if (!tar_obj.dynamic) {
    // Static obstacle 
    static_dist_to_tar_obj = test_ret.static_distance;
  } else {
    // Dynamic obstacle
    common::OBBox2d obb_check;
    map_var_t heading = coll_info->tar_obj_proj.heading;
    if (obj_in.is_backing) {
      // For backing mode of vehicle
      heading = common::NormalizeAngle(heading + COM_PI);
    }

    obb_check.set_unit_direction(
          common::com_cos(heading), common::com_sin(heading));
    obb_check.set_center( 
          coll_info->tar_obj_proj.point +
          obj_in.x_offset * obb_check.unit_direction_x()); //几何点坐标 = 后轴点坐标 + 后轴点到几何中心的向量距离
    obb_check.set_extents(obj_in.obj_half_length, obj_in.obj_half_width);
    // obb_check 自车 走到 障碍物投影到轨迹上的点 
    static_dist_to_tar_obj =
        common::DistOBBToOBB_2D(obb_check, tar_obj.obb);

    if ((static_dist_to_tar_obj > 0.01F) &&
        (coll_info->min_static_distance < 0.01F)) {
      coll_info->cut_in = true;
    }
  }

  if (coll_info->dist_to_tar_obj >= 0.0F) {
    if ((obj_in.x_offset+obj_in.obj_half_length) < coll_info->dist_to_tar_obj) { //TODO:下一步改为：0 < coll_info->dist_to_tar_obj
      if (static_dist_to_tar_obj > 0.2F) {
        if (coll_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左前方
          coll_info->obj_position = OBJ_POSITION_LEFT_FRONT;
        } else {
          // 障碍物本体位于右前方
          coll_info->obj_position = OBJ_POSITION_RIGHT_FRONT;
        }
      } else {
        // 障碍物本体位于正前方
        coll_info->obj_position = OBJ_POSITION_FRONT;
      }
    } else {
      if (static_dist_to_tar_obj > 0.2F) {
        if (coll_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左侧方
          coll_info->obj_position = OBJ_POSITION_LEFT;
        } else {
          // 障碍物本体位于右侧方
          coll_info->obj_position = OBJ_POSITION_RIGHT;
        }
      } else {
        // 障碍物本体紧靠自车
        coll_info->obj_position = OBJ_POSITION_CLOSE;
      }
    }
  } else {
    if ((obj_in.x_offset-obj_in.obj_half_length) > coll_info->dist_to_tar_obj) {
      if (static_dist_to_tar_obj > 0.2F) {
        if (coll_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左后方
          coll_info->obj_position = OBJ_POSITION_LEFT_BACK;
        } else {
          // 障碍物本体位于右后方
          coll_info->obj_position = OBJ_POSITION_RIGHT_BACK;
        }
      } else {
        // 障碍物本体位于正后方
        coll_info->obj_position = OBJ_POSITION_BACK;
      }
    } else {
      if (static_dist_to_tar_obj > 0.2F) {
        if (coll_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左侧方
          coll_info->obj_position = OBJ_POSITION_LEFT;
        } else {
          // 障碍物本体位于右侧方
          coll_info->obj_position = OBJ_POSITION_RIGHT;
        }
      } else {
        // 障碍物本体紧靠自车
        coll_info->obj_position = OBJ_POSITION_CLOSE;
      }
    }
  }

  map_var_t angle_diff = common::com_abs(
        common::AngleDiff(tar_obj.heading, coll_info->tar_obj_proj.heading));
  if (angle_diff < common::com_deg2rad(60.0F)) {
    coll_info->obj_direction = OBJ_DIRECTION_FORWARD;
  } else if (angle_diff > common::com_deg2rad(120.0F)) {
    coll_info->obj_direction = OBJ_DIRECTION_BACKWARD;
  } else {
    coll_info->obj_direction = OBJ_DIRECTION_CROSSED;
  }
#if (ENABLE_OUTPUT_TEST_COLLISION_LOG)
  std::cout << "  目标障碍物ID=" << tar_obj.id << std::endl;
  std::cout << "  最小静态距离="
            << coll_info->min_static_distance << std::endl;
  std::cout << "  到目标障碍物的距离="
            << coll_info->dist_to_tar_obj << std::endl;
  std::cout << "  目标障碍物所在区域="
            << coll_info->obj_position << std::endl;
  std::cout << "  目标障碍物方向="
            << coll_info->obj_direction << std::endl;
  std::cout << "  动态="
            << (Int32_t)tar_obj.dynamic << std::endl;
  std::cout << "  cut_in=" << coll_info->cut_in << std::endl;
  std::cout << "  碰撞信息: "
            << "风险值(" << test_ret.risk_value
            << ") 静态距离(" << test_ret.static_distance
            << ") 动态距离(" << test_ret.dynamic_distance
            << ") 碰撞路径长(" << test_ret.collision_s
            << ") 采样点{t=" << test_ret.obj_traj_point.relative_time
            << ",v=" << test_ret.obj_traj_point.v*3.6F
            << "} "
            << std::endl;
#endif
}


}  // namespace driv_map
}  // namespace phoenix
