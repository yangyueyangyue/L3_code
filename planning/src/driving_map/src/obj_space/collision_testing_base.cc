//
#include "obj_space/collision_testing_base.h"


namespace phoenix {
namespace driv_map {


CollisionTestingBase::CollisionTestingBase() {
}

CollisionTestingBase::~CollisionTestingBase() {
}


void CollisionTestingBase::Clear() {
}

bool CollisionTestingBase::AddObject(
    const Object& new_obj,
    Int32_t new_obj_list_index) {
  return false;
}

void CollisionTestingBase::CompleteAddingObject() {
}

const CollisionTestingBase::ObjectAssociation*
CollisionTestingBase::GetObjectAssociation(
    Int32_t obj_index) const {
  return (Nullptr_t);
}

Int32_t CollisionTestingBase::TestCollision(
    const CollisionTestObj& test_obj,
    const CallbackGetObject& callback_get_obj,
    CallbackIfObjHasRisk* callback_if_has_risk) const {
  return 0;
}

Int32_t CollisionTestingBase::TestCollisionPairwise(
    const CollisionTestObj& test_obj, const ObjInfoBase& obj_in_space,
    const CallbackGetObject& callback_get_obj,
    RiskTestResult* result) const {
  Int32_t risk_value = 0;
  // 计算风险值时需要使用到的中间变量
  Float32_t risk_value_tmp = 0.0F;
  map_var_t distance = phoenix::common::DistOBBToOBB_2D(
        test_obj.obb, obj_in_space.obb);
  result->static_distance = distance;

  const Object* tar_obj = callback_get_obj.Get(
        obj_in_space.obj_association.obj_list_index);
  if (Nullptr_t == tar_obj) {
    LOG_ERR << "Invalid target obj.";
    result->obj_index = obj_in_space.obj_index;
    result->risk_value = risk_value;
    result->dynamic_distance = distance;
    result->collision_s = test_obj.s;

    return (0);
  }

  if (obj_in_space.obj_association.dynamic) {
    // 对于动态障碍物还需要在S-T中分析是否会碰撞
    map_var_t t_diff = common::com_abs(
          obj_in_space.obj_association.trj_point.relative_time - test_obj.t);
    map_var_t s = obj_in_space.obj_association.trj_point.v * t_diff;
    map_var_t safe_dist = obj_in_space.obb_radius + test_obj.obb_circumradius;
    if (s > safe_dist) {
      distance += s;
    }
  }

  // 根据距离计算风险值
  if (distance < 0.1F) {
    risk_value_tmp = 100.0F;
  } else if (distance < 0.5F) {
    risk_value_tmp = 100.0F - (distance / 0.5F) * 15.0F;
  } else if (distance < 1.0F) {
    risk_value_tmp = 100.0F - (distance / 1.0F) * 40.0F;
  } else if (distance < 2.0F) {
    risk_value_tmp = 100.0F - (distance / 2.0F) * 90.0F;
  } else {
    risk_value_tmp = 0.0F;
  }
  risk_value = static_cast<Int32_t>(risk_value_tmp);

  // 保存碰撞风险信息
  result->obj_index = obj_in_space.obj_index;
  result->risk_value = risk_value;
  result->dynamic_distance = distance;
  result->collision_s = test_obj.s;

  return (risk_value);
}


}  // namespace driv_map
}  // namespace phoenix

