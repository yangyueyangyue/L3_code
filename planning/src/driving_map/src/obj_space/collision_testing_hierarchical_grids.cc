/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       object_hierarchical_grids.cc
 * @brief      分层网格空间分割
 * @details    使用分层的二维网格来分割某个二维空间，用于加快二维碰撞分析的速度
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

#include "obj_space/collision_testing_hierarchical_grids.h"
#include "geometry/geometry_utils.h"

namespace phoenix {
namespace driv_map {


CollisionTestingHierarchicalGrids::CollisionTestingHierarchicalGrids(
    map_var_t min_cell_size,
    map_var_t sphere_to_cell_ratio,
    map_var_t cell_to_cell_ratio) {
  min_cell_size_ = min_cell_size;
  sphere_to_cell_ratio_ = sphere_to_cell_ratio;
  cell_to_cell_ratio_ = cell_to_cell_ratio;

  occupied_levels_mask_ = 0;
  common::com_memset(objects_at_level_, 0, sizeof(objects_at_level_));

  obj_list_.Clear();
  grid_bucket_.Resize(MAX_GRID_BUCKET_NUM);
}

void CollisionTestingHierarchicalGrids::Clear() {
  // 清除内部变量
  occupied_levels_mask_ = 0;
  common::com_memset(objects_at_level_, 0, sizeof(objects_at_level_));

  // 清除网格中的障碍物列表
  obj_list_.Clear();

  // 需要清除每个网格单元中的信息
  for (Int32_t i = 0; i < grid_bucket_.Size(); ++i) {
    grid_bucket_[i].Clear();
  }
}

bool CollisionTestingHierarchicalGrids::AddObject(
    const Object& new_obj, Int32_t new_obj_list_index) {
  if (new_obj_list_index < 0) {
    LOG_ERR << "Invalid object index.";
    return false;
  }

  Int32_t obj_index = -1;
  Obj* obj = obj_list_.Allocate(&obj_index);
  if (Nullptr_t == obj) {
    LOG_ERR << "Storage is full and cann't save objects anymore.";
    return false;
  }

  // 计算障碍物包围盒的外接圆半径
  map_var_t obb_radius =
      common::com_sqrt(common::Square(new_obj.obb.extents().x()) +
          common::Square(new_obj.obb.extents().y()));
  // 和上层模块的障碍物的关联信息
  obj->base.obj_association.obj_list_index = new_obj_list_index;
  obj->base.obj_association.dynamic = new_obj.dynamic;
  obj->base.obj_association.pred_trj_index = -1;
  obj->base.obj_association.pred_trj_point_index = -1;
  obj->base.obj_association.trj_point.path_point.point = new_obj.obb.center();
  obj->base.obj_association.trj_point.path_point.heading = new_obj.heading;
  obj->base.obj_association.trj_point.path_point.curvature = 0;
  obj->base.obj_association.trj_point.path_point.s = 0;
  obj->base.obj_association.trj_point.path_point.l = 0;
  // 速度
  obj->base.obj_association.trj_point.v = new_obj.v;
  // 加速度
  obj->base.obj_association.trj_point.a = new_obj.a;
  // 横摆角速度
  obj->base.obj_association.trj_point.yaw_rate = new_obj.yaw_rate;
  // 相对于曲线起点已行驶的相对时间
  obj->base.obj_association.trj_point.relative_time = 0;
  // 障碍物的包围盒
  obj->base.obb = new_obj.obb;
  common::ConstructAABBoxFromOBBox_2D(obj->base.obb, &obj->base.aabb);
  // 障碍物的包围盒的外接圆的半径
  obj->base.obb_radius = obb_radius;
  // index of in the obj_list_
  obj->base.obj_index = obj_index;

  if (new_obj.dynamic) {
    // 添加动态障碍物，其预测轨迹上的每个采样点都会作为障碍物添加到网格中
    if (new_obj.pred_trajectory.Empty()) {
      if (false == AddObjectInternal(obj)) {
        obj_list_.PopBack();
        LOG_ERR << "Cann't add the object to this grid.";
        return (false);
      }
    } else {
      if (new_obj.pred_trajectory[0].Empty()) {
        if (false == AddObjectInternal(obj)) {
          obj_list_.PopBack();
          LOG_ERR << "Cann't add the object to this grid.";
          return (false);
        }
      } else {
        // 由于动态障碍物当前时刻的位置在其轨迹中已经包含了，所以在
        // 添加其轨迹采样点处的障碍物目标前，先将其当前时刻处的障碍
        // 物目标从列表中删除。
        obj_list_.PopBack();
        // 添加是否失败
        // 添加失败说明预测轨迹采样点上的障碍物，由于障碍物列表已经
        // 满了，不再能添加进去。但是由于至少添加了一个轨迹采样点上
        // 的目标，所以仍然返回成功。
        bool failed_to_add = false;
        for (Int32_t i = 0; i < new_obj.pred_trajectory.Size(); ++i) {
          for (Int32_t j = 0; j < new_obj.pred_trajectory[i].Size(); ++j) {
            obj_index = -1;
            obj = obj_list_.Allocate(&obj_index);
            if (Nullptr_t == obj) {
              LOG_ERR << "Storage is full and cann't save objects anymore.";
              //return false;
              failed_to_add = true;
              break;
            }
            // 和上层模块的障碍物的关联信息
            obj->base.obj_association.obj_list_index = new_obj_list_index;
            obj->base.obj_association.dynamic = new_obj.dynamic;
            obj->base.obj_association.pred_trj_index = i;
            obj->base.obj_association.pred_trj_point_index = j;
            obj->base.obj_association.trj_point = new_obj.pred_trajectory[i][j];
            // 障碍物的包围盒
            obj->base.obb.set_center(new_obj.pred_trajectory[i][j].path_point.point);
            obj->base.obb.set_unit_direction(
                  common::com_cos(
                    new_obj.pred_trajectory[i][j].path_point.heading),
                  common::com_sin(
                    new_obj.pred_trajectory[i][j].path_point.heading));
            // Extend the radius of the bounding box of obstacle according to
            // the distance between the adjacent samples of the predicted path
            // of this dynamic obstacle.
            map_var_t diff_s_1 = 0;
            map_var_t diff_s_2 = 0;
            if (j < (new_obj.pred_trajectory[i].Size()-1)) {
              diff_s_1 = common::com_abs(
                  new_obj.pred_trajectory[i][j+1].path_point.s -
                  new_obj.pred_trajectory[i][j].path_point.s);
            }
            if (j > 0) {
              diff_s_2 = common::com_abs(
                  new_obj.pred_trajectory[i][j].path_point.s -
                  new_obj.pred_trajectory[i][j-1].path_point.s);
            }
            obj->base.obb.set_extents(
                  common::Max(common::Max(0.5F * diff_s_1, 0.5F * diff_s_2),
                              new_obj.obb.extents().x()),
                  new_obj.obb.extents().y());
            common::ConstructAABBoxFromOBBox_2D(obj->base.obb, &obj->base.aabb);
            // 障碍物的包围盒的外接圆的半径
            obj->base.obb_radius = obj->base.obb.CalcCircumradius();
            // index of in the obj_list_
            obj->base.obj_index = obj_index;

            if (false == AddObjectInternal(obj)) {
              obj_list_.PopBack();
              LOG_ERR << "Cann't add the object to this grid.";
              //return (false);
              failed_to_add = true;
              break;
            }
          }
          if (failed_to_add) {
            break;
          }
        }
      }
    }
  } else {
    // 添加静态障碍物
    if (false == AddObjectInternal(obj)) {
      obj_list_.PopBack();
      LOG_ERR << "Cann't add the object to this grid.";
      return (false);
    }
  }

  return true;
}

const CollisionTestingBase::ObjectAssociation*
CollisionTestingHierarchicalGrids::GetObjectAssociation(
    Int32_t obj_index) const {
  COM_CHECK((obj_index >= 0) && (obj_index < obj_list_.Size()));
  if ((obj_index >= 0) && (obj_index < obj_list_.Size())) {
    return (&(obj_list_[obj_index].base.obj_association));
  }

  return (Nullptr_t);
}

Int32_t CollisionTestingHierarchicalGrids::TestCollision(
    const CollisionTestObj& test_obj,
    const CallbackGetObject& callback_get_obj,
    CallbackIfObjHasRisk* callback_if_has_risk) const {
  if (0 == occupied_levels_mask_) {
    // no obstacle in these grids
    return (0);
  }

  // Initialize the risky object list
  Int32_t ret_risk_value = 0;

  map_var_t size = min_cell_size_;
  Uint32_t occupied_levels_mask = occupied_levels_mask_;

  Int32_t x_min = 0;
  Int32_t y_min = 0;
  Int32_t x_max = 0;
  Int32_t y_max = 0;
  RiskTestResult risk_test_result;
  for (Int32_t level = 0; level < HGRID_MAX_LEVEL;
       size *= cell_to_cell_ratio_, occupied_levels_mask >>= 1, level++) {
    // If no objects at this level, go on to the next level
    if (0 == (occupied_levels_mask & 1)) continue;
    // Compute ranges [x1..x2, y1..y2] of cells overlapped on this level. To
    // make sure objects in neighboring cells are tested, by increasing range by
    // the maximum object overlap: size * SPHERE_TO_CELL_RATIO
    CalcCellsOverlapped(test_obj.obb, test_obj.obb_circumradius, size,
                        &x_min, &y_min, &x_max, &y_max);

    // Check all the grid cells overlapped on current level
    for (Int32_t x = x_min; x <= x_max; ++x) {
      for (Int32_t y = y_min; y <= y_max; y++) {
        // Treat level as a third dimension coordinate
        CellKey cell_key(x, y, level);
        Int32_t bucket = CalcBucketIndex(cell_key);
        const Cell& cell = grid_bucket_[bucket];
        // Loop through all objects in the bucket to find nearby objects
        for (Int32_t i = 0; i < cell.items().Size(); ++i) {
          Int32_t obj_index = cell.items()[i];
          const Obj& curr_obj = obj_list_[obj_index];
          map_var_t sq_dist = (test_obj.obb.center() -
                               curr_obj.base.obb.center()).LengthSquare();
          map_var_t radius = test_obj.obb_circumradius +
              curr_obj.base.obb_radius + 2.0F;
          if (sq_dist <= (radius * radius)) {
            // Close, do narrow phase pairwise tests
            Int32_t risk_value = TestCollisionPairwise(
                  test_obj, curr_obj.base, callback_get_obj, &risk_test_result);
            if (risk_value > ret_risk_value) {
              ret_risk_value = risk_value;
            }
            // Callback if this object has risk to inputted object
            callback_if_has_risk->Handle(risk_test_result);
          }
        }  // for (Int32_t i = 0; i < cell.items().Size(); ++i)
      }  // for (Int32_t y = y_min; y <= y_max; y++) {
    }  // for (Int32_t x = x_min; x <= x_max; ++x) {
  }  // for (Int32_t level = 0; level < HGRID_MAX_LEVEL;

  return (ret_risk_value);
}

void CollisionTestingHierarchicalGrids::CalcCellsOverlapped(
    const common::OBBox2d& obj, map_var_t obj_radius, map_var_t cell_size,
    Int32_t* x_min, Int32_t* y_min,
    Int32_t* x_max, Int32_t* y_max) const {
  map_var_t delta = obj_radius + cell_size * sphere_to_cell_ratio_
      + phoenix::common::kGeometryEpsilon;
  map_var_t inverse_size = 1.0f / cell_size;
  *x_min = phoenix::common::com_floor(
      (obj.center().x() - delta) * inverse_size);
  *y_min = phoenix::common::com_floor(
      (obj.center().y() - delta) * inverse_size);
  *x_max = phoenix::common::com_ceil(
      (obj.center().x() + delta) * inverse_size);
  *y_max = phoenix::common::com_ceil(
      (obj.center().y() + delta) * inverse_size);
}

bool CollisionTestingHierarchicalGrids::AddObjectInternal(Obj* new_obj) {
  // Find lowest level where object fully fits inside cell,
  // taking RATIO into account
  Int32_t level = 0;
  map_var_t size = min_cell_size_;
  map_var_t diameter = 2 * new_obj->base.obb_radius;
  for (level = 0; size * sphere_to_cell_ratio_ < diameter; level++) {
    size *= cell_to_cell_ratio_;
  }

  // Assert if object is larger than largest grid cell
  if (level >= HGRID_MAX_LEVEL) {
    LOG_ERR << "The object needs grid level (" << level
            << ") that is more than the max level.";
    return false;
  }

  // Add object to grid square, and remember cell and level numbers,
  // treating level as a third dimension coordinate
  CellKey key(new_obj->base.obb.center().x() / size,
      new_obj->base.obb.center().y() / size, level);
  Int32_t bucket = CalcBucketIndex(key);
  new_obj->bucket = bucket;
  new_obj->cell_key = key;
  if (false == grid_bucket_[bucket].PushBack(new_obj->base.obj_index)) {
    LOG_ERR << "The storage in the bucket[" << bucket << "] is full, level="
            << level << ", size=" << size << ", diameter=" << diameter;
    return (false);
  }

  // Mark this level as having one more object. Also indicate level is in use
  objects_at_level_[level]++;
  occupied_levels_mask_ |= (1 << level);

  return (true);
}


}  // namespace driv_map
}  // namespace phoenix


