//
#include "radar/filter_srr2_detections.h"

#include "utils/log.h"


namespace phoenix {
namespace perception {


FilterSrr2Detections::FilterSrr2Detections() {
  // nothing to do
}

FilterSrr2Detections::~FilterSrr2Detections() {
}

bool FilterSrr2Detections::Filter(
    Int32_t radar_idx,
    const ad_msg::ObstacleRadarList& detection_list,
    const ad_msg::Chassis& chassis,
    ad_msg::ObstacleRadarList* tracked_obj_list) {
  tracked_obj_list->Clear();

  if ((radar_idx < 0) || (radar_idx >= MAX_RADAR_NUM)) {
    LOG_ERR << "Invalid radar index(" << radar_idx << ").";
    return false;
  }

  //if (!detection_list.msg_head.valid) {
  //  return false;
  //}
  if (detection_list.obstacle_num < 1) {
    return false;
  }

  // 清理 Tracked Object List
  tracked_objects_.Clear();

  // 清理KD-Tree
  phoenix::common::AABBoxKDTreeParams params;
  params.max_leaf_dimension = 5.0f;  // meters.
  params.max_leaf_size = 16;
  tracked_objects_kdtree_.SetKDTreeParams(params);
  tracked_objects_kdtree_.Clear();
  kdtree_nodes_stack_.Clear();

  // 构建跟踪表
  for (Int32_t i = 0; i < detection_list.obstacle_num; ++i) {
    ObjIndex tracked_obj_idx;
    const ad_msg::ObstacleRadar& src_obj = detection_list.obstacles[i];
    ObjTracked* dst_obj = tracked_objects_.objects.Allocate(
          &tracked_obj_idx.obj_idx);
    if (Nullptr_t == dst_obj) {
      LOG_ERR << "Failed to add tracked obj, storage is full.";
      break;
    }

    dst_obj->raw_obj_idx = i;
    dst_obj->erase = false;

    dst_obj->obj_id = src_obj.id;

    dst_obj->pos.x = src_obj.x;
    dst_obj->pos.y = src_obj.y;
    dst_obj->length = 0.5F;
    dst_obj->width = 0.0F;
    Scalar half_width = 0.5F * dst_obj->width;
    Scalar half_lenght = 0.5F * dst_obj->length;
    dst_obj->aabb.min().set_x(dst_obj->pos.x - half_lenght);
    dst_obj->aabb.min().set_y(dst_obj->pos.y - half_width);
    dst_obj->aabb.max().set_x(dst_obj->pos.x + half_lenght);
    dst_obj->aabb.max().set_y(dst_obj->pos.y + half_width);

    dst_obj->v_x = src_obj.v_x;
    dst_obj->v_y = src_obj.v_y;

    tracked_objects_kdtree_.AddObject(tracked_obj_idx, dst_obj->aabb);
  }
  Int32_t tracked_obj_num = tracked_objects_.objects.Size();
  if (tracked_obj_num < 1) {
    LOG_ERR << "There isn't enough tracked objects.";
    return false;
  }
  // 根据跟踪列表中的障碍物, 构建KD-Tree, 加快最近障碍物的检索
  tracked_objects_kdtree_.Partition(kdtree_nodes_stack_);

  // 聚类
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjTracked& src_obj = tracked_objects_.objects[i];

    if (src_obj.erase) {
      continue;
    }

    common::StaticVector<Int32_t, MAX_PARTNER_NUM> obj_idxs_in_range;

    // 查找临近的所有障碍物（检索半径以内）
    Scalar searching_radius = 2.0F;
    tracked_objects_kdtree_.FindObjects(
          kdtree_nodes_stack_,
          common::Vec2d(src_obj.pos.x, src_obj.pos.y),
          searching_radius,
          CalcSquareDistToObjTracked(src_obj.aabb, tracked_objects_.objects),
          &obj_idxs_in_range);

    Int32_t num_objs_in_range = obj_idxs_in_range.Size();

    if (num_objs_in_range < 3) {
      src_obj.erase = true;
      continue;
    }

    Int32_t max_matched_obj_idx = i;
    Int32_t max_matched_num = num_objs_in_range;
    common::StaticVector<Int32_t, MAX_PARTNER_NUM>
        max_matched_obj_idxs_in_range = obj_idxs_in_range;
    for (Int32_t j = 0; j < num_objs_in_range; ++j) {
      const ObjIndex* idx =
          tracked_objects_kdtree_.GetObjectByIndex(obj_idxs_in_range[j]);
      if (Nullptr_t == idx) {
        LOG_ERR << "Invalid ObjIndex";
        continue;
      }
      if (!idx->IsValid()) {
        //LOG_ERR << "Invalid obj index.";
        continue;
      }
      if (idx->obj_idx == i) {
        continue;
      }
      ObjTracked& obj = tracked_objects_.objects[idx->obj_idx];

      common::StaticVector<Int32_t, MAX_PARTNER_NUM> tmp_obj_idxs_in_range;
      tracked_objects_kdtree_.FindObjects(
            kdtree_nodes_stack_,
            common::Vec2d(obj.pos.x, obj.pos.y),
            searching_radius,
            CalcSquareDistToObjTracked(obj.aabb, tracked_objects_.objects),
            &tmp_obj_idxs_in_range);
      if (tmp_obj_idxs_in_range.Size() > max_matched_num) {
        max_matched_obj_idx = idx->obj_idx;
        max_matched_num = tmp_obj_idxs_in_range.Size();
        max_matched_obj_idxs_in_range = tmp_obj_idxs_in_range;
      }
    }

    const ObjTracked& matched_obj =
        tracked_objects_.objects[max_matched_obj_idx];
    Scalar sum_x = 0.0F;
    Scalar sum_y = 0.0F;
    for (Int32_t j = 0; j < max_matched_num; ++j) {
      const ObjIndex* idx =
          tracked_objects_kdtree_.GetObjectByIndex(max_matched_obj_idxs_in_range[j]);
      if (Nullptr_t == idx) {
        LOG_ERR << "Invalid ObjIndex";
        continue;
      }
      if (!idx->IsValid()) {
        //LOG_ERR << "Invalid obj index.";
        continue;
      }

      ObjTracked& obj = tracked_objects_.objects[idx->obj_idx];

      if (idx->obj_idx != max_matched_obj_idx) {
        obj.erase = true;
      }

      sum_x += obj.pos.x;
      sum_y += obj.pos.y;
    }

    // 输出跟踪后的障碍物
    if (tracked_obj_list->obstacle_num >=
        ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM) {
      break;
    }
    ad_msg::ObstacleRadar& dst_obj =
        tracked_obj_list->obstacles[tracked_obj_list->obstacle_num];
    tracked_obj_list->obstacle_num++;

    dst_obj.id = matched_obj.obj_id;
    dst_obj.x = sum_x / max_matched_num;
    dst_obj.y = sum_y / max_matched_num;
    dst_obj.v_x = 0.0F;//matched_obj.v_x;
    dst_obj.v_y = 0.0F;//matched_obj.v_y;
  }

  tracked_obj_list->msg_head = detection_list.msg_head;

  return true;
}


}  // namespace perception
}  // namespace phoenix

