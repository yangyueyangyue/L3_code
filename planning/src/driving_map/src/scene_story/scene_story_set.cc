/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       scene_story_set.cc
 * @brief      场景任务
 * @details    定义场景任务的集合
 *
 * @author     longjiaoy
 * @date       2021.12.10
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2021/12/10  <td>1.0      <td>longjiaoy  <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "scene_story/scene_story_set.h"


namespace phoenix {
namespace driv_map {


SceneStorySet::SceneStorySet() {
  Clear();
  mat_convert_utm_to_rel_coor_.SetIdentity();
}

void SceneStorySet::Clear() {
  raw_scene_story_list_.Clear();
  for (Int32_t i = 0; i < scene_story_list_.Size(); ++i) {
    scene_story_list_[i].Clear();
  }
  scene_story_list_.Clear();
}

void SceneStorySet::UpdateSceneStorys(
    Int64_t timestamp, const ad_msg::SceneStoryList* story_list,
    const common::Path& ref_path, const common::PathPoint& veh_on_ref_path) {
  Int64_t time_elapsed = 0;
  if (raw_scene_story_list_.msg_head.valid) {
    time_elapsed = common::CalcElapsedClockMs(
          raw_scene_story_list_.msg_head.timestamp, timestamp);
    if (time_elapsed > 2000 || time_elapsed < 0) {
      // 之前保存的列表过期了，清除它
      Clear();
    }
  }

  if (Nullptr_t != story_list) {
    if (story_list->msg_head.valid) {
      Uint32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
            raw_scene_story_list_.msg_head.sequence,
            story_list->msg_head.sequence);
      if (sequence_diff > 0) {
        // 当前传递进来的列表是有效的, 且是更新后的,
        raw_scene_story_list_ = *story_list;
      }
    }
  }

  scene_story_list_.Clear();
  for (Int32_t i = 0; i < raw_scene_story_list_.story_num; ++i) {
    const ad_msg::SceneStory& story = raw_scene_story_list_.storys[i];
    if (ad_msg::SCENE_STORY_TYPE_INVALID == story.type) {
      continue;
    }
    scene_story_list_.PushBack(story);

    ad_msg::SceneStory& new_story = scene_story_list_.Back();
    if (ad_msg::SceneStoryArea::AREA_TYPE_02 == story.area.area_type) {
      ConvertUtmPointToRelCoordinate(
            story.area.area_type_02.control_line.start_point,
            &new_story.area.area_type_02.control_line.start_point);
      ConvertUtmPointToRelCoordinate(
            story.area.area_type_02.control_line.end_point,
            &new_story.area.area_type_02.control_line.end_point);

      common::Vec2d start_point(
            static_cast<common::geo_var_t>(new_story.area.area_type_02.control_line.start_point.x),
            static_cast<common::geo_var_t>(new_story.area.area_type_02.control_line.start_point.y));
      common::Vec2d end_point(
            static_cast<common::geo_var_t>(new_story.area.area_type_02.control_line.end_point.x),
            static_cast<common::geo_var_t>(new_story.area.area_type_02.control_line.end_point.y));
      common::PathPoint cross_point;
      bool is_cross = ref_path.IsIntersect(start_point, end_point, &cross_point);
      if (is_cross) {
        new_story.area.area_type_02.valid = true;
        new_story.area.area_type_02.dist_to_ctl_line =
            cross_point.s - veh_on_ref_path.s;
      } else {
        new_story.area.area_type_02.valid = false;
      }
    }
  }
}

void SceneStorySet::ConvertUtmPointToRelCoordinate(
    const ad_msg::SceneStoryControlLine::Point2D& point_in,
    ad_msg::SceneStoryControlLine::Point2D* point_out) {
  common::Matrix<Float64_t, 2, 1> point_conv;

  point_conv(0) = point_in.x;
  point_conv(1) = point_in.y;

  common::TransformVert_2D(mat_convert_utm_to_rel_coor_, &point_conv);

  point_out->x = point_conv(0);
  point_out->y = point_conv(1);
}


} // namespace driv_map
} // namespace phoenix

