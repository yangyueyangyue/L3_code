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

#include "planning_story/planning_story_set.h"


namespace phoenix {
namespace driv_map {


PlanningStorySet::PlanningStorySet() {
  Clear();
  mat_convert_utm_to_rel_coor_.SetIdentity();
}

void PlanningStorySet::Clear() {
  planning_story_list_.Clear();
}

void PlanningStorySet::UpdateStorys(
    const ad_msg::PlanningStoryList& story_list,
    const ad_msg::RelativePos& current_position) {
  planning_story_list_ = story_list;

  Int32_t story_num = planning_story_list_.story_num;
  for (Int32_t i = 0; i < story_num; ++i) {
    ad_msg::PlanningStory& story = planning_story_list_.storys[i];

    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        points = temp_data_.points_2d_list[0];
    common::Path& path = temp_data_.path_list[0];

    for (Int32_t j = 0; j < story.ref_line.point_num; ++j) {
      common::Vec2d* p = points.Allocate();
      if (Nullptr_t != p) {
        ConvertUtmPointToRelCoordinate(story.ref_line.points[j], p);
      }
    }

    common::Vec2d cur_pos(current_position.x, current_position.y);
    common::PathPoint path_point;
    story.is_in_range = false;
    if (path.Construct(points)) {
      if (path.points().Size() > 1) {
        if (path.FindProjection(cur_pos, &path_point)) {
          Float32_t left_width = 2.0F;
          Float32_t right_width = 2.0F;
          Float32_t start_s = -0.1F;
          Float32_t end_s = path.total_length() + 0.1F;
          if (story.condition.vaild) {
            if (story.condition.area.valid) {
              if (story.condition.area.left_width > 0.01F) {
                left_width = story.condition.area.left_width;
              }
              if (story.condition.area.right_width > 0.01F) {
                right_width = story.condition.area.right_width;
              }
              start_s = story.condition.area.start_s;
              if (story.condition.area.end_s > 0.01F) {
                end_s = story.condition.area.end_s;
              }
            }
          }
          if ((start_s < path_point.s) && (path_point.s < end_s)) {
            if (path_point.l < 0.0F) {
              if (common::com_abs(path_point.l) < right_width) {
                story.is_in_range = true;
              }
            } else {
              if (common::com_abs(path_point.l) < left_width) {
                story.is_in_range = true;
              }
            }
          }
        }
      }
    }
  }
}

void PlanningStorySet::ConvertUtmPointToRelCoordinate(
    const ad_msg::PlanningStoryRefLine::Point& point_in,
    common::Vec2d* point_out) {
  common::Matrix<Float64_t, 2, 1> point_conv;

  point_conv(0) = point_in.x;
  point_conv(1) = point_in.y;

  common::TransformVert_2D(mat_convert_utm_to_rel_coor_, &point_conv);

  point_out->set_x(static_cast<common::geo_var_t>(point_conv(0)));
  point_out->set_y(static_cast<common::geo_var_t>(point_conv(1)));
}


} // namespace driv_map
} // namespace phoenix

