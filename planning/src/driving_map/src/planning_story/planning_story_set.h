/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       scene_story_set.h
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

#ifndef PHOENIX_DRIVING_PLANNING_STORY_SET_H_
#define PHOENIX_DRIVING_PLANNING_STORY_SET_H_

#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "driving_map.h"


namespace phoenix {
namespace driv_map {


class PlanningStorySet{
public:
  PlanningStorySet();
  ~PlanningStorySet() {
    // nothing to do
  }

  void Clear();

  inline void set_mat_convert_utm_to_rel_coor(
      const common::Matrix<Float64_t, 3, 3>& mat) {
    mat_convert_utm_to_rel_coor_ = mat;
  }

  void UpdateStorys(
      const ad_msg::PlanningStoryList& story_list,
      const ad_msg::RelativePos& current_position);

  const ad_msg::PlanningStoryList& GetStoryList() const {
    return (planning_story_list_);
  }

private:
  void ConvertUtmPointToRelCoordinate(
      const ad_msg::PlanningStoryRefLine::Point& point_in,
      common::Vec2d* point_out);

private:
  // 场景任务列表
  ad_msg::PlanningStoryList planning_story_list_;

  // 将UTM坐标转换到相对坐标的转换矩阵
  common::Matrix<Float64_t, 3, 3> mat_convert_utm_to_rel_coor_;

  struct TemporaryData {
    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
        points_2d_list[1];
    common::Path path_list[1];
  } temp_data_;
};


} // namespace driv_map
} // namespace phoenix


#endif // PHOENIX_DRIVING_PLANNING_STORY_SET_H_

