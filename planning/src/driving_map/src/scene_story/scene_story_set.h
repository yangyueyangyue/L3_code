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

#ifndef PHOENIX_DRIVING_MAP_SCENE_STORY_SET_H_
#define PHOENIX_DRIVING_MAP_SCENE_STORY_SET_H_

#include"driving_map.h"
#include "map_space/hd_map.h"


namespace phoenix {
namespace driv_map {


class SceneStorySet{
public:
  SceneStorySet();
  ~SceneStorySet() {
    // nothing to do
  }

  void Clear();

  inline void set_mat_convert_utm_to_rel_coor(
      const common::Matrix<Float64_t, 3, 3>& mat) {
    mat_convert_utm_to_rel_coor_ = mat;
  }

  void UpdateSceneStorys(
      Int64_t timestamp, const ad_msg::SceneStoryList* story_list,
      const common::Path& ref_path, const common::PathPoint& veh_on_ref_path);

  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& GetSceneStoryList() const {
    return (scene_story_list_);
  }

private:
  void ConvertUtmPointToRelCoordinate(
      const ad_msg::SceneStoryControlLine::Point2D& point_in,
      ad_msg::SceneStoryControlLine::Point2D* point_out);

private:
  // 场景任务列表(原数据)
  ad_msg::SceneStoryList raw_scene_story_list_;
  // 场景任务列表
  common::StaticVector<ad_msg::SceneStory,
    ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> scene_story_list_;

  // 将UTM坐标转换到相对坐标的转换矩阵
  common::Matrix<Float64_t, 3, 3> mat_convert_utm_to_rel_coor_;
};


} // namespace driv_map
} // namespace phoenix


#endif // PHOENIX_DRIVING_MAP_SCENE_STORY_SET_H_

