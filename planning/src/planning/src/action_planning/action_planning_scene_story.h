/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       action_planning_base.h
 * @brief      行为规划基类
 * @details    行为规划的基类，规划车辆行为(变道、跟车、避让、巡航等)
 *
 * @author     sip (sip@goyu-ai.com)
 * @date       2021/05/07
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/05/07  <td>1.0      <td>sip       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_ACTION_PLANNING_SCENE_STORY_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_SCENE_STORY_H_

#include "action_planning/action_planning_base.h"


namespace phoenix {
namespace planning {


/// 根据场景任务来规划行为
class ActionPlanningSceneStory : public ActionPlanningBase{
public:
  ActionPlanningSceneStory();
  ~ActionPlanningSceneStory(void);

private:
  void Handle(
      const ActionPlanningDataSource& data_source,
      ActionPlanningResult* planning_result);

  bool IsSatisfyingStoryCondition(const ad_msg::SceneStory& story);
};


}  // namespace planning
}  // namespace phoenix


#endif  // PHOENIX_PLANNING_ACTION_PLANNING_SCENE_STORY_H_
