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

#ifndef PHOENIX_PLANNING_ACTION_PLANNING_PULLING_OVER_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_PULLING_OVER_H_

#include "action_planning/action_planning_base.h"


namespace phoenix {
namespace planning {


/// 应急停车，车辆自动靠边停车。
class ActionPlanningPullingOver : public ActionPlanningBase{
public:
  ActionPlanningPullingOver();
  ~ActionPlanningPullingOver(void);

private:
  void Handle(
      const ActionPlanningDataSource& data_source,
      ActionPlanningResult* planning_result);
};


}  // namespace planning
}  // namespace phoenix


#endif  // PHOENIX_PLANNING_ACTION_PLANNING_PULLING_OVER_H_
