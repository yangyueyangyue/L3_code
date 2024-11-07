/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       action_planning_rule_based.h
 * @brief      行为规划（Rule Based）
 * @details    使用基于规则的方法，规划车辆行为(变道、跟车、避让、巡航等)
 *
 * @author     pengc
 * @date       2020.07.16
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/07/16  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_ACTION_PLANNING_RULE_BASED_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_RULE_BASED_H_

#include "motion_planning.h"
#include "driving_map_wrapper.h"

#include "action_planning/action_planning_acc_following.h"
#include "action_planning/action_planning_changing_lane.h"
#include "action_planning/action_planning_emergency_stopping.h"
#include "action_planning/action_planning_lane_keeping.h"
#include "action_planning/action_planning_pulling_over.h"
#include "action_planning/action_planning_bypassing.h"
#include "action_planning/action_planning_scene_story.h"
#include "action_planning/action_planning_fallback.h"

namespace phoenix {
namespace planning {


/**
 * @struct ActionPlanningRuleBased
 * @brief 行为规划（Rule Based）
 */
class ActionPlanningRuleBased {
public:
  /**
   * @brief 构造函数
   */
  ActionPlanningRuleBased();

  /**
   * @brief 析构函数
   */
  ~ActionPlanningRuleBased();

  /**
   * @brief 配置模块
   */
  void Configurate(const ActionPlanningConfig& conf);

  /**
   * @brief 规划车辆行为(变道、跟车、避让、巡航等)
   * @return true - 成功，false - 失败
   */
  bool Plan(const ActionPlanningDataSource& data_source);

  /**
   * @brief 获取规划的结果
   * @return 规划的结果
   */
  inline const ActionPlanningResult& GetResultOfPlanning() const {
    return (result_of_planning_);
  }

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

private:
  void UpdateLaneInfo(const ActionPlanningDataSource &data_source);

private:
  ActionPlanningLaneKeeping planner_lane_keeping_;
  ActionPlanningChangingLane planner_changing_lane_;
  ActionPlanningBypassing planner_bypassing_;
  ActionPlanningAccFollowing planner_acc_following_;
  ActionPlanningPullingOver planner_pulling_over_;
  ActionPlanningEmergencyStopping planner_emergency_stopping_;
  ActionPlanningSceneStory planner_scene_story_;
  ActionPlanningFallback planner_fallback_;

  ActionPlanningBase* action_planner_;

  ActionPlanningSharedData act_planning_shared_data_;

  ActionPlanningLaneRiskAnalyser lane_risk_analyser_;

  // 规划的结果
  ActionPlanningResult result_of_planning_;

  // 界面启动按钮上一个状态
  bool adas_start_status_pre;
};


} // namespace planning
} // namespace phoenix


#endif // PHOENIX_PLANNING_ACTION_PLANNING_RULE_BASED_H_
