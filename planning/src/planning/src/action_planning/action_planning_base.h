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
 * <tr><td>2021/06/09  <td>2.0      <td>pengc     <td>修改类的结构，增加容错处理
 * </table>
 *
 ******************************************************************************/
 
#ifndef PHOENIX_PLANNING_ACTION_PLANNING_BASE_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_BASE_H_


#include "utils/com_timer.h"
#include "motion_planning.h"
#include "action_planning/action_planning_lane_risk_analyser.h"


namespace phoenix {
namespace planning {


/// 行为规划用的共享数据
struct ActionPlanningSharedData {
  enum {
    EVENT_TYPE_START_ROBOTIC,
    EVENT_TYPE_STOP_ROBOTIC,
    EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_LEFT,
    EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_RIGHT,
    EVENT_TYPE_START_CHANGING_LANE_LEFT,
    EVENT_TYPE_START_CHANGING_LANE_RIGHT,
    EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT,
    EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT,
    EVENT_TYPE_ABORT_CHANGING_LANE_LEFT,
    EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT,
    EVENT_TYPE_REFUSE_ABORT_CHANGING_LANE,
    EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT,
    EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT
  };

  enum {
    LANE_IDX_CURR = 0,
    LANE_IDX_LEFT_1,
    LANE_IDX_RIGHT_1,

    LANE_IDX_MAX
  };

  struct LaneInfo {
    bool valid;
    bool is_on_routing;
    ActionPlanningLaneRiskAnalyser::LaneRiskInfo risk_info;

    LaneInfo() {
      Clear();
    }

    void Clear() {
      valid = false;
      is_on_routing = false;
      risk_info.Clear();
    }
  };

  /// 配置信息
  ActionPlanningConfig config;
  /// 车身信息
  ad_msg::Chassis chassis;
  /// 车道风险信息
  LaneInfo lane_info_list[LANE_IDX_MAX];
  /// 定时器
  struct {
    common::ComTimer forbid_auto_changing_lane;
    common::ComTimer forbid_alc_changing_lane;
    common::ComTimer hold_changing_lane_req;
    common::ComTimer forbid_changing_lane_in_curve;
    common::ComTimer forbid_auto_fallback;
  } timers;
  common::ComTimerList<11> timer_list;
  /// 事件通知列表
  common::StaticVector<ad_msg::EventReporting, 4> event_reporting_list;

  ActionPlanningSharedData() {
    Clear();

    timer_list.AddTimer(&timers.forbid_auto_changing_lane);
    timer_list.AddTimer(&timers.forbid_alc_changing_lane);
    timer_list.AddTimer(&timers.hold_changing_lane_req);
    timer_list.AddTimer(&timers.forbid_changing_lane_in_curve);
    timer_list.AddTimer(&timers.forbid_auto_fallback);
  }

  void Clear() {
    event_reporting_list.Clear();
  }

  void UpdateTimerList() {
    timer_list.Update();
  }

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

  void AddEventReporting(Int32_t event_type, Int32_t param1 = 0);
};


class ActionPlanningBase {
public:
  ActionPlanningBase();

  virtual ~ActionPlanningBase();

  void set_sequence_id(Int32_t seq) {
    sequence_id_ = seq;
  }

  Int32_t sequence_id() const { return sequence_id_; }

  void set_next_action_planner(ActionPlanningBase* planner) {
    next_action_planner_ = planner;
  }

  void set_shared_data(ActionPlanningSharedData* data) {
    act_planning_shared_data_ = data;
  }

  void Proceed(
      const ActionPlanningDataSource& data_source,
      ActionPlanningResult* planning_result);

private:
  virtual void Handle(
      const ActionPlanningDataSource& data_source,
      ActionPlanningResult* planning_result) = 0;

protected:
  // 职责链中各个规划器的共享数据
  ActionPlanningSharedData* act_planning_shared_data_;

private:
  // 职责链中规划器执行的顺序号，下一个规划器的执行顺序号必须是当前的顺序号加1
  Int32_t sequence_id_;
  // 职责链中下一个规划器的地址
  // !!!注意：此变量不允许派生类访问
  ActionPlanningBase* next_action_planner_;
};


}  // namespace planning
}  // namespace phoenix


#endif  // PHOENIX_PLANNING_ACTION_PLANNING_BASE_H_

