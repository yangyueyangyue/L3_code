/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       ActionPlanningWrapper.h
 * @brief      行为规划
 * @details    行为规划模块的封装层
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

#ifndef PHOENIX_PLANNING_ACTION_PLANNING_WRAPPER_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_WRAPPER_H_

#include "motion_planning.h"
#include "driving_map_wrapper.h"

namespace phoenix {
namespace planning {


/// 具体实现行为规划功能的模块的前向声明
class ActionPlanningRuleBased;


/**
 * @struct VelocityPlanningWrapper
 * @brief 行为规划模块的封装层
 */
class ActionPlanningWrapper {
public:
  /**
   * @brief 构造函数
   */
  ActionPlanningWrapper();

  /**
   * @brief 析构函数
   */
  ~ActionPlanningWrapper();

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
  const ActionPlanningResult& GetResultOfPlanning() const;

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

private:
  // 具体实现行为规划功能的模块的实例的地址
  ActionPlanningRuleBased* action_planner_;
};


} // namespace phoenix
} // namespace planning


#endif // PHOENIX_PLANNING_ACTION_PLANNING_WRAPPER_H_

