/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       TrajectoryPlanningWrapper.h
 * @brief      轨迹规划
 * @details    轨迹规划模块的封装层
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

#ifndef PHOENIX_PLANNING_TRAJECTORY_PLANNING_WRAPPER_H_
#define PHOENIX_PLANNING_TRAJECTORY_PLANNING_WRAPPER_H_

#include "motion_planning.h"
#include "driving_map_wrapper.h"

namespace phoenix {
namespace planning {


/**
 * @brief 获取具体实现轨迹规划功能的模块的实例的SIZE
 * @return 具体实现轨迹规划功能的模块的实例的SIZE
 */
Uint32_t GetTrajectoryPlanningImplSize();


/// 具体实现轨迹规划功能的模块的前向声明
class TrajectoryPlanningStateLattice;


/**
 * @struct TrajectoryPlanningWrapper
 * @brief 轨迹规划模块的封装层
 */
class TrajectoryPlanningWrapper {
public:
  /**
   * @brief 构造函数
   */
  TrajectoryPlanningWrapper();

  /**
   * @brief 析构函数
   */
  ~TrajectoryPlanningWrapper();

  /**
   * @brief 配置模块
   */
  void Configurate(const TrajectoryPlanningConfig& conf);

  /**
   * @brief 规划局部轨迹
   * @return true - 成功，false - 失败
   */
  bool Plan(const TrajectoryPlanningDataSource& data_source);

  /**
   * @brief 获取规划的结果
   * @return 轨迹规划的结果
   */
  const TrajectoryPlanningResult& GetResultOfPlanning() const;

  /**
   * @brief 获取轨迹规划的内部信息（调试用）
   * @param[out] trj_planning_info 轨迹规划的内部信息
   */
  void GetTrajectoryPlanningInfo(
      TrajectoryPlanningInfo* trj_planning_info) const;

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

private:
  // 具体实现轨迹规划功能的模块的实例的地址
  TrajectoryPlanningStateLattice* trajectory_planner_;
};


} // namespace phoenix
} // namespace planning


#endif // PHOENIX_PLANNING_TRAJECTORY_PLANNING_WRAPPER_H_

