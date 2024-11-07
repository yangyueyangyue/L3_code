/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       VelocityPlanningWrapper.h
 * @brief      速度规划
 * @details    速度规划模块的封装层
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

#ifndef PHOENIX_PLANNING_VELOCITY_PLANNING_WRAPPER_H_
#define PHOENIX_PLANNING_VELOCITY_PLANNING_WRAPPER_H_

#include "motion_planning.h"
#include "driving_map_wrapper.h"

namespace phoenix {
namespace planning {

/// 具体实现速度规划功能的模块的前向声明
#ifdef ENABLE_PACC
class VelocityPlanningPolynomialFitting;
#else
class VelocityPlanningFuzzyPID;
#endif

/**
 * @struct VelocityPlanningWrapper
 * @brief 速度规划模块的封装层
 */
class VelocityPlanningWrapper {
public:
  /**
   * @brief 构造函数
   */
  VelocityPlanningWrapper();

  /**
   * @brief 析构函数
   */
  ~VelocityPlanningWrapper();

  /**
   * @brief 配置模块
   */
  void Configurate(const VelocityPlanningConfig& conf);

  /**
   * @brief 规划目标速度及目标加速度
   * @return true - 成功，false - 失败
   */
  bool Plan(const VelocityPlanningDataSource& data_source);

  /**
   * @brief 获取规划的结果
   * @return 规划的结果
   */
  const VelocityPlanningResult& GetResultOfPlanning() const;

#if (ENABLE_BACK_TO_BACK_TEST)
  const VelocityPlanningResult& GetResultOfPlanningOrg() const;
#endif

  /**
   * @brief 获取速度规划内部数据
   * 
   */
  const VelocityPlanningInternal& GetInternalOfPlanningDebug() const;

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

private:
  // 具体实现速度规划功能的模块的实例的地址
  //TODO: Desgin a base class ?
#ifdef ENABLE_PACC
VelocityPlanningPolynomialFitting* velocity_planner_;
#else
VelocityPlanningFuzzyPID* velocity_planner_;
#endif
};


} // namespace phoenix
} // namespace planning


#endif // PHOENIX_PLANNING_VELOCITY_PLANNING_WRAPPER_H_

