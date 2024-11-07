/** @copyright Copyright (c) 2018-2023 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       velocity_planning_polynomial_fitting.h
 * @brief      使用多项式拟合的速度及加速度规划, 基于velocity_planning_fuzzy_pid代码重构
 * @details    速度规划功能的实现根据 场景 划分： 道路约束(弯道/匝道，坡道)，交通规则(红绿灯，人行横道...)，交通流(ACC：高速跟车，低速跟车，跟车停止，跟车起步； 避障：避障减速，避障停车； AEB：紧急停车)
 *             这些场景逻辑上可以归为三大类：无障碍物，虚拟障碍物，真实障碍物
 *             1. 无障碍物(road rules)
 *             1.1. 驾驶员设定限速 ： 使用者设定未来一段时间的目标行驶速度。通过一段时间的加速/减速，稳定在此速度附近，主要针对平直道路，对弯道，坡道需要额外处理，速度可在此目标速度上下浮动
 *             1.2. 曲率(curve) ： 通过一段弯道，或者变道(LCA)时，为安全考虑，需要对这段行驶路径限速
 *             1.3. 坡道(slope) ： 考虑到上坡动力不足，下坡刹车片过热，坡道行驶限速范围相比平路要更有弹性
 *             1.4. 道路限速 ： 因为道路条件限制导致的一段长路径的限速，数值来自限速标志(卡车)，地图限速(上限及下限)，如果减速不及时会出现超速情况
 *             
 *             2. 障碍物
 *             相对无障碍物约束场景，障碍物具有几何(长宽高，航向，距离)和运动(横纵向速度及加速度)属性
 *             2.1. 虚拟障碍物(traffic rules)
 *                根据交通规则约束的场景细分： 隧道(tunnel, 地图可提前告知距离), 匝道(ramp, 地图可提前告知距离), 桥梁，人行横道(crossing)，减速带，交通灯(traffic light)...
 *             2.2. 真实障碍物
 *                根据场景细分：
 *             2.2.1. ACC(following) : 跟车加速，跟车减速，稳定跟车，Stop&Go, Cut In, Cut out
 *             2.2.2. 避障减速(avoidancing) : 可能有碰撞风险的障碍物： 前方静止障碍物，旁车道贴线车辆，快速Cut in后Cut out车辆
 *             2.2.3. AEB(emergency brake) : 鬼探头(横穿移动物体)，前方车辆紧急制动，前方车辆紧急转向后前前方车突然出现，旁车近距离Cut In
 * 
 *             3. ODD Fallback : 场景之外，本车道减速到停(驾驶员不接管)
 *             
 *             4. PACC : 在上述功能上 新增 基于ADASIS地图的经济性最优约束条件下的速度规划 @see velocity_planning_pacc.h
 * @authors    pengc, xiarf, zhangq, wangwh, qiuzw
 * @date       2023.04.24
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_VELOCITY_PLANNING_POLYNOMIAL_FITTING_H_
#define PHOENIX_PLANNING_VELOCITY_PLANNING_POLYNOMIAL_FITTING_H_

#include "driving_map_wrapper.h"
#include "math/math_utils.h"
#include "math/matrix.h"
#include "motion_planning.h"
#include "utils/linear_interpolation.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "curve/quintic_polynomial_curve1d.h"
#include "utils/macros.h"
#include "velocity_planning_pacc.h"
#include "pcc_map/adasis_v2.h"

namespace phoenix {
namespace planning {
/**
 * @class VelocityPlanningPolynomialFitting
 * @brief 根据不同场景计算目标速度和加速度, 使用多项式拟合平滑计算出下一个周期控制模块需要跟随的目标速度
 */
class VelocityPlanningPolynomialFitting {
/**
 * 速度规划API
 */
public:
  VelocityPlanningPolynomialFitting();

  ~VelocityPlanningPolynomialFitting();

  /**
   * @brief 设定默认的参数配置
   * @details 1：设置功能使能选项；2：设置车辆尺寸及定位安装位置参数；3：不同场景下速度及相对速度限制值
   */
  void SetDefaultParam();

  /**
   * @brief 更新车辆参数配置
   */

  void UpdateVehicleParameter();
  /**
   * @brief 加载planning_config文件内的信息进行参数配置
   * @details 1：配置信息管理在xml文件中；2：对设置的参数进行最小值限制
   */
  void Configurate(const VelocityPlanningConfig& conf);

  /**
   * @brief 调用Clear（）函数进行信息的清除
   * @details 1：清除目标防抖的统计处理信息；2：清除平滑信息；3：清除输出的目标速度及加速度array内的信息
   */
  void ClearHoldInfo();

  /**
   * @brief Main Function, 规划目标车速及目标加速度
   * @return true - 成功，false - 失败
   */
  bool Plan(const VelocityPlanningDataSource& data_source);

  /**
   * @brief 获取规划的结果
   * @return 规划的结果
   */
  inline const VelocityPlanningResult& GetResultOfPlanning() const {
      return (result_of_planning_);
  }

  inline const VelocityPlanningInternal& GetInternalOfPlanningDebug() const {
      return (planning_debug_);
  }

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(Int32_t num, ad_msg::EventReporting *const events) const;
  
  bool GetRoadEventReporting() const;
/**
 * 模块配置和算法参数
 */
private:

  struct ConsideringObstaclesDebug {
    ObstacleDecision cipv;
    // 跟车场景，计算的速度规划类型
    Int8_t tar_type_following;
    // 跟车场景，计算的目标速度，m/s
    Float32_t tar_v_following;
    // 跟车场景，计算的目标加速度，m/s^2
    Float32_t tar_a_following;
    // 跟车场景，五次多项式补丁状态
    Int8_t fsm_follow_state;

    // 速度规划：避障
    // 避障目标的所有相关数据
    ObstacleDecision avoidance;
    // 避障场景，计算的速度规划类型
    Int8_t tar_type_obstacle;
    // 避障场景，计算的目标速度，m/s
    Float32_t tar_v_obstacle;
    // 避障场景，计算的目标加速度，m/s^2
    Float32_t tar_a_obstacle;
    // 避障场景，五次多项式补丁状态
    Int8_t fsm_obstacle_state;

    // 速度规划：紧急制动 AEB
    // 紧急制动目标的所有相关数据
    ObstacleDecision mio; // 最可能碰撞的障碍物
    //collision_test ct // 碰撞检测结果

    //Float32_t ttc;

    // 紧急制动场景，计算的速度规划类型
    Int8_t tar_type_aeb;
    // 紧急制动场景，计算的目标速度，m/s
    Float32_t tar_v_aeb;
    // 紧急制动场景，计算的目标加速度，m/s^2
    Float32_t tar_a_aeb;

    void Clear(){
      cipv.Clear();
      tar_type_following = 0;
      tar_v_following = 0.0F;
      tar_a_following = 0.0F;
      fsm_follow_state = 0;

      avoidance.Clear();
      tar_type_obstacle = 0;
      tar_v_obstacle = 0.0F;
      tar_a_obstacle = 0.0F;
      fsm_obstacle_state = 0;

      mio.Clear(); // 最可能碰撞的障碍物
      tar_type_aeb = 0;
      tar_v_aeb = 0.0F;
      tar_a_aeb = 0.0F;

    }

    ConsideringObstaclesDebug () {
      Clear();
    }
  };
  
  struct {
    bool enable_acc;
    bool enable_aeb;
    bool enable_aeb_pre_dec;
    bool enable_following;
    bool enable_low_speed_following;
    bool enable_pacc;

    plan_var_t tar_v;
    plan_var_t tar_a;
    plan_var_t time_gap_setting;

    plan_var_t lateral_acceleration_limit;
    bool       enable_stop_by_traffic_light;
  } settings_;

  struct {
    /// 车长，单位：米
    plan_var_t vehicle_length;

    /// 车宽，单位：米
    plan_var_t vehicle_width;

    /// 车辆的定位点到 front of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_front;

    /// 车辆的定位点到 rear of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_rear;

    /// 车辆的定位点到中心点的距离，单位：米
    plan_var_t dist_of_localization_to_center;
    
    /// 车辆外接圆半径，单位：米
    plan_var_t vehicle_circumradius;

    /// 速度规划全局范围的最大速度限制
    plan_var_t max_velocity_limit;

    /// 速度规划全局范围的最大纵向加速度限制
    plan_var_t max_acceleration_limit;

    /// 速度规划全局范围的最大纵向减速度限制(负值)
    plan_var_t max_deceleration_limit;

    plan_var_t tunnel_velocity_limit;

    plan_var_t ramp_velocity_limit;

    /// 风险感知
    plan_var_t risk_perception;

    /// 速度规划起步目标车速 和 加速度
    plan_var_t go_target_velocity;
    plan_var_t go_target_acc;

    /// AEB 减速度
    plan_var_t aeb_target_acc_I;
    plan_var_t aeb_target_acc_II;
    plan_var_t aeb_target_detla_v;

    /// 障碍物的加速度 和减速度限制 
    plan_var_t obsacle_final_dec_ax_limit;
    plan_var_t obsacle_final_acc_ax_limit;
    /// 停车场景速度限制列表
    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> 
        velocity_limit_table_for_stop_situation;

    /// 精准停车场景速度限制列表
    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
        velocity_limit_table_for_stop_accurately_situation;

    struct StopAccurately {
      bool       enable;
      plan_var_t braking_distance;
      plan_var_t proposed_decelaration;

      StopAccurately() {
        enable                = false;
        braking_distance      = 0.8F;
        proposed_decelaration = 3.0F;
      }
    } stop_accurately;    //精准停车struct定义

    plan_var_t sample_step_len_for_collision_test;

    /// 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
        relative_velocity_limit_table_by_dist_to_static_obj;

    /// 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
        relative_velocity_limit_table_by_dist_to_dynamic_obj;

    /// AEB启动TTC时间
    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> aeb_action_ttc_table;

    common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> curvatrue_limit_velocity_table;

    plan_var_t safe_distance_to_static_obstacle;
    plan_var_t safe_distance_for_low_speed_following;
    plan_var_t safe_distance_to_dynamic_obstacle;
    plan_var_t safe_time_to_dynamic_obstacle;
    plan_var_t safe_distance_in_backing_mode;

    /// paramters for suppressing the jitter caused by the noise of sensors
    Int32_t jitter_suppressing_acce_delay_time;
    Int32_t jitter_suppressing_dece_delay_time;
    Int32_t jitter_suppressing_acce_req_counter_threshold;
    Int32_t jitter_suppressing_dece_req_counter_threshold;
  } param_;

/**
 * Input Data
 */
private:
  /// Environment
  Int64_t                 curr_timestamp_;

  Int32_t                 driving_direction_;

  /// 驾驶地图信息
  const driv_map::DrivingMapWrapper *driving_map_;

  /// 行为规划的结果
  ActionPlanningResult result_of_action_planning_;

  /// 轨迹规划的结果
  TrajectoryPlanningResult result_of_trajectory_planning_;

  /// 车身信息
  ad_msg::Chassis chassis_status_;

  /// 特殊的底盘信息
  ad_msg::SpecialChassisInfo special_chassis_status_;

  /// 特定型号车辆的信息
  ad_msg::ChassisCtlCmd chassis_ctl_cmd_;

  /// 红绿灯信息
  ad_msg::TrafficLightList traffic_light_list_;

  /// 
  ad_msg::TrafficSignalList traffic_signal_list_;

/**
 * Output Data
*/
private:
  // 规划的结果
  VelocityPlanningResult result_of_planning_;
  VelocityPlanningInternal planning_debug_;

  enum
  {
      EVENT_TYPE_AEB_WARNING,
      EVENT_TYPE_AEB_ACTION,
      EVENT_TYPE_UNCERTAIN_OBSTACLE,
      EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL,
      EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP
  };
  // Event report
  void AddEventReporting(Int32_t event_type);

  common::StaticVector<ad_msg::EventReporting, 4> event_reporting_list_;

/**
 * 自车航迹推算及规划路径空间同步
 */
private:
  common::TrajectoryPoint curr_position_;
  /// 规划用的当前车速（此车速必须大于0,否则将产生除0错误）
  plan_var_t   veh_velocity_for_planning_;
  common::Path target_trajectory_;
  common::Path changed_trajectory_;
  common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum>
                    target_trajectory_curvature_info_;
  common::PathPoint veh_proj_on_major_ref_line_;
  common::PathPoint veh_proj_on_target_trajectory_;

/**
 * 多项式拟合的速度规划，以及相关补丁
*/
private:

  /**
   * @brief 多项式拟合设定参数，尽可能泛化，以利于调用者根据不同场景设定不同的参数
   * 
   */
  struct ParamOfPolynomialFitting {
    bool is_cc;
    /// 速度规划时长
    plan_var_t tf;

    /// 安全距离
    plan_var_t safe_distance;

    /// 根据场景约束加速度最大值
    plan_var_t ax_max;

    /// 根据场景约束减速度最大值
    plan_var_t ax_min;

    plan_var_t replan_limit_acc;

    plan_var_t replan_limit_dec;

    plan_var_t s_cost;

    plan_var_t v_cost;

    plan_var_t jerk_cost;

    plan_var_t preview_time;


    bool polynomial_order_is_quintic;

    ParamOfPolynomialFitting() {
      is_cc = true;
      polynomial_order_is_quintic = true;
      tf = 3.0;
      safe_distance = 0.0;
      ax_max = 0.5;
      ax_min = -0.2;
      replan_limit_acc = 1.0;
      replan_limit_dec = 2.0;
      
      s_cost = 0.2;
      v_cost = 0.4;
      jerk_cost = 0.4;
      preview_time = 0.2;
    }
  };

  /**
   * @brief 多项式拟合的初始和结束状态
   * 
   */
  struct StateOfPolynomial {
    plan_var_t initial_state_vx;
    plan_var_t initial_state_ax;

    plan_var_t final_state_vx;
    plan_var_t final_state_distance;
    
    StateOfPolynomial() {
      initial_state_vx = 0.0F;
      initial_state_ax = 0.0F;
      final_state_vx = 0.0F;
      final_state_distance = 0.0F;
    }
  };

  /**
   * @brief 多项式拟合输出结果
   * 
   */
  struct PolynomialFittingOutput {
    plan_var_t tar_s;
    plan_var_t tar_v;
    plan_var_t tar_a;

    plan_var_t s_sample_start;
    plan_var_t s_sample_end;

    plan_var_t v_sample_start;
    plan_var_t v_sample_end;

    plan_var_t min_cost;
    plan_var_t optimal_s;
    plan_var_t optimal_v;

    plan_var_t c5;
    plan_var_t c4;
    plan_var_t c3;
    plan_var_t c2;
    plan_var_t c1;
    plan_var_t c0;
    
    PolynomialFittingOutput() {
      tar_s = 0.0F;
      tar_v = 0.0F;
      tar_a = 0.0F;
      s_sample_start = 0.0F;
      s_sample_end = 0.0F;

      v_sample_start = 0.0F;
      v_sample_end = 0.0F;
      min_cost = 0.0F;
      optimal_s = 0.0F;
      optimal_v = 0.0F;

    }
  };

  /**
   * @brief 基于Cost筛选出最优的状态，在根据五次多项式 计算出 一条速度曲线
   *        然后选取一个预瞄时间点计算出目标速度和加速度
   *        基于V-S Graph采样点拟合多项式，根据多项式计算cost最小的曲线作为最优V-S曲线
   * @param param 参数配置
   * @param state 多项式的起始和末状态
   * @param tar_v 目标的车速
   * @param tar_a 目标的加速度
   */
  void CalcTarVelocityPolynomial(const ParamOfPolynomialFitting &param, const StateOfPolynomial &state, PolynomialFittingOutput& output);

  void  linspace(Float32_t x1, Float32_t x2, int n, Float32_t* y);

  void CubicComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t x1,   const Float32_t dx1, const Float32_t p, Float32_t* coef3_);
  void CubicPolynomialCurve1d(const Float32_t* coef3_, const Int32_t order, const Float32_t p, Float32_t* Cubic);

  void QuinticComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t ddx0, const Float32_t x1, const Float32_t dx1, const Float32_t ddx1, const Float32_t p, Float32_t* coef5_);
  void QuinticPolynomialCurve1d(const Float32_t* coef5_, const Int32_t order, const Float32_t p, Float32_t* Quintic);
  
  /**
   * @brief [A] 根据当前的车辆状态(载荷，档位等)，环境因素(附着系数，坡度等)
   *            更改车辆的加速度上下限 ax_max , ax_min
   *            [1] 根据当前的车辆状态 调用 UpdatePolynomialParamByVehicleDynamic；

    *        [B] 根据当前的车辆状态(载荷，档位等)，环境因素(附着系数，坡度等)
    *            重新设定 重规划的上下限值 replan_limit_acc ， replan_limit_dec
    *        [C] 根据其他车辆决定 减速上限
    * @param is_cc 是否是巡航状态
    * @param ego_v 自车车速
    * @param obj_distance 障碍物的距离 在有障碍物的时候为SL坐标系下的S值 
    *                                在无障碍物的时候为 -1.0
    * @param ax_max 最大加速度上限
    * @param ax_min 最小减速度上限
    * @param replan_limit_acc 重新规划上限值
    * @param replan_limit_dec 重新规划下限值
    */
  void UpdatePolynomialParamAccMaxMinByCurrentVehicleState(const bool &is_cc, const plan_var_t &ego_v, const plan_var_t &obj_distance, plan_var_t obj_v, 
                                                    plan_var_t *ax_max, plan_var_t *ax_min, plan_var_t *safe_distance,
                                                    plan_var_t *replan_limit_acc, plan_var_t* replan_limit_dec);

  /**
   * @brief 当前的车辆状态更改车辆的加速度上下限，当前主要根据档位，限制最大加速度
   *        Note : 人工驾驶的时候 60km/h --> 90km/h 的车速的时候
   *               在坡度为 2% 附近 平均加速度 0.08 m/s^2 ---> 60%  油门 --> 11 档  换挡之后 11 to 12 会有 -0.1 的掉速
   *               在坡度为 1.24% 附近 平均加速度 0.25 m/s^2 ---> 100% 油门 --> 11 档
   *               在坡度为 1.2% ~ 0.2% 附近 平均加速度 0.3m/s^2   ---> 100% 油门 --> 10 档
   *               需根据不同档位值进行标定 对应的加速值 随着档 上升 -> 加速度
   *               20230617 在 茶陵 -> 佛山 路段 对 加速度 进行标定
   *               [A] 60%  油门(实际：有坡度 但是 未猜到坡度值)
   *                     -> 10档 0.35m/s^2   近似  0.4m/s^2
   *                     -> 11档 0.269m/s^2  近似  0.3m/s^2
   *                     -> 12档 0.18 m/s^2  近似  0.2m/s^2
   * 
   *               [B] 100% 油门(实际：有坡度 但是 未猜到坡度值) 
   *                     -> 10档 0.35m/s^2  近似 0.4m/s^2
   *                     -> 11档 0.35m/s^2  近似 0.4m/s^2
   *                     -> 12档 0.35m/s^2  近似 0.3m/s^2
  */
  void UpdatePolynomialParamAccAccordingToGear(plan_var_t *ax_max_limit);


  //根据纵向边界表，限制最大减速度
  void UpdatePolynomialParamDecAccordingToTable(const plan_var_t &ego_v, const plan_var_t &obj_distance, plan_var_t obj_v, plan_var_t safe_distance, plan_var_t *ax_min_limit);
  
  /**
   * @brief 根据 当前时刻的车速状态 与 上一时刻的下发的目标的车速 进行比较
   *        从而 来决定 以当前时刻状态(速度，加速度 ，位置) 为 多项式计算的起点
   *        还是 以上一时刻规划出的结果为(速度 ，加速度 ，位置) 为 多项式计算的起点
   *        
   *        注： 这里的位置 始终为 0，是否是对的？？ 以当前时刻为起点 为 0 合理 ，以上一时刻为起点 为 0 是否合理？？
   * 
   * @param ax_max 最大加速度上限
   * @param ax_min 最小减速度上限
   * @param replan_limit_acc 重新规划上限值
   * @param replan_limit_dec 重新规划下限值
   */
  void UpdatePolynomialInitState(const ParamOfPolynomialFitting &param,
                                                  StateOfPolynomial &Polynomial_input);

  /**
   * @brief 根据 目标的车速 是否需要减速，当需要减的时候
   *        目标车速 与 当前车速相差太大的时候 是需要 对 目标车速 进行
   *        二次加工，目的：与当前状态 相差不要太大，减速不要太猛
   * 
   * @param target_vel 目标的车速
   * @param Polynomial_output 多项式的末状态
   */
  void UpdatePolynomialFinalStateForCC(const plan_var_t &target_vel, StateOfPolynomial &Polynomial_output);

/**
 * 无障碍物场景的速度规划，相比CC考虑更多场景
*/
private:
  enum {
      DRIVING_DIRECTION_FORWARD = 0,
      DRIVING_DIRECTION_BACKWARD
  };

  //
  struct TargetVelocityUpperLimit { 
    bool valid;
    Int32_t tar_type;
    plan_var_t tar_v;

    void Clear() {
      tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
      tar_v = 0.0F;
      valid = false;
    }

    TargetVelocityUpperLimit() {
      Clear();
    }
  };

  /**
   * @brief 每个场景计算的速度规划结果
   * 
   */
  struct TargetVelocityInfo {
    Int32_t                    tar_type;
    plan_var_t                 tar_v;
    plan_var_t                 tar_a;
    plan_var_t                 tar_s;
    bool release_throttle;
    plan_var_t tar_throttle;

    phoenix::common::PathPoint tar_pos;

    struct ObstacleInfo {
      bool valid;

      plan_var_t dist_to_obj;
      plan_var_t time_gap;
      plan_var_t relative_v;
      plan_var_t ttc;
      plan_var_t obj_v;

      Int32_t dist_gap_level;
      Int32_t rel_spd_level;

      plan_var_t velocity_limit;
      plan_var_t safe_dist;
      Int8_t fsm_state;
      void Clear() {
        valid       = false;
        dist_to_obj = 0.0F;
        time_gap    = 0.0F;
        relative_v  = 0.0F;
        ttc         = 100.0F;
        obj_v       = 0.0F;

        dist_gap_level = -1;
        rel_spd_level  = -1;

        velocity_limit = 0.0F;
        safe_dist      = 0.0F;
        fsm_state = 0;
      }

      ObstacleInfo() {
        Clear();
      }
    } obstacle_info;

    plan_var_t tar_v_true;
    plan_var_t tar_a_true;

    void Clear() {
      tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
      tar_v    = 0.0F;
      tar_a    = 0.0F;
      tar_s    = 0.0F;
      release_throttle = false;
      tar_throttle = 0.0;
      tar_pos.Clear();
      obstacle_info.Clear();
      tar_v_true = 0.0F;
      tar_a_true = 0.0F;
    }

    TargetVelocityInfo() {
      Clear();
    }
  };

  common::StaticVector<TargetVelocityInfo, 20> tar_v_info_list_;

  common::StaticVector<TargetVelocityUpperLimit, 20> tar_v_upper_limit_list_;

  common::StaticVector<TargetVelocityInfo, 10> tar_v_obstacle_list_;

  common::StaticVector<TargetVelocityInfo, 20> tar_v_virtual_obstacle_list_;

  plan_var_t cc_upper_limit_v_;

  plan_var_t pre_time_gap_setting_ = 3.0F;

  /**
   * @brief 用于道路约束场景的速度规划，主要是根据道路情况计算巡航速度的上限
   *        (就是不要求车速到达特定位置的时候车速为某个数值，这种情况在减速的时候可能仅仅需要
   *         松油门即可)，松油门的动作可以通过 标定不同载重 不同变速箱的减速度实现，或者给控制发一个状态
   *         让控制自己做。
   *         场景包括 ： 用户设定的巡航限速 、匝道内 、隧道内 、道路限速 、弯道内 、特殊场景(上下坡、变道不加速)
   *         根据 2023.06.15 ~ 2023.06.21 湖南段出差 YD的变速箱 + 32吨的载重 一半怠速的减速为 -0.2m/s^2
   *        // TODO: 根据坡度计算速度限制
   * @param result 五次多项式计算出的目标的加速，目标速度
   * @param cc_vel_road_rules 最終取小的巡航车速 
   */

  // A. 根据道路约束计算巡航速度上限
  void CalcTargetVelocityAccordingToRoadRules(TargetVelocityUpperLimit *cc_vel_road_rules);
  
  void PlanVelocityAccordingToRoadRules(const TargetVelocityUpperLimit &cc_vel_road_rules, TargetVelocityInfo *result);

  bool VelocityPlanningIsGoState(const float &target_v);

  void ConsiderSlopeOnPruitCC(VelocityPlanningResult *resuit_vel_plan);

  //A1. 用户设定CC车速
  void CalculateCruiseVelocityAccordingToUserSetting(TargetVelocityUpperLimit *target_vel_result);

  //A2. 道路限速CC车速
  void CalculateCruiseVelocityAccordingToRoadLimit(TargetVelocityUpperLimit *target_vel_result);

  //A3. 隧道内限速CC车速
  void CalculateCruiseVelocityAccordingToTunnelLimit(TargetVelocityUpperLimit *target_vel_result);

  //A4. 匝道内限速CC车速
  void CalculateCruiseVelocityAccordingToRampLimit(TargetVelocityUpperLimit *target_vel_result);

  //A5. 下一帧曲率限速CC车速
  void CalculateCruiseVelocityAccordingToCurvatureLimit(TargetVelocityUpperLimit *target_vel_result);

  //A6. 变道场景CC车速
  void CalculateCruiseVelocityAccordingToLaneChange(TargetVelocityUpperLimit *target_vel_result);

  enum {
      CC_ACTION_INVALID = 0,
      CC_ACTION_HOLD,
      CC_ACTION_NEED_ACC,
      CC_ACTION_ACC,
      CC_ACTION_NEED_DEC,
      CC_ACTION_DEC,
      CC_ACTION_ACC_TO_DEC,
      CC_ACTION_DEC_TO_ACC
  };
  
  int CCActionJudge(plan_var_t target_cc_vel);


private:
  
  //FIXME B. 虚拟障碍物virtual_obs场景的速度规划，主要考虑交规约束场景
  void PlanVelocityAccordingToTrafficRules(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo *result);

  //B1. 虚拟障碍物-交通规则（交通灯）
  void PlanVelocityAccordingToTrafficLight(const common::Path &path, const common::PathPoint &proj_on_path,
                                            TargetVelocityInfo *result);

  //B2. 虚拟障碍物-交通规则（交通标志）
  void PlanVelocityAccordingToTrafficSignal(const common::Path &path, const common::PathPoint &proj_on_path,
                                            TargetVelocityInfo *result);

  //B3. 虚拟障碍物-交通规则（交通场景）
  void PlanVelocityAccordingToSceneStory(const common::Path &path, const common::PathPoint &proj_on_path,
                                          TargetVelocityInfo *result);

  //B4. 虚拟障碍物-交通规则（接近隧道）
  void PlanVelocityAccordingToCloseToTunnel(const common::Path &path, const common::PathPoint &proj_on_path,
                                          TargetVelocityInfo *result);

  //B5. 虚拟障碍物-交通规则（接近匝道）
  void PlanVelocityAccordingToCloseToRamp(const common::Path &path, const common::PathPoint &proj_on_path,
                                          TargetVelocityInfo *result);


  //FIXME C. 虚拟障碍物（弯道曲率）
  void PlanVelocityAccordingToPathCurvature(
      const common::Path &                                                                       path,
      const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
      const common::PathPoint &proj_on_path, plan_var_t range_start_s, plan_var_t range_end_s,
      TargetVelocityInfo *result);

private:
  

  struct ClassifyTargetByObstacleResult {
    bool is_uncertain;
    bool aeb_warning;
    bool aeb_action;
    bool req_dec;

    TargetVelocityInfo tar_velocity_info;

    void Clear() {
      is_uncertain = false;
      aeb_warning  = false;
      aeb_action   = false;
      req_dec      = false;
      tar_velocity_info.Clear();
    }

    ClassifyTargetByObstacleResult() {
      Clear();
    }
  };
  /// 驾驶地图的障碍物个数
  Int32_t obstacle_driving_map_num_;
  /// 用于速度规划决策的障碍物数量
  Int32_t obstacle_decision_num_;
  /// 与DrivingMap中障碍物列表保持一致
  common::StaticVector<ObstacleDecision, ad_msg::ObstacleList::MAX_OBSTACLE_NUM> obstacle_decision_list_;

    /**
     * @brief 根据主参考线轨迹、变道轨迹 和 障碍物 运算出 目标的车速 和 加速度
     * 
     * Note : 原来的代码，将变道轨迹遍历障碍物 决策出的 目标车速 和 目标加速速    \ 取大
     *                  将主参考轨迹遍历障碍物 决策出的 目标车速 和 目标加速速  /   
     *                  tar_v_info.tar_v = common::Max(tar_v_info.tar_v, corrected_tar_v_info.tar_v);
     *                  tar_v_info.tar_a = common::Max(tar_v_info.tar_a, corrected_tar_v_info.tar_a);
     * 
     *                  存在 在 遍历 主参考线的时候 做出跟加的情况
     *                       在 遍历 变道轨迹的时候 做出跟减的情况  在变道的情况会出现 装车
     *               
     *                          |++++++++|  |++++++++|  |++++++++|
     *                          |        |  |        |  |        |
     *                          |position|  |position|  |position|
     *                          | front  |  | front  |  | front  |
     *                          | left   |  |        |  | right  |
     *                          |++++++++|  |++++++++|  |++++++++|
     *                               ^           ^
     *                               |           |
     *                     变道轨迹。  |           |
     *                                 +         |
     *                                    +      |   主参考线
     *                                       +   |
     *                                          +|
     *                                      |++++++++| 
     *                                      |        | 
     *                                      |  ego   |  
     *                                      |  back  |  
     *                                      |        |  
     *                                      |++++++++|  
     * @param result 速度规划
     * @param acc_vel_plan_debug 用于存放针对对应障碍物的消息
     * 
    */
  void PlanVelocityAccordingToObstacles(TargetVelocityInfo *result);

  bool CalcVelocityConsideringObstacles(const common::Path &path, const common::PathPoint &proj_on_path,
                                        TargetVelocityInfo *result, bool *req_aeb_action, bool *req_aeb_warning,
                                        bool *req_notify_uncertain, ConsideringObstaclesDebug& obs_debug);

  /**
   * @brief 用于获取障碍物信息
  */
  void GetObstacleDecisionData(const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info, 
                                                       ObstacleDecision &target_obstacle_information);

  /**
   * @brief 是否更新障碍物信息-不同方位应该考虑不同更新条件
   *        对于正前方的障碍物 暂时只考虑 距离 最近
   *        对于左前方的障碍物 暂时只考虑 距离 最近
   *        对于右前方的障碍物 暂时只考虑 距离 最近
   *        对于周围紧贴的障碍物 无需比较就很要命了 ！！！ 因为穿过来的 速度为 0 、距离为 0
   * 
   *                                                        |---- 切入 ---- 速度赋值为 0；
   *                                      |----- 逆向行驶 ---|---- 不切入
   *                                      |
   * 注Note ： 对于侧前方障碍物 行为 的分类 ---|----- 横穿行驶 --- 风险值 > 50 --- 速度赋值为 0；
   *                                      |
   *                                      |----- 同向行驶 ---|----  切入 ---- 速度直接拷贝 
   *                                                        |---- 不切入 --- 速度
   *                                                                    --- 原来的处理方法将速度根据横向距离进行相应抬高
   *                                                            这样处理方法：自车车速 与 障碍物车速 相差较大的时候可以达到降速通过效果
   *                                                                        但是相差不大的时候 抬高 反而 没有达到 降速通过效果
   * @param new_info 新的障碍信息
   * @param obstacle_orientation 障碍物方位
   * @param old_info 老的障碍物
   * @return 是否更新障碍诶信息
  */

  /**
   * @brief 根据目标所在区域判断是否需要做相关控制动作（跟车/低速跟车，AEB）
   * @details
   * 1：侧前方目标考虑切入；2：静态目标考虑是否可低速跟车；
   * 3：有tar_following时前方的rsk_obj进行剔除；4：筛选出的目标根据TTC阈值判断是否触发不同等级AEB
   * @param param_of_calc_tar_v
   * @param path
   * @param proj_on_path
   * @param is_uncertain
   * @param cur_v_for_plan
   * @param rsk_obj_info
   * @param result
   * @param tar_follwing_obj
   * @return true
   * @return false
   */


      /**
     * @brief 是否考虑该障碍物，并更新障碍物的车速
     *        [A] 对于动态障碍物，正前方、左前方、右前方、周围的紧贴的，其他方位不考虑
     *        [B] 对于静态障碍物，正前方、左前方、右前方、周围的紧贴的，其他方位不考虑
     *                          |++++++++|  |++++++++|  |++++++++|
     *                          |        |  |        |  |        |
     *                          | front  |  | front  |  | front  |
     *                          | left   |  |        |  | right  |
     *                          |++++++++|  |++++++++|  |++++++++|
     * 
     *                          |++++++++|  |++++++++|  |++++++++|
     *                          |        |  |        |  |        |
     *                          |  left  |  |   ego  |  | right  |
     *                          |        |  |        |  |        |
     *                          |++++++++|  |++++++++|  |++++++++|
     * 
     *                          |++++++++|  |++++++++|  |++++++++|
     *                          |        |  |        |  |        |
     *                          |  back  |  |  back  |  |  back  |
     *                          |  left  |  |        |  | right  |
     *                          |++++++++|  |++++++++|  |++++++++|   
     * 
     * @param is_uncertain 是否为不确定障碍物
     * @param obstacle_list_id 障碍物在障碍物列表的位置号
     * @param collision_result 从驾驶地图获取的相关障碍物信息
     * @param obs_information 考虑障碍物信息的障碍物的消息
     * @return 是否考虑障碍物
    */
  void ClassifyTargetByObstacle(bool is_uncertain, plan_var_t cur_v_for_plan, const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info, ObstacleDecision& decision);

  bool IsFollowingObject(const ObstacleDecision &decision, float distance, float l_distance); //

 /**
   * 筛选出跟车场景的CIPV(Closest In-Path Vehicle)
   * 考虑到跟停场景，跟随目标可以是静态车辆；跟随目标应该是车辆类型
   */

  void PlanVelocityAccordingToFollowing(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug);

  /**
   * 筛选出避障场景的前方障碍物，包括：
   * 1. 左前方/右前方切入的车辆
   * 2. 正前方静止障碍物(车辆，遗撒物，锥桶...)
   * 3. 旁车道压线/贴线静止物体/运动车辆
   */
  void PlanVelocityAccordingToAvoidingCollision(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug);

    /**
     * @brief 根据 TTC 判断是否要触发 AEB 当前的触发原则
     *        1. ttc 小于某一阈值 [ttc < tar_ttc]
     *        2. 置信度 大于某一阈值 [obstalce_information.confidence >= 90] 
     *        3. 障碍物类型必须是融合 [obstalce_information.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED]
     *        4. 使能AEB开关 [settings_.enable_aeb]
     *        5. 包围盒之间的距离小于某一阈值 [obstalce_information.static_distance < 0.01F]
     *                                                            障碍物的位置
     *                                                           |+++++++++|
     *                                                           |         |
     *                                                           |+++++++++|
     *                                                                ▵
     * 车辆当前位置                                                     ▿  static_distance
     * |+++++++++|                                               |+++++++++|
     * |    *    |           *                   *               |    *    |
     * |+++++++++|         轨迹点                                 |+++++++++|
     *                                                         假设车辆走到该位置
     *                               distance
     * TTC计算方法： ttc =  -----------------------------
     *                      ego_velocity - obs_velocity
     * 
     * |+++++++++|                              |+++++++++++++++|
     * |   ego   |------------------------------|   target_obs  |
     * |+++++++++|                              |+++++++++++++++|  
     * @param obstalce_information 障碍物信息
     * @return 是否执行AEB  AEB_INVALID, AEB_WARNING, AEB_WARNING_ACTION, AEB_ACTION
    */
  void PlanVelocityAccordingToEmergencyWarning(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info);

  void PlanVelocityAccordingToEmergencyBrake(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug);

  /**
   * @brief 判断是否更新tar_v
   *
   * @param new_info
   * @param old_info
   * @return true
   * @return falses
   */
  bool UpdateTargetVelocityInfo(const TargetVelocityInfo &new_info, TargetVelocityInfo *const old_info) const;


//////////////////////////////  refactoring  ///////////////////////
    
  /**
   * @brief 基于障碍物信息进行 相应的 目标速度 和 目标加速度 计算
   *        同样调用 风险感知计算 多项式的函数计算 对应的数值 并更新 对应的速度规划类型
   * @param main_obs_info 主障碍物信息
   * @param result 结果
  */
  void CalculateVelocityAccordingToObstacle(const ObstacleDecision &decision, ParamOfPolynomialFitting &param, StateOfPolynomial &state, TargetVelocityInfo &result);

  /**
   * @brief 基于障碍物信息进行 相应的 时距 、TTC 和 风险感知值 计算
   * @param main_obs_info 主障碍物信息
   * @param result 结果
  */
  bool CalculateVelocityRiskPerception(const plan_var_t &ego_v, const plan_var_t &obj_distance, plan_var_t obj_v, plan_var_t safe_distance, plan_var_t &risk_perception);

  enum
  {
      FSM_INVAILD = 0,
      FSM_OVER_XMAX_STATIC_OBJ,
      FSM_CUTIN_OBJ,
      FSM_CUTIN_OBJ_EMERGENT, 
      FSM_CUTIN_CROSS,
      FSM_CUTIN_NO_ACTION,
      FSM_CUTIN_NORMAL
  };

  /**
   * @brief 对现在五次多项式打的补丁的状态机 即 针对多项式撒点区间 (x_min ~ x_max)之外 进行一些状态的区分
   *        包括 ： [1]: 200m 开外的静止障碍物 (obj_v < obj_low_speed_boundary) && (obj_distance > xf_max)[FSM_OVER_XMAX_STATIC_OBJ]
   *               [2]: 在采样区间之后进来的障碍物 (A) 自车车速 > 障碍物车速 (a) 障碍物高速 [FSM_CUTIN_OBJ]
   *                                                                  (b) 障碍物低速 [FSM_CUTIN_OBJ_EMERGENT]
   * 
   *                                           (B) 自车车速 < 障碍物车速 (a) 在一定区间内 [FSM_CUTIN_CROSS]
   *                                                                  (b) 在一定区间外 [FSM_CUTIN_NO_ACTION]
   * 
   * @param param 多项式参数 该函数主要利用加速度和减速上下限
   * @param state 多项式初始和末状态 该函数主要利用初状态
   * @param obj_distance 目标车速
   * @param obj_v 目标加速
   * @return 状态机状态
  */
  int FSMPolynomialCurve1d(const ParamOfPolynomialFitting &param, const StateOfPolynomial &state,
                              const plan_var_t &ego_v, const plan_var_t &obj_distance, const plan_var_t &obj_v);

  /**
   * @brief [patch] 正前方静止障碍物，且距离在S最远采样区间[200m]的场景中外，对现在五次多项式打的补丁 ，进行匀减速的目标速度 和 减速度
   *                                         v_ego * v_ego 
   *         Note : a = -1 * ----------------------------------------------
   *                            2 * (distance_obj_to_ego - safe_distance)
   *         针对场景： 
   * @param obj_distance 障碍物距离
   * @param ego_v 自车车速
   * @param tar_v 目标车速
   * @param tar_a 目标加速
  */
  void CalcVelForStaticObstacle(const plan_var_t &obj_distance, const plan_var_t &ego_v,
                                  plan_var_t *tar_v, plan_var_t *tar_a);

  /**
   * @brief [patch] 对现在五次多项式打的补丁 ，针对 x_min之后 障碍物 进行分类
   *        [1]:自车车速 > 障碍物车速 && 障碍物低速  ==> 根据 障碍物距离 和 自车车速 进行 匀减速计算 防止多项式减速不及时
   *            node : 为什么不及时 ？？ 原因对应的末状态加速度值 为 0
   *          加速度                                +  加速度
   *            ^                                  +    ^
   *            |     *--------------*             +    |
   *            |   *                  *           +    |-----------------------------
   *            | *                      *         +    |
   *            ----------------------------->     +    -------------------------------> 时间  
   *        [2]:自车车速 > 障碍物车速 && 障碍物低速  ==> 直接下发 较大减速
   *        [3]:自车车速 < 障碍物车速 && 在一定区间外 ==> 不做动作
   *        [4]:自车车速 < 障碍物车速 && 在一定区间内 ==> 松油门
   * 
   * @param cut_state 对切进来的障碍物进行分类
   * @param param 多项式参数 该函数主减速下限
   * @param obj_v 障碍物速度
   * @param obj_distance 障碍物距离
   * @param ego_v 自车车速
   * @param tar_v 目标车速
   * @param tar_a 目标加速
  */
  void CalcVelForCutInObstacle(const int &cut_state, const ParamOfPolynomialFitting &param, const plan_var_t &obj_distance, const plan_var_t &obj_v,
                                  const plan_var_t &ego_v, plan_var_t *tar_v, plan_var_t *tar_a);

  //CC2ACC
  void UpdatePolynomialInputValueFinalStateOnACC(const ParamOfPolynomialFitting &param,
                                                  const plan_var_t &ego_v, const plan_var_t &obj_distance,
                                                  const plan_var_t &obj_v, StateOfPolynomial &Polynomial_output);

  void CreateVehOBB(const common::Vec2d &pos, plan_var_t heading, common::OBBox2d *obb) const;


/**
 * AEB
*/
  class ActionSmoothing
  {
  public:
      ActionSmoothing()
      {
          req_count_threshold_ = 0;
          time_allowed_        = 0;
          req_count_           = 0;
          time_                = 0;
      }

      void Clear()
      {
          req_count_ = 0;
          time_      = 0;
      }

      void set_req_count_threshold(Int32_t count)
      {
          req_count_threshold_ = count;
      }
      void set_time_allowed(Int32_t time)
      {
          time_allowed_ = time;
      }

      bool Smooth(bool req)
      {
          if (req)
          {
              req_count_++;
              if (req_count_ >= req_count_threshold_)
              {
                  req_count_ = req_count_threshold_;
                  time_      = 0;
                  return true;
              }
          }
          else
          {
              req_count_--;
              if (req_count_ < 0)
              {
                  req_count_ = 0;
                  time_      = 0;
              }
          }

          if (req_count_ > 0)
          {
              time_++;
              if (time_ >= time_allowed_)
              {
                  req_count_ = 0;
                  time_      = 0;
              }
          }

          return false;
      }

  private:
      Int32_t req_count_threshold_;
      Int32_t time_allowed_;
      Int32_t req_count_;
      Int32_t time_;
  };

  ActionSmoothing action_smoothing_aeb_action_;
  ActionSmoothing action_smoothing_aeb_warning_;
  ActionSmoothing action_smoothing_notify_uncertain_;

  struct
  {
      bool aeb_hold_flag;
      bool exist_uncertain_obstacle;

      void Clear()
      {
          aeb_hold_flag            = false;
          exist_uncertain_obstacle = false;
      }
  } status_;

  struct InternalData
  {
      common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum> sample_points;
      driv_map::CollisionTestOnPathResult                                     collision_test_on_path_result;
  } internal_data_;

  /**
   * @brief [patch] 在跟停及避障停车场景中，五次多项式计算出的v平缓 导致 速度 误差导致反馈较小一直蠕行
   * @param final_vel_type 速度规划类型
   * @param final_vel
   * @param final_a
  */
  void VelocityPlanningForObstacleStop(const int &final_vel_type, const plan_var_t &obs_v, plan_var_t &final_vel, plan_var_t &final_a);

/**
 * ODD Fallback
*/
private:
  Float32_t prev_tar_v_ = chassis_status_.v;
  Float32_t prev_tar_a_ = chassis_status_.a;
  Float32_t stop_count_ = 0;
  Float32_t start_count_ = 0;
  bool pre_adas_mode_ = false;

  Int32_t pre_vel_plan_type_;

  bool first_B_II_ = true;
  Float32_t dec_B_II_ = 0.0;
  bool first_C_II_ = true;
  Float32_t dec_C_II_ = 0.0;

  Float32_t curr_slope_ = 0.0;//坡度
  bool slope_flag_ = false;
  Float32_t control_acc_pedal_ = 0.0;//油门开度量
  bool adas_mode_ = false;

  Int8_t road_type_; 

  //降级
  void CalcVelocityAccordingToFallback(ActionPlanningResult action_planning_result, plan_var_t curr_v, VelocityPlanningResult *vel_planning_result);

/**
 * Predictive ACC
*/ 
private:
  const adasisv2::Horizon* adasisv2_horizon_;

  /**
   * @brief PACC entry function. 根据功能分为: CC 和 ACC, 根据道路场景分为 : 直道，坡道，弯道
   * 
   * @param result [in, out] plan target v and a for next predictive time
   */
  void PlanVelocityAccordingToPredictiveControl(plan_var_t cruise_speed, TargetVelocityInfo& result);

  /// check whether PACC function can execute or not
  bool CheckPACCIfValid();
  /// activate PACC or not
  bool is_pacc_valid_;
  
  /// last activate PACC or not
  bool is_pacc_valid_pre_;
  
  /// construct PACC map from ADASIS v2 map or HDMAP
  bool ConstructPACCMap();

#if ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE
  VehicleDynamicParam vehicle_dynamic_param_;

  void RoadSlicing();

  void MergeSlopeNodes();

  int DeterSlopeValues(Float32_t cur_slope); 
  /// 判断是否状态发生跳变 主要是否持续pcc模式


  /**
   * @brief 根据切片后的T-box地图提供的坡度和坡长信息，计算出合理的上坡减速度，
   *        然后根据减速度要求，结合纵向车辆动力学计算出合理的扭矩。
   * @param Torq_limit 上坡扭矩上限，单位:N*m
  */
  bool CalcuTorqueUpLimitAccordSlope(Float32_t &torq_limit);

  /**
   * @brief 根据初步坡道切片信息，确定坡底和坡顶位置，从而确定一个完整的坡道，
   *        然后对该坡道进行坡度平滑，计算出唯一坡度值。
   * @param slope_smooth_info 坡道平滑信息
  */
  bool CalcuSlopeInfo(SlopeSmoothInfo &slope_smooth_info);

  /**
   * @brief 找出经过坡道合并后T-box地图路径上的第一个坡道
   * @param slope_smooth_info 坡道平滑信息
  */
  bool FindOutFirstSlope(SlopeSmoothInfo &slope_smooth_info);

  /**
   * @brief 根据平滑后的唯一坡度值和坡道长度，查表得出该坡道的推荐减速度
   * @param slope_smooth 平滑后的坡度值，单位:%
   * @param slope_length  坡道长度，单位:m
   * @return 当前坡道推荐减速度
  */
  Float32_t LookupDecTable(const Float32_t slope_smooth, const Float32_t slope_length);  
#endif

  /**
   * @brief 当目标速度与自车车速差异过大的时候，根据速度变化阈值限制对其使速度进行平滑过渡，避免加速过猛
   *        调整方式为将当前速度加上（减去）阈值所得的值作为临时目标车速，直到临时目标速度与实际目标速度
   *        之差小于阈值后退出平滑。（PCC适用）
   * @param tar_v 目标速度，单位： m/s
   * @param vel_delta_threshold_min 自车速度与目标速度的速度差平滑下限阈值，单位： m/s
   * @param vel_delta_threshold_max 自车速度与目标速度的速度差平滑上限阈值，单位： m/s
   * @param cur_slope 当前坡度，单位： %
  */
  void TargetVelAndEgoVelDeltaSmooth(plan_var_t &tar_v, plan_var_t cur_slope);

  /**
   * @brief 当目标速度与自车速度差异过大的时候，根据速度变化阈值限制对其使速度进行平滑过渡，避免加速过猛
   *        调整方式为将当前车速与加（减）步长所得值作为临时目标车速，直到临时目标速度与实际目标速度之差
   *        小于阈值后退出平滑。（CC适用）
   * @param tar_v 目标速度，单位： m/s
   * @param vel_delta_threshold 自车速度与目标速度的速度差平滑阈值 单位： m/s
   * @param dec_step 平滑速度每帧减少步长，单位： m/s
   * @param acc_step 平滑速度每帧增加步长，单位： m/s
  */
  void TargetVelAndEgoVelDeltaSmooth(plan_var_t &tar_v, plan_var_t vel_delta_threshold, plan_var_t dec_step, plan_var_t acc_step);

  bool IsStateChanged();

  /**
   * @brief 根据发动机 map 反差表 由发动机当前的转速 和 pcc的目标扭矩
   *        进行是否释放油门的判断
  */
  bool PccIsReleaseThottle(const PAccInput &input, const PAccOutput &output);

  PACCMap pacc_map_;
  
#if ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE
  SlopeNodeList slope_nodes_;

  bool torque_limit_flag_ = false;
  plan_var_t slope_distance_arry[2] = {0};
  plan_var_t torque_limit_arry[2] = {0};
#endif

  /// PACC implement algorithm
  VelocityPlanningPACC pacc_;

/**
 * 对计算出的速度/加速度进行后处理
*/ 
private:

  /**
   * @brief 针对五次多项式会跟根据上一帧计算，会在进自动驾驶点刹
   *        该函数用于更新所有非自动驾驶和自动驾驶的之间过渡
  */
  void VelocityPlanningStartAD();

  /**
   * @brief 平滑后的目标速度
   * @details
   * 1：根据目标速度向量判断是否加减速；2：判断加速，但是目标加速度为负，且当前速度已经小于目标速度，则保持当前车速
   * 3：加速时，平滑后的目标速度小于0.01则归为0
   * @param tar_v
   * @param tar_a
   * @param smoothed_tar_v
   * @param smoothed_tar_a
   */
  void SmoothVelocity(VelocityPlanningResult vel_planning_result, plan_var_t *smoothed_tar_v, plan_var_t *smoothed_tar_a);

  bool       valid_target_velocity_array_;
  plan_var_t array_target_velocity_[3];
  plan_var_t array_target_acceleration_[3];

  /**
   * @brief 用于限制最終的速度， 加速度
  */
  void LimitFinalResult(VelocityPlanningResult& finalresult);

  /**
   * @brief 用于记录本周期的数据作为下一周期的参考
  */
  void RecordData(const VelocityPlanningResult& finalresult);

  void PCCinputfilter(PAccInput &pcc_input);

  void PCCoutputfilter(PAccOutput &pcc_output, const int &pre_vel_plan_type);

  PccModeChangeAccordingToVelSource pcc_mode_change_;
};

} // namespace planning
} // namespace phoenix

#endif // PHOENIX_PLANNING_VELOCITY_PLANNING_POLYNOMIAL_FITTING_H_
