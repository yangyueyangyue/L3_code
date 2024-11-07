/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       velocity_planning_fuzzy_pid.h
 * @brief      速度规划（Fuzzy PID）
 * @details    使用模糊PID算法，规划目标车速及目标加速度
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

#ifndef PHOENIX_PLANNING_VELOCITY_PLANNING_FUZZY_PID_H_
#define PHOENIX_PLANNING_VELOCITY_PLANNING_FUZZY_PID_H_

#include "driving_map_wrapper.h"
#include "math/math_utils.h"
#include "math/matrix.h"
#include "motion_planning.h"
#include "utils/linear_interpolation.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "curve/quintic_polynomial_curve1d.h"
#include "utils/macros.h"

// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <iostream>
// #include <vector>

#define ENABLE_VELOCITY_PLANNING_DEBUGGING_FILE (0)


namespace phoenix
{
namespace planning
{


/**
 * @struct VelocityPlanningFuzzyPID
 * @brief 速度规划（Fuzzy PID）
 */
class VelocityPlanningFuzzyPID
{
public:
    /**
     * @brief VelocityPlanningFuzzyPID的构造函数
     * @details 1：设置默认的参数；2：清除暂存的信息
     * @author  yzdfcv
     * @date 2022.6.21
     */
    VelocityPlanningFuzzyPID();

    /**
     * @brief VelocityPlanningFuzzyPID的析构函数
     * @details 1：未做任何处置
     * @author yzdfcv
     * @date 2022.6.21
     */
    ~VelocityPlanningFuzzyPID();

    /**
     * @brief 设定默认的参数配置
     * @details 1：设置功能使能选项；2：设置车辆尺寸及定位安装位置参数；3：不同场景下速度及相对速度限制值
     * @author yzdfcv
     * @date 2022.6.21
     */
    void SetDefaultParam();

    /**
     * @brief 更新车辆参数配置
     */

    void UpdateVehicleParameter();
    /**
     * @brief 加载planning_config文件内的信息进行参数配置
     * @details 1：配置信息管理在xml文件中；2：对设置的参数进行最小值限制
     * @author yzdfcv
     * @date 2022.6.22
     */
    void Configurate(const VelocityPlanningConfig &conf);
    /**
     * @brief 调用Clear（）函数进行信息的清除
     * @details 1：清除目标防抖的统计处理信息；2：清除平滑信息；3：清除输出的目标速度及加速度array内的信息
     * @author yzdfcv
     * @date 2022.6.22
     */
    void ClearHoldInfo();

    /**
     * @brief 规划目标车速及目标加速度
     * @return true - 成功，false - 失败
     */
    bool Plan(const VelocityPlanningDataSource &data_source);

    /**
     * @brief 获取规划的结果
     * @return 规划的结果
     */
    inline const VelocityPlanningResult &GetResultOfPlanning() const
    {
        return (result_of_planning_);
    }

    /**
     * @brief 读取事件报告信息
     * @param[in] num 最多可以输出的事件的数量
     * @param[out] events 保存输出的事件的列表地址
     * @return 实际读取的事件的数量
     */
    Int32_t GetEventReporting(Int32_t num, ad_msg::EventReporting *const events) const;

private:
    enum
    {
        DRIVING_DIRECTION_FORWARD = 0,
        DRIVING_DIRECTION_BACKWARD
    };

    enum
    {
        EVENT_TYPE_AEB_WARNING,
        EVENT_TYPE_AEB_ACTION,
        EVENT_TYPE_UNCERTAIN_OBSTACLE,
        EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL,
        EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP
    };
    
    //
    struct TargetVelocityUpperLimit { 
        bool valid;
        Int32_t tar_type;
        plan_var_t tar_v;

        void Clear() {
            valid = false;
            tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
            tar_v = 0.0F;
        }

        TargetVelocityUpperLimit() {
            Clear();
        }
    };//

    struct TargetVelocityInfo
    {
        Int32_t                    tar_type;
        plan_var_t                 tar_v;
        plan_var_t                 tar_a;
        plan_var_t                 tar_s;
        phoenix::common::PathPoint tar_pos;

        struct ObstacleInfo
        {
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

            void Clear()
            {
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
            }

            ObstacleInfo()
            {
                Clear();
            }
        } obstacle_info;

        plan_var_t tar_v_true;
        plan_var_t tar_a_true;

        void Clear()
        {
            tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
            tar_v    = 0.0F;
            tar_a    = 0.0F;
            tar_s    = 0.0F;
            tar_pos.Clear();
            obstacle_info.Clear();

            tar_v_true = 0.0F;
            tar_a_true = 0.0F;
        }

        TargetVelocityInfo()
        {
            Clear();
        }
    };

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

        bool Smooth(bool req)//  //AEB执行之前有个hold状态平滑（aeb_hold_flag）：障碍物持续5帧以下就消失-3m/s2减速度，否则-5m/s2减速度
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

    struct JitterSuppression
    {
        bool valid = false;

        enum
        {
            STATUS_NO_OBSTACLE = 0,
            STATUS_REQ_DECELERATION,
            STATUS_IN_DECELERATION,
            STATUS_REQ_ACCELERATION
        };
        Int32_t            status;
        Int32_t            acc_request_counter;
        Int32_t            dec_request_counter;
        Int32_t            acc_delay_time;
        Int32_t            dec_delay_time;
        TargetVelocityInfo tar_velocity_info;

        void Clear()
        {
            status              = STATUS_NO_OBSTACLE;
            acc_request_counter = 0;
            dec_request_counter = 0;
            acc_delay_time      = 0;
            dec_delay_time      = 0;
            tar_velocity_info.Clear();//
        }

        JitterSuppression()
        {
            Clear();
        }
    };

private:
   #if 1 //多项式
   void CubicComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t x1,   const Float32_t dx1, const Float32_t p, Float32_t* coef3_) ;///////
   void QuinticComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t ddx0,const Float32_t x1, const Float32_t dx1, const Float32_t ddx1, const Float32_t p, Float32_t* coef5_);///////

   void CubicPolynomialCurve1d(const Float32_t* coef3_, const Int32_t order, const Float32_t p, Float32_t* Cubic) ;//////
   void QuinticPolynomialCurve1d(const Float32_t* coef5_, const Int32_t order, const Float32_t p, Float32_t* Quintic) ;//////

   void  linspace(Float32_t x1, Float32_t x2, int n, Float32_t* y);////
    
   // void polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order,Eigen::VectorXd coeffs) ;


   #endif

    void PlanVelocityAccordingToPathCurvature(
        const common::Path &                                                                       path,
        const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
        const common::PathPoint &proj_on_path, plan_var_t range_start_s, plan_var_t range_end_s,
        TargetVelocityInfo *result);

    void PlanVelocityAccordingToObstacles(TargetVelocityInfo *result);
    bool CalcVelocityConsideringObstacles(const common::Path &path, const common::PathPoint &proj_on_path,
                                          TargetVelocityInfo *result, bool *req_aeb_action, bool *req_aeb_warning,
                                          bool *req_notify_uncertain);

    void PlanVelocityAccordingToTrafficLight(const common::Path &path, const common::PathPoint &proj_on_path,
                                             TargetVelocityInfo *result);

    void PlanVelocityAccordingToTrafficSignal(const common::Path &path, const common::PathPoint &proj_on_path,
                                              TargetVelocityInfo *result);

    void PlanVelocityAccordingToSceneStory(const common::Path &path, const common::PathPoint &proj_on_path,
                                           TargetVelocityInfo *result);
                                           
    void PlanVelocityAccordingToTunnel(const common::Path &path, const common::PathPoint &proj_on_path,
                                           TargetVelocityInfo *result);

    void PlanVelocityAccordingToRamp(const common::Path &path, const common::PathPoint &proj_on_path,
                                           TargetVelocityInfo *result);
    /**
     * @brief 判断是否更新tar_v
     *
     * @param new_info
     * @param old_info
     * @return true
     * @return false
     */
    bool UpdateTargetVelocityInfo(const TargetVelocityInfo &new_info, TargetVelocityInfo *const old_info) const;

    void SuppressJitter(JitterSuppression &jitter_suppression, const TargetVelocityInfo &tar_v_info,
                        TargetVelocityInfo *result) const;

    void CreateVehOBB(const common::Vec2d &pos, plan_var_t heading, common::OBBox2d *obb) const;
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

    struct ParamOfCalcTarVelocityLinearly
    {
        plan_var_t time_of_ctl_range;
        plan_var_t min_dist_of_ctl_range;
        plan_var_t accelaration_of_activity_condition;
        plan_var_t deceleration_of_activity_condition;

        bool       stop_accurately;
        plan_var_t min_deceleration_time;

        ParamOfCalcTarVelocityLinearly()
        {
            time_of_ctl_range                  = 3.0F;
            min_dist_of_ctl_range              = 5.0F;
            accelaration_of_activity_condition = 0.5F;
            deceleration_of_activity_condition = -0.5F;

            stop_accurately       = false;
            min_deceleration_time = 1.0F;
        }
    };
    #if 1 // 多项式
    bool CalcTarVelocityLinearly(const ParamOfCalcTarVelocityLinearly &param, plan_var_t curr_v, plan_var_t obj_v,
                                 plan_var_t dist_to_obj, plan_var_t safe_dist, plan_var_t *tar_v, plan_var_t *tar_a,
                                 Int32_t *dist_gap_level, Int32_t *rel_spd_level) const;
    
    bool CalcTarVelocityQuinticPolynomialCurve1d(const ParamOfCalcTarVelocityLinearly &param, plan_var_t curr_v, plan_var_t obj_v,
                                 plan_var_t dist_to_obj, plan_var_t safe_dist, bool is_cc, plan_var_t *tar_v, plan_var_t *tar_a) ;

    void CalcVelocityToFallback(ActionPlanningResult action_planning_result, plan_var_t curr_v, VelocityPlanningResult *vel_planning_result);
    
    bool TargetVelocityFromPathCurvature(
    const common::Path &path,
    const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
    const common::PathPoint &proj_on_path, plan_var_t *target_velocity) ;
    
    #endif

    struct ClassifyTargetByObstacleResult
    {
        bool is_uncertain;
        bool aeb_warning;
        bool aeb_action;
        bool req_dec;

        TargetVelocityInfo tar_velocity_info;

        void Clear()
        {
            is_uncertain = false;
            aeb_warning  = false;
            aeb_action   = false;
            req_dec      = false;
            tar_velocity_info.Clear();
        }
        ClassifyTargetByObstacleResult()
        {
            Clear();
        }
    };
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
    bool
    ClassifyTargetByObstacle(const ParamOfCalcTarVelocityLinearly &param_of_calc_tar_v, const common::Path &path,
                             const common::PathPoint &proj_on_path, bool is_uncertain, plan_var_t cur_v_for_plan,
                             const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info,
                             ClassifyTargetByObstacleResult *const               result,
                             const driv_map::CollisionTestOnPathResult::ObjInfo *tar_follwing_obj = Nullptr_t) const;

    void AddEventReporting(Int32_t event_type);

private:
    //
    VelocityPlanningConfig config_;

    struct
    {
        bool enable_acc;
        bool enable_aeb;
        bool enable_aeb_pre_dec;
        bool enable_following;
        bool enable_low_speed_following;

        plan_var_t tar_v;
        plan_var_t tar_a;

        plan_var_t lateral_acceleration_limit;
        bool       enable_stop_by_traffic_light;
    } settings_;

    struct
    {
        // 车长，单位：米
        plan_var_t vehicle_length;
        // 车宽，单位：米
        plan_var_t vehicle_width;
        // 车辆的定位点到 front of vehicle 的距离，单位：米
        plan_var_t dist_of_localization_to_front;
        // 车辆的定位点到 rear of vehicle 的距离，单位：米
        plan_var_t dist_of_localization_to_rear;
        // 车辆的定位点到中心点的距离，单位：米
        plan_var_t dist_of_localization_to_center;
        // 车辆外接圆半径，单位：米
        plan_var_t vehicle_circumradius;

        plan_var_t max_velocity_limit;
        plan_var_t max_acceleration_limit;
        plan_var_t max_deceleration_limit;
        plan_var_t tunnel_velocity_limit;
        plan_var_t ramp_velocity_limit;

        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> 
            velocity_limit_table_for_stop_situation;//stop_situation 停车场景速度限制列表

        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
            velocity_limit_table_for_stop_accurately_situation;//stop_accurately_situation 精准停车场景速度限制列表

        struct StopAccurately
        {
            bool       enable;
            plan_var_t braking_distance;
            plan_var_t proposed_decelaration;

            StopAccurately()
            {
                enable                = false;
                braking_distance      = 0.8F;
                proposed_decelaration = 3.0F;
            }
        } stop_accurately;    //精准停车struct定义

        plan_var_t sample_step_len_for_collision_test;

        // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
            relative_velocity_limit_table_by_dist_to_static_obj;
        // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE>
            relative_velocity_limit_table_by_dist_to_dynamic_obj;

        // AEB启动TTC时间
        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> aeb_action_ttc_table;

        common::StaticVector<common::LerpTableNodeType1, MAX_LERP_TABLE_SIZE> curvatrue_limit_velocity_table;

        plan_var_t safe_distance_to_static_obstacle;
        plan_var_t safe_distance_for_low_speed_following;
        plan_var_t safe_distance_to_dynamic_obstacle;
        plan_var_t safe_time_to_dynamic_obstacle;
        plan_var_t safe_distance_in_backing_mode;

        // paramters for suppressing the jitter caused by the noise of sensors
        Int32_t jitter_suppressing_acce_delay_time;
        Int32_t jitter_suppressing_dece_delay_time;
        Int32_t jitter_suppressing_acce_req_counter_threshold;
        Int32_t jitter_suppressing_dece_req_counter_threshold;
    } param_;

    // Environment
    Int64_t                 curr_timestamp_;
    Int32_t                 driving_direction_;
    common::TrajectoryPoint curr_position_;
    // 驾驶地图信息
    const driv_map::DrivingMapWrapper *driving_map_;
    // 行为规划的结果
    ActionPlanningResult result_of_action_planning_;
    // 轨迹规划的结果
    TrajectoryPlanningResult result_of_trajectory_planning_;
    // 车身信息
    ad_msg::Chassis chassis_status_;
    // 特殊的底盘信息
    ad_msg::SpecialChassisInfo special_chassis_status_;
    // 特定型号车辆的信息
    ad_msg::ChassisCtlCmd chassis_ctl_cmd_;
    // 红绿灯信息
    ad_msg::TrafficLightList traffic_light_list_;
    //
    ad_msg::TrafficSignalList traffic_signal_list_;

    // 规划用的当前车速（此车速必须大于0,否则将产生除0错误）
    plan_var_t   veh_velocity_for_planning_;
    common::Path target_trajectory_;
    common::Path changed_trajectory_;
    common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum>
                      target_trajectory_curvature_info_;
    common::PathPoint veh_proj_on_major_ref_line_;
    common::PathPoint veh_proj_on_target_trajectory_;

    common::StaticVector<TargetVelocityInfo, 20> tar_v_info_list_;

    // 规划的结果
    VelocityPlanningResult result_of_planning_;

    JitterSuppression obj_jitter_suppression_;

    ActionSmoothing action_smoothing_aeb_action_;
    ActionSmoothing action_smoothing_aeb_warning_;
    ActionSmoothing action_smoothing_notify_uncertain_;

    bool       valid_target_velocity_array_;
    plan_var_t array_target_velocity_[3];
    plan_var_t array_target_acceleration_[3];

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

    common::StaticVector<ad_msg::EventReporting, 4> event_reporting_list_;

    struct InternalData
    {
        common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum> sample_points;
        driv_map::CollisionTestOnPathResult                                     collision_test_on_path_result;
    } internal_data_;

      /// TODO: 初始化
    /* xrf   begin*/
    private:

        Float32_t prev_tar_v_ = chassis_status_.v;
        Float32_t prev_tar_a_ = chassis_status_.a;
        Float32_t stop_count_ = 0;
        Float32_t start_count_ = 0;
        bool pre_adas_mode_ = false;

        bool first_B_II_ = true;
        Float32_t dec_B_II_ = 0.0;
        bool first_C_II_ = true;
        Float32_t dec_C_II_ = 0.0;

        Float32_t curr_slope_ = 0.0;//坡度
        bool slope_flag_ = false;
        Float32_t control_acc_pedal_ = 0.0;//油门开度量
        bool adas_mode_ = false;

        void CalculateTargetVelocityFromTunnel(TargetVelocityUpperLimit *target_vel_result);//
    /* xrf   end*/

      /// TODO: For debugging
        #if (ENABLE_VELOCITY_PLANNING_DEBUGGING_FILE)
        Char_t str_buff_[1024*2];
        common::LogFile log_file_speed;
        bool open_log_file_speed = false;
        #endif

};

} // namespace planning
} // namespace phoenix

#endif // PHOENIX_PLANNING_VELOCITY_PLANNING_FUZZY_PID_H_
