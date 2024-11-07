/** @copyright Copyright (c) 2018-2023 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       velocity_planning_pacc.h
 * @brief      基于ADAS地图和最优控制算法实现满足时效性要求的经济性最优速度规划
 * @details    基于ADAS地图通过获取前方道路的坡度、曲率、限速等道路信息，并进行地形匹配，
 * 按照最优算法控制发动机和变速箱，主动实现对车辆车速、挡位等的最优控制，从而使得不同场景下燃油经济性最优:
 * 1. Predictive CC  : 直道，坡道，弯道
 * 2. Predictive ACC : 包括PCC，并在此基础上实现ACC，动态调整参考速度
 * 在条件满足时, 激活 PCC, PACC功能; 否则复用常规实现: CC, ACC
 * the detail design document @see DFCV_L3_PACC.xlsx
 * @authors    zhangq, wangwh
 * @date       2023.05.15
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_VELOCITY_PLANNING_PACC_H_
#define PHOENIX_PLANNING_VELOCITY_PLANNING_PACC_H_
#include "container/static_vector.h"
#include "utils/linear_interpolation.h"

#include "motion_planning.h"
#include "msg_chassis.h"

namespace phoenix {
namespace planning {

/**
 * @enum
 * @brief 二分法求解状态
 *        DichotomySolutionSuccess : 二分法 F(lambda)在允许的范围内
 *        DichotomySolutionFailInit ： 在进入二分法的失败 即 左值对应的下边界 与 右值对应的上边界 同号
 *        DichotomySolutionUpperLimit ： 二分法在求解过程超出循环次数
 *        DichotomySolutionLeftRightTooBig : 二分法的左值和右值 相差过大
*/  
enum {
  DichotomySolutionInValid = 0,
  DichotomySolutionSuccess = 1,
  DichotomySolutionFailInit = 2,
  DichotomySolutionUpperLimit = 3,
  DichotomySolutionLeftRightTooBig = 4
};


/**
 * @enum
 * @brief 基于模型预测的求解状态 即 MPC的求解状态
 *        MpcSolutionUpperLimit ： MPC 达到了设定预测时间
 *        MpcSolutionMapTooShort : MPC 获取的地图长度不够
*/  
enum {
  MpcSolutionInValid = 0,
  MpcSolutionUpperLimit = 1,
  MpcSolutionMapTooShort = 2
};

#if ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE
/**
 * @enum
 * @brief 坡度分类
*/

enum {
   SLOPE_VALUE_INVAILD = 0,
   SLOPE_VALUE_ONE,
   SLOPE_VALUE_TWO,
   SLOPE_VALUE_THREE,
   SLOPE_VALUE_FOUR,
   SLOPE_VALUE_FIVE,
   SLOPE_VALUE_SIX,
   SLOPE_TYPE_DOWN_SLOPE,
   SLOPE_TYPE_FLAT_SLOPE
};

enum class UphillType : uint8_t{
  UPHILL_TYPE_INVAILD = 0, // 无效
  UPHILL_TYPE_ONE,         // 正在坡中       
  UPHILL_TYPE_TWO,         // 下坡转上坡                   
  UPHILL_TYPE_THREE,       // 平路转上坡   
  UPHILL_TYPE_FOUR,        // 上坡转下坡
  UPHILL_TYPE_FIVE         // 上坡转平路
};

struct SlopeNode {
  // 每一段坡度的起点
  Float32_t start_s;
  // 每一段坡度的终点
  Float32_t end_s;
  // 每一段坡度的平均坡度
  Float32_t average_slope;
  // 每段的最大坡度
  Float32_t max_slope;
  // 每一段坡度的长度
  Float32_t slope_length;
  // 每一段坡度的类型
  int slope_type;

  void Clear() {
    start_s = 0.0;
    end_s = 0.0;
	  max_slope = 0.0;
    average_slope = 0.0;
    slope_type = SLOPE_VALUE_INVAILD;
    slope_length = 0.0;
  }

  SlopeNode(){
    Clear();
  }
};

struct SlopeNodeList{
  bool is_valid;
  /// Max PACC point number
  static const Uint16_t SLOPE_NODE_NUM = 300;
  
  common::StaticVector<SlopeNode, SLOPE_NODE_NUM> nodes;

  void Clear() {
    nodes.Clear();
    is_valid = false;
  }

  SlopeNodeList(){
    Clear();
  }

};

/**
 * @brief 道路切片后路径上第一个坡道的信息
*/
struct SlopeSmoothInfo {
  // 平滑后的坡度值，单位:%
  Float32_t slope_smooth;
  // 坡道长度，单位:m
  Float32_t slope_length;
  // 当前位置距离第一个坡底的长度，单位:m 
  Float32_t distance_to_slope;
  // 坡底处坡道切片ID
  uint8_t slope_start_id;
  // 坡顶出坡道切片ID
  uint8_t slope_end_id;
  // 上坡类型，由坡底前一段坡道切片类型决定
  UphillType uphill_type;

  void Clear(){
    slope_smooth = 0.0f;
    slope_length = 0.0f;
    distance_to_slope = 0.0f;
    slope_start_id = 0u;
    slope_end_id = 0u;
    uphill_type = UphillType::UPHILL_TYPE_INVAILD;
  }

  SlopeSmoothInfo(){
    Clear();
  }
};

#endif

struct PACCPoint {
  /// 以自车为原点 longitudinal offset [m]
  Float32_t s;

  /// 坡度 [-] 比值，非%
  Float32_t slope;

  /// 曲率 [1/m]
  Float32_t curvature;

  /// 卡车限速 [m/s]
  Float32_t speed_limit;

  void Clear() {
    s = 0.0f;
    slope = 0.0f;
    curvature = 0.0f;
    speed_limit = 0.0f;
  }

  PACCPoint() {
    Clear();
  }

};

/**
 * @struct PACCMap
 * @brief map for PACC
 * 
 */
struct PACCMap {
  /// Max PACC point number
  static const Uint16_t PACC_MAX_POINT_NUM = 500;
  
  common::StaticVector<PACCPoint, PACC_MAX_POINT_NUM> points;
  /// the actual path points number
  Uint16_t pacc_point_num;

  /// 等长距离, step distance [m], TBOX : 10m, MPU : 5m
  Float32_t pacc_point_s_step;

  /// 数据来源, 0 : invalid, 1 : TBOX(ADASIS v2), 2 : MPU(private)
  Uint8_t pacc_points_source;

  void Clear() {
    pacc_point_num = 0;
    pacc_point_s_step = 10.0F;
    pacc_points_source = 0;
    points.Clear();
  }

  PACCMap() {
    Clear();
  }
};

/**
 * @struct PAccInput
 * @brief PACC Algorithm Input data
 * 
 */
struct PAccInput {
  /// 地图是否有效
  bool pcc_map_vaild;
  /// Pcc地图
  PACCMap pacc_map;
  /// 未来地图的类型
  Int8_t road_type;

  /// 巡航车速 [m/s]
  Float32_t cruise_speed;

  /// 道路限速 [m/s], 用于PCC退出条件计算
  Float32_t road_speed_limit;

  /// 发动机转速 [rpm]
  Float32_t engine_speed;
  /// 发动机扭矩 [nm]
  Float32_t engine_torque;
  
  Float32_t engine_torque_limit;

  /// 当前档位 [-]  -1 : R, 0 : N, 1-14 : D, 126 : P
  Int8_t gear_num;

  /// 当前制动力矩 [nm]
  Float32_t brake_torque;

  /// 原始 自车车速 [m/s] : 从底盘获取
  Float32_t ego_speed_raw;

  /// 滤波后 自车车速 [m/s]
  Float32_t ego_speed;
  /// 自车加速度 [m/s^2]
  Float32_t ego_accel;
  // 自车质量 [kg]
  Float32_t ego_mass;

  // 速度规划类型[-]
  Int8_t pre_velocity_planning_type;

  VelocityPlanningInternal* planning_debug_;

  void Clear() {
    pacc_map.Clear();
    pcc_map_vaild = false;
    road_speed_limit = 100 / 3.6;
    engine_speed = 0.0;
    engine_torque = 0.0;
    gear_num = 0;
    brake_torque = 0.0;
    ego_speed_raw = 0.0F;
    ego_speed = ego_speed_raw;
    ego_accel = 0.0;
    ego_mass = 0.0;
    pre_velocity_planning_type = 0;
    road_type = 0;
    engine_torque_limit = 0.0;
  }

  PAccInput(){
    Clear();
  }
};

/**
 * @struct PAccOutput
 * @brief PACC Algorithm Output data
 * 
 */
struct PAccOutput {
  /// 目标车速 [m/s]
  Float32_t tar_v;
  /// 目标加速度 [m/s^2]
  Float32_t tar_a;
  // 目标档位 [-]
  Int8_t tar_gear;
  // 目标档位是否有效
  bool tar_gear_vaild;
  // 目标发动机扭矩 [N/m]
  Float32_t tar_engine_torque;
  // 目标驱动力 [N]
  Float32_t tar_drive_force;
  // 当前阻力值 [N]
  Float32_t cur_resistance;
  // 当前滚阻 [N] 
  Float32_t cur_Ff;
  // 当前风阻 [N] 
  Float32_t cur_Fw;
  // 当前坡阻 [N] 
  Float32_t cur_Fi;
  // 目标发动机转速 [rpm]
  Float32_t tar_engine_ne;
  /// 目标油门开度 [-]
  Float32_t tar_engine_throttle;
  // pcc 速度规划状态
  Int8_t pcc_vel_plan_type;
  // 二分法求解状态
  Int8_t dichotomy_solution_state;
  // 未限制车速的状态
  Float32_t target_unlimit_vel;
  // 对应的Mpc求解状态 
  // note : 该状态只有进入到二分法的正式求解的情况才有效
  Int8_t mpc_solution_state;

  // pcc输出的结果是否有效
  // note : 限制只认为初始化失败为无效的
  bool pcc_result_vaild;

  void Clear() {
      tar_v = 0;
      tar_a = 0.0;
      tar_gear = 0;
      tar_engine_torque = 0.0;
      tar_drive_force = 0.0;
      cur_resistance = 0.0;
      cur_Ff = 0.0;
      cur_Fw = 0.0;
      cur_Fi = 0.0;
      target_unlimit_vel = 0.0;
      tar_engine_ne = 0.0;
      tar_engine_throttle = 0.0;
      pcc_vel_plan_type = 0;
      dichotomy_solution_state = 0;
      mpc_solution_state = 0;
      pcc_result_vaild = true;
      tar_gear_vaild = false;
  }

  PAccOutput(){
    Clear();
  }

};

/**
 * @struct PccModelParameter
 * @brief the key parameters to tunning
 * 
 */
struct PccModelParameter {
  /// 权重系数_参考车速 [-]
  Float32_t kappa_1;

  /// 权重系数_末状态车速 [-]
  Float32_t kappa_2;
  
  /// control time interval [s]
  Float32_t delta_t;

  /// 预测total time length [s]
  Float32_t tp;

  /// 预测步长 [-]
  Uint16_t np;

  void Clear() {
    delta_t = 0.1F;
    kappa_1 = 0.0F;
    kappa_2 = 0.0F;
    tp = 0.1F;
    np = 1;
  }

  /**
   * @brief 构造函数
   */
  PccModelParameter() {
    Clear();
  }
};

struct PccFollowParameter {
    // 跟车时距 --  key ： 车速[m/s] 、 value : 时距[s]
    common::StaticVector<common::LerpTableNodeType1, 8> time_distance_table;
    // 安全距离 --  key ： 车速[m/s] 、 value : 安全距离[m]
    common::StaticVector<common::LerpTableNodeType1, 8> safe_distance_table;
    // 跟车 与 巡航 切换距离 --  key : 车速[m/s] 、value ： 切换距离([m]
    common::StaticVector<common::LerpTableNodeType1, 8> cc_to_follow_distance_table;
    // 跟车距离 --  key : 车速[m/s] 、 value : 跟车距离[m]
    common::StaticVector<common::LerpTableNodeType1, 8> follow_distance_table;
    // 预留距离 --  key ： 相对车速[m/s] 、value : 预留距离[m]
    common::StaticVector<common::LerpTableNodeType1, 8> redundance_distance_table;

    void Clear() {
        time_distance_table.Clear();
        safe_distance_table.Clear();
        cc_to_follow_distance_table.Clear();
        follow_distance_table.Clear();
        redundance_distance_table.Clear();
    }

    /**
     * @brief 构造函数
     */
    PccFollowParameter(){
        Clear();
    }
};

/**
 * @struct FuelRateFitCoef
 * @brief Fuel Consumption Rate Model Fitting Coefficient
 * Mf = L00 + L01 * Ne + L02 * Ne^2 + L10 * Te + L11 * Te * Ne + L12 * Te * Ne^2 + L20 * Te^2 + L21 * Te^2 * Ne + L22 * Te^2 * Ne^2
 * TODO: BIT-TFCM-2
 */
struct FuelRateFitCoef {
  Float32_t L00;
  Float32_t L01;
  Float32_t L02;
  Float32_t L10;
  Float32_t L11;
  Float32_t L12;
  Float32_t L20;
  Float32_t L21;
  Float32_t L22;

void Clear() {
  L00 = 0.0;
  L01 = 0.0;
  L02 = 0.0;
  L10 = 0.0;
  L11 = 0.0;
  L12 = 0.0;
  L20 = 0.0;
  L21 = 0.0;
  L22 = 0.0;
}

FuelRateFitCoef() {
  Clear();
}
};

/**
 * @struct struct FuelRateFitCoefBasaVandF 
 * @brief Fuel Consumption Rate Model Fitting Coefficient Base on Vx and Ft
    MFF = h01 * vv + h02 * (vv^2) + h03 * (vv^3) + h00
          h10 *  FF + h11 * FF * vv + h12 * FF * (vv^2) + h13 * FF * (vv^3) + 
          h20 * (FF^2) + h21 * vv * (FF^2) + h22 * (vv^2) * (FF^2) + h23 * (vv^3) * (FF^2)
 * TODO: BIT-TFCM-2
 */
struct FuelRateFitCoefBasaVandF {
  Float32_t h00;
  Float32_t h01;
  Float32_t h02;
  Float32_t h03;
  Float32_t h10;
  Float32_t h11;
  Float32_t h12;
  Float32_t h13;
  Float32_t h20;
  Float32_t h21;
  Float32_t h22;
  Float32_t h23;

void Clear() {
  h00 = 0.0;
  h01 = 0.0;
  h02 = 0.0;
  h03 = 0.0;
  h10 = 0.0;
  h11 = 0.0;
  h12 = 0.0;
  h13 = 0.0;
  h20 = 0.0;
  h21 = 0.0;
  h22 = 0.0;
  h23 = 0.0;
}

FuelRateFitCoefBasaVandF() {
  Clear();
}
};


/**
 * @brief Fuel consumption rate model parameter
 * 
 */
struct FuelRateModelParam {
  // 燃油消耗系数
  FuelRateFitCoef fuel_comsun;
  FuelRateFitCoefBasaVandF fuel_comsun_v_f;

  void Clear() {
    fuel_comsun.Clear();
    fuel_comsun_v_f.Clear();
  }

  FuelRateModelParam() {
    Clear();
  }
};

/**
 * @struct EnvironmentParam
 * @brief Variable environment parameters, come from sensor(filter), estimate or machine learning 
 * 
 */
struct EnvironmentParam {
  /// 空气密度 [kg/m^3]
  Float32_t rho;
  /// 风阻系数 [-]
  Float32_t cd;
  /// 重力加速度 [m/s^2]
  Float32_t g;
  /// 整车质量 [kg]
  Float32_t mass;
  /// 滚动阻力系数
  Float32_t f;

  void Clear() {
    rho = 0.0F;
    cd = 0.0F;
    g = 0.0F;
    mass = 0.0F;
    f = 0.0F;
  }

  EnvironmentParam(){
    Clear();
  }
};

/**
 * @struct LongitudinalDynamicModelParam
 * @brief Longitudinal dynamic model parameter
 * 
 */
// TODO: 与VehicleDynamicParam重复, 车辆动力学相关参数需统一使用单例方式获取
struct LongitudinalDynamicModelParam {
  /// 环境相关的数据
  EnvironmentParam environment;

  /// 机械传动效率 [-]
  Float32_t eta;
  /// 车轮滚动半径 [m]
  Float32_t rw;
  /// 主减速器传动比 [-]
  Float32_t If;
  /// 迎风面积 [m^2]
  Float32_t Af;
  
  // 变速箱传动比 - Vector
  common::StaticVector<Float32_t, 14> Ig;

  void Clear() {
    Ig.Clear();

    environment.Clear();
    
    eta = 0.0F;
    rw = 0.0F;
    If = 0.0F;
    Af = 0.0F;
  }

  LongitudinalDynamicModelParam() {
    Clear();
  }
};

/**
 * @struct VehicleConstraintSet
 * @brief 
 * 
 */
struct VehicleConstraintSet {
    /// 车速上下限 [m/s]
    Float32_t vmin;
    Float32_t vmax;

    Float32_t ref_vd_limit_upper;
    Float32_t ref_vd_limit_lower;

    /// 最大扭矩 和 最大转速 对应表
    common::StaticVector<common::LerpTableNodeType1, 11> 
          engine_speed_limit_table_for_engine_torque;

    // 最小扭矩
    Float32_t engine_torque_min;

    // 转速的上下限
    Float32_t ne_min;
    Float32_t ne_max;

    void Clear() {
      engine_speed_limit_table_for_engine_torque.Clear();
      vmin = 0.0;
      vmax = 0.0;
      ref_vd_limit_upper = 0.0;
      ref_vd_limit_lower = 0.0;
    }

    VehicleConstraintSet() {
      Clear();
    }
};

/**
 * @class PccResultCount
 * @brief 用于记录pcc是否可用
 */

class PccResultCount {
  public:
    PccResultCount() {
      result_count_threshold_ = 0;
      result_count_ = 0;
      result_is_vaild_ = true;
    }

    void Clear() {
      result_count_ = 0;
      result_is_vaild_ = true;
    }

    void Set_Result_Count_Threshold(int32_t count) {
      result_count_threshold_ = count;
    }

    bool GetResultState() const {
      return result_is_vaild_;
    }

    void JudgeResultIsVaild(int16_t result) {
      if (DichotomySolutionFailInit == result) {
          result_count_++;
      } else {
          result_count_ = 0;
      }

      if (result_count_ <= result_count_threshold_) {
          result_is_vaild_ = true;
      } else {
          result_is_vaild_ = false;
      }
    }

  private:
    int32_t result_count_threshold_;
    int32_t result_count_;
    bool result_is_vaild_;
};

/**
 * @class KickdownCount
 * @brief 用于记录Kickdown周期
 */
class KickdownCount {
  public:
    KickdownCount() {
      kickdown_count_threshold_ = 0;
      kickdown_count_ = 0;
      is_free_count_ = true;
    }

    void Clear() {
      kickdown_count_ = 0;
      is_free_count_ = true;
    }

    void Set_Kickdown_Count_Threshold(int32_t count) {
      kickdown_count_threshold_ = count;
    }

    bool GetResultState() const {
      return is_free_count_;
    }

    void Kickdown_Count_start(bool Kickdown_Count_flag) {     
      if (Kickdown_Count_flag) {
        kickdown_count_ ++;
      }

      if (kickdown_count_ <= kickdown_count_threshold_) {
          is_free_count_ = false;
      } else {
          is_free_count_ = true;
      }
    }

  private:
    int32_t kickdown_count_threshold_;
    int32_t kickdown_count_;
    bool is_free_count_;
};

/**
 * @struct LongitudinalModelPrediction
 * @brief longitudinal dynamic predciton based on fuel optimal control
 * 
 */
class LongitudinalModelPrediction {
 public:
  /**
   * @brief constructor
   */
  LongitudinalModelPrediction();

  /**
   * @brief destructor
   */
  ~LongitudinalModelPrediction();

  /**
   * @brief 初始化参数车身固有参数  和 算法可调解的参数
   * 
   */
  void Init(const LongitudinalDynamicModelParam *lon_dyn_param,  
            const FuelRateModelParam* fuel_rate_param,
            const VehicleConstraintSet* constraint);

  /**
   * @brief 求解出最优的 lambda ， 扭矩 ，发动机转速
   * @param init_lambda 初始的λ 
   * @param data_source 用于获取轨迹规划 行为规划的结果
   */
  void Solver(const Float32_t &init_lambda, const PccModelParameter *mpc_config, const PAccInput& input);

  /**
   * @brief 根据最优的发动机扭矩 计算 得出 目标的加速度
   */
  Float32_t GetNextAx() const;

  /**
   * @brief 根据最优的目标车速 计算 得出 目标的发动机转速
   */
  Float32_t GetNextNe() const;
  
  /**
   * @brief 获取目标的速度
   */
  Float32_t GetNextVel() const {
    // prediction_vel_points_num 至少为 1 因为一个数据为当前车的状态信息
    int prediction_vel_points_num = prediction_vel_points_.Size();
    Float32_t best_next_vel;

    if (prediction_vel_points_num >= 2) {
      best_next_vel = prediction_vel_points_[1];
    } else {
      best_next_vel = prediction_vel_points_[0];
    }
    
    return best_next_vel;
  } 

  /**
   * @brief 获取目标的最后的速度
   */
  Float32_t GetFinalVel() const {
    return prediction_vel_points_.Back();
  } 

  Float32_t GetNextDriveForce() const {
    // prediction_vel_points_num 至少为 1 因为一个数据为当前车的状态信息
    int prediction_Ft_points_num = prediction_Ft_points_.Size();
    Float32_t best_next_Ft;

    if (prediction_Ft_points_num >= 2) {
      best_next_Ft = prediction_Ft_points_[1];
    } else {
      best_next_Ft = prediction_Ft_points_[0];
    }
    return best_next_Ft;
  }

  Float32_t GetNextNextDriveForce() const {
    int prediction_Ft_points_num;
    Float32_t best_next_next_Ft;

    if (!prediction_Ft_points_.Empty()) {
      prediction_Ft_points_num = prediction_Ft_points_.Size();
    }

    if (prediction_Ft_points_num <= 2) {
      best_next_next_Ft = GetNextDriveForce();
    } else {
      best_next_next_Ft = prediction_Ft_points_[2];
    }

    return best_next_next_Ft;
  }
   
  Float32_t GetFinalLambda() const {
    return prediction_lambda_points_.Back();
  }

  int GetMpcSolutionState() const {
    return mpc_solution_state_;
  }
  
  static const Int32_t PredictionPoints = 500;

  const common::StaticVector<Float32_t, PredictionPoints> &GetBestStationPoints() const {
    return prediction_station_points_;
  }

  const common::StaticVector<Float32_t, PredictionPoints> &GetBestSlopeAlphaPoints() const {
    return prediction_slope_alpha_points_;
  }

  const common::StaticVector<Float32_t, PredictionPoints> &GetBestVelPoints() const {
    return prediction_vel_points_;
  }

  const common::StaticVector<Float32_t, PredictionPoints> &GetBestFtPoints() const {
    return prediction_Ft_points_;
  }

  const common::StaticVector<Float32_t, PredictionPoints> &GetBestlamdbaPoints() const {
    return prediction_lambda_points_;
  }
 

private: 
  /**
   * @brief 求解哈密顿函数的系数矩阵 H[x] = A*x^2 + B*x + C
   * @param vel 上一步求解出的最优车速 [m/s]
   * @param lambda 上一步求解出的最优协状态变量 [-]
   */
  void ComputeHamiltonCoefficient(const Float32_t &vel, const Float32_t &lambda);

  /**
   * @brief 根据求解 A B C 系数 和 x 的上下边界 x 的极小值
   *        求解最优扭矩 、最优车速 和 最优的协状态变量
   * @param cur_vel 当前车速 [m/s]
   * @param air_acc 空气阻力 [N]
   * @param roll_acc 滚动阻力 [N]
   * @param slope_acc 坡度阻力 [N]
   * @param best_torque 求解出的下一步最优的扭矩 [N/m]
   * @param best_v 求解出的下一步最优的车速 [m/s]
   * @param best_lambda 求解出的下一步最优的协状态变量(或者对偶变量) [-]
   */
  void SolverOptimalSolution(const Float32_t &cur_vel, const Float32_t &air_acc,
                             const Float32_t &roll_acc, const Float32_t &slope_acc,
                             Float32_t &best_Ft, Float32_t &best_v, Float32_t &best_lambda);

  /**
   * @brief 根据求解出的最优的 驱动力矩 和 车速 推算出 下一时刻 车辆位置对应坡度值 同时根据设定的车速限制
   *        进行最优驱动力矩的限制
   * @param cur_vel 当前车速 [m/s]
   * @param air_acc 空气阻力 [N]
   * @param roll_acc 滚动阻力 [N]
   * @param slope_acc 坡度阻力 [N]
   * @param best_torque 求解根据上下限制的下一步最优的扭矩 [N/m]
   * @param best_v 求解根据上下限制的求解出的下一步最优的车速 [m/s]
   * @param next_slope_alpha 根据最优的车速推算出坡度 [rad]
   */
  void LongitudeStatePrediction(const Float32_t &cur_vel, const Float32_t &air_acc,
                                const Float32_t &roll_acc, const Float32_t &slope_acc,
                                Float32_t &best_vel, Float32_t &best_Ft, Float32_t &best_Fb, 
                                Float32_t &next_slope_alpha, Float32_t &next_station);

class FuncCmpPACCPoint {
public:
  inline bool operator()(const PACCPoint& slope_a,
                    const PACCPoint& slope_b) const {
    if (slope_a.s > slope_b.s) {
      return true;
    }
    return false;
  }
};

private:
  // 车辆相关的参数
  const LongitudinalDynamicModelParam *vehparam_;
  const FuelRateModelParam* fuel_rate_param_;
  const VehicleConstraintSet* constraint_;

  // MPC相关参数
  const PccModelParameter *mpc_param_;
  // 质量换算系数
  Float32_t delta_;
  // 设定的巡航车速
  Float32_t v_ref_;
  
  // 坡度信息-[rad]
  Float32_t prediction_alpha_;
    // 预测位置点
  Float32_t prediction_station_;

  // 哈密顿函数的系数A
  Float32_t A_;
  // 哈密顿函数的系数B
  Float32_t B_;
  
  const PAccInput* input_;

  /// 地图最长的station
  Float32_t map_s_length_;

  // 当前变速箱传动比
  Float32_t cur_ig_ratio_;
  // 当前所在坡度值
  Float32_t cur_slope_rad_; 
  // MPC 求解状态
  Int8_t mpc_solution_state_;

  // 预测点 位置
  common::StaticVector<Float32_t, PredictionPoints> prediction_station_points_;
  // 预测点 坡度/弧度
  common::StaticVector<Float32_t, PredictionPoints> prediction_slope_alpha_points_;

  // 最优的驱动力
  common::StaticVector<Float32_t, PredictionPoints> prediction_Ft_points_;
  // 最优的制动力
  common::StaticVector<Float32_t, PredictionPoints> prediction_Fb_points_;
  
  // 最优的车速
  common::StaticVector<Float32_t, PredictionPoints> prediction_vel_points_;

  common::StaticVector<Float32_t, PredictionPoints> prediction_ax_points_;

  // 最終的lambda值
  common::StaticVector<Float32_t, PredictionPoints> prediction_lambda_points_;

  Float32_t temp_cur_ft_min_;
  Float32_t temp_cur_ft_max_;
};

/**
 * @struct DichotomySolution
 * @brief 
 * 
 */
class DichotomySolution {
public:
  /**
 * @brief constructor
 */
  DichotomySolution();
  
  /**
 * @brief destructor
 */
  ~DichotomySolution();


  void Clear();

  /**
 * @brief PCC 计算
 * @param is_first_pcc_mode 是否是pcc的第一帧 
 * @param data_source 用于获取轨迹规划 行为规划的结果
 * @param is_obstacle 是否有障碍物
 * @param obstacle 障碍物的相关信息
 */
  void Plan(const PAccInput& input,
                          const bool &is_obstacle, const ad_msg::Obstacle &obstacle);

  bool GetResultVaild() const {
      
      bool temp_vaild = false;

      if(is_pre_predict_valid_) {
        temp_vaild = dichotomy_solution_count_.GetResultState();
      } else {
        temp_vaild = frist_pcc_result_valid_;
      }

      return temp_vaild;
  }

  Float32_t GetTargetThrottle();

  /**
   * @brief 在控制油门的状态的时候，在起步初期出现油门的抖动---->二分法出现超出迭代的次数
   *        对应的目标扭矩 小于等于 0
  */
  void UpdateTargetTorqueAndNe();
  
  inline Uint8_t GetTargetGear() const {
      return target_gear_;
  }
  
  bool GetTargetGearIsVaild() const {
    bool temp_vaild = false;
    
    if (target_gear_ > 0 && target_gear_ < 13) {
      temp_vaild = true;
    }
    return temp_vaild;
  }

  inline Float32_t GetTargetVel() const {
      return target_vel_;
  }

  inline Float32_t GetTargetUlimitVel() const {
      return target_unlimit_vel_;
  }

  inline Float32_t GetTargetTorque() const {
      return target_torque_;
  }

  inline Float32_t GetTargetDriveForce() const {
      return target_Ft_;
  }

  inline Float32_t GetTargetNe() const {
      return target_ne_;
  }

  inline Float32_t GetTargetAx() const {
      return target_ax_;
  }

  inline int GetDichotomySolutionState() const {
      return dichotomy_solution_state_;
  }

  inline int GetMiddleMpcSolutionState() const {
      return middle_model_prediction_.GetMpcSolutionState(); 
  }

  inline int GetPccVelPlanType() const {
      return cur_pcc_mode_;  
  }

  Float32_t GetCurrentResistance() const;

  Float32_t GetCurrentFf() const;

  Float32_t GetCurrentFw() const;

  Float32_t GetCurrentFi() const;

private:
  /**
  * @brief 是决策PCC子状态  纯巡航、跟车模式(不踩刹车)、滑行模式(不踩刹车、不踩油门)、制动模式 
  *        根据 主目标车 位置 决策 对应的 状态
  * 
  * |+++++++++|             safe_distance
  * |   ego   |-------------------|-----------------------|--------------------------|
  * |+++++++++|                                     follow_distance        cc_to_follow_distance
  *                         |-----|                 |-----|                    |-----|
  *                    redundance_distance     redundance_distance        redundance_distance
  *
  * @param is_obstcle 是否有障碍物
  * @param obstacle 障碍物相关信息
  */
  Uint8_t PccModeChange(const bool &is_obstacle, const ad_msg::Obstacle &obstacle, const Float32_t &cc_target_vel);

  /**
  * @brief 在跟车状态用于计算最終参考车速
  * @param obstacle 障碍物的相关信息
  */
  Float32_t CalculateReferenceByFollow(const ad_msg::Obstacle &obstacle, const Float32_t &cc_vef);

  /**
   * @brief 根据目标车速调整 终端的 COST权重
   */
  void UpdateParamOnCC(const Float32_t &target_vel);

  /**
   * @brief 平滑PCC计算的扭矩
   */
  void TargetTorqueLimit();

  void TargetAxLimitBaseTorque(Uint8_t gear_num);

  void TargetVxLimitBaseTorque();

  void TargetNeLimitBaseTorque(Uint8_t gear_num);

  void UpdateTargetTorqueFromDriveForce(Uint8_t gear_num);

  Uint8_t UpdateTargetGear();

  Uint8_t Two_Lookup_Table(float x, float y, const float x_table[],
                           const float y_table[], const uint8_t table[], 
                           const uint32_t maxIndex_x,const uint32_t maxIndex_y);

  Float32_t UpdateDriveForceFromTarget(const Float32_t cur_engine_speed, 
                                       const Float32_t cur_engine_torque, const Float32_t cur_ego_speed);

  // TODO: 生成速度，驱动力，档位，扭矩，转速，燃油消耗率等数据表
  void Init();
  
private:
  const PAccInput* input_;

  // 配置参数
  LongitudinalDynamicModelParam lon_dyn_param_;

  FuelRateModelParam fuel_rate_param_;

  VehicleConstraintSet constraint_;
  
  VehicleConstraintSet pre_constraint_;

  PccResultCount dichotomy_solution_count_;

  KickdownCount kickdown_count_;
  // 模型预测的参数
  PccModelParameter follow_param_mpc_;
  PccModelParameter cc_param_mpc_;
  PccModelParameter param_mpc_;
  // 跟车相关的参数
  PccFollowParameter follow_param_;

  // 模型预测的对象
  LongitudinalModelPrediction left_model_prediction_;
  LongitudinalModelPrediction right_model_prediction_;
  LongitudinalModelPrediction middle_model_prediction_;

  // 二分法最大迭代次数
  Uint8_t dichotomy_max_iteration_;
  // 二分法最优解的容忍度
  Float32_t dichotomy_eps_;
  // 二分法lambda的左值的值
  Float32_t PCC_FGC_lambda_L_;
  // 二分法lambda的右值的值
  Float32_t PCC_FGC_lambda_U_;
  // 二分法lambda初始化左右值相差限制
  Float32_t detla_lambda_limit_;

  // 二分法lambda的初值状态
  Float32_t PCC_FGC_lambda_L0_;
  Float32_t PCC_FGC_lambda_U0_;
  Float32_t best_lambda_;
  Float32_t pre_lambda_;

  // 二分法F(lambda)的左值
  Float32_t Term_L_;
  // 二分法F(lambda)的右值
  Float32_t Term_U_;
  // 二分法F(lambda)的中间值
  Float32_t F_lambda_;
  // 二分法初始迭代方式
  Float32_t diff_;
  // 二分法放大步长
  Float32_t diff_Lmt_;

  // pcc当前模式
  Uint8_t cur_pcc_mode_;
  // pcc上一时刻的模式
  Uint8_t pre_pcc_mode_;

  // 权重系数
  Float32_t final_vel_weight_;
  // 参考车速
  Float32_t vx_ref_;

  bool is_control_throttle_;
  bool is_pre_predict_valid_;
  bool is_target_torque_delta_limit_;
  bool is_target_torque_avoid_jitter_;
  bool is_cur_engine_speed_;
  bool is_limit_speed_undulate_range_;
  bool is_use_previous_lambda_;

  bool frist_pcc_result_valid_;

  bool is_kickdown_control_;
  
  bool is_consider_target_gear_;

  Float32_t limit_speed_undulate_range_;

  Float32_t pre_target_vel_;
  Float32_t pre_target_ax_;

  Float32_t pre_target_torque_;
  Float32_t pre_target_ne_;
  Float32_t pre_target_Ft_;
  Uint8_t pre_target_gear_;

  Float32_t pre_target_throttle_;

  bool pre_kickdown_flag_;

  // 最优解
  Int8_t dichotomy_solution_state_;
  Float32_t target_vel_;
  Float32_t target_unlimit_vel_;
  Float32_t target_ax_;

  Float32_t target_torque_;
  Float32_t target_Ft_;
  Float32_t target_ne_;
  Uint8_t target_gear_;

  Uint8_t pre_chassis_gear_num_;

  Float32_t pcc_cruise_vel_deviation_limit_;

};

/**
 * @brief planning/predict target velocity 
 * @details optimal control based algorithm to implement PACC function:
 * 1. define objective/cost function
 * 2. design vehicle dynamic model / state equation
 * 3. design energy consumption model
 * 4. define constraint set
 * 5. implement Optimal Control Problem Solutions
 */
class VelocityPlanningPACC {
public:
  VelocityPlanningPACC();
  ~VelocityPlanningPACC();

  bool IsEnable();
  void Clear();

  /**
   * @brief PACC to different scenarios (need different parameters) : 
   * 1. straight road
   * 2. slope road
   * 3. curved road
   * 4. curved on lane change
   * 5. following
   * 
   * @param input 
   * @return PAccOutput 
   */
  PAccOutput Plan(const PAccInput& input);

  // TODO: need more knowledge and practice
  PAccOutput PlanOnStraightRoad(const PAccInput& input);
  PAccOutput PlanOnSlopeRoad(const PAccInput& input);
  PAccOutput PlanOnCurvedRoad(const PAccInput& input);
  PAccOutput PlanOnChangeLaneRoad(const PAccInput& input);

private:

  DichotomySolution solution_;
};

} // planning
} // phoenix

#endif // PHOENIX_PLANNING_VELOCITY_PLANNING_PACC_H_
