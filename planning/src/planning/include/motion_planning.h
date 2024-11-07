#ifndef PHOENIX_PLANNING_MOTION_PLANNING_H_
#define PHOENIX_PLANNING_MOTION_PLANNING_H_

#include "container/static_vector.h"
#include "curve/path.h"
#include "driving_map_wrapper.h"
#include <cstdint>

#define PLANNING_DEBUG (1)

#if (ENABLE_ROS_NODE)
#include "planning/plan_debug.h"
#endif
#include "pcc_map/adasis_v2.h"

namespace phoenix {
namespace planning {


/// Planning模块基本数据类型
typedef Float32_t plan_var_t;

/**
 * @enum MAX_LERP_TABLE_SIZE
 * @brief 插值用的表的最大长度
 */
enum { MAX_LERP_TABLE_SIZE = 20 };

/**
 * @enum MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM
 * @brief 轨迹规划中最大纵向采样的数量
 */
enum { MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM = 4 };

/**
 * @enum MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM
 * @brief 轨迹规划中最大link的数量
 */
enum { MAX_GRAPH_LINK_NUM = 200 };

/**
 * @enum MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM
 * @brief 轨迹规划中最大候选轨迹的数量
 */
enum { MAX_CANDIDATE_TRAJECTORY_NUM = 200 };

enum { MAX_LAT_OFFSET_NUM_FOR_SINGLE_LANE = 2 };

/**
 * @enum
 * @brief 纵向控制模块自适应跟车补偿表的尺寸
 */
enum { VELOCITY_PLANNING_ACC_FEED_TABLE_SIZE = 8 };

/**
 * @enum
 * @brief 路网构建类型
 */
enum {
  /// 无效的类型
  ROAD_BUILED_TYPE_INVALID = 0,
  /// 从高精度地图中构建
  ROAD_BUILED_TYPE_HD_MAP,
  /// 从相机车道线中构建
  ROAD_BUILED_TYPE_CAMERA_LANE,
  /// 从融合了相机车道线及高精度地图中构建
  ROAD_BUILED_TYPE_MIXED_HD_MAP,
  /// 从跟随前车轨迹中构建
  ROAD_BUILED_TYPE_FOLLOWING,
  /// 从根据当前车辆状态预测的轨迹中构建
  ROAD_BUILED_TYPE_PRED_PATH
};

/**
 * @enum
 * @brief 生成的轨迹类型
 */
enum {
  /// 无效的类型
  TRJ_STATUS_INVALID = 0,
  /// 车道保持
  TRJ_STATUS_LKA,
  /// 向左绕行
  TRJ_STATUS_LKA_BYPASSING_LEFT,
  /// 向右绕行
  TRJ_STATUS_LKA_BYPASSING_RIGHT,
  /// 向左变道(还未变更到左侧车道)
  TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I,
  /// 向左变道(已经变更到左侧车道)
  TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II,
  /// 向右变道(还未变更到右侧车道)
  TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I,
  /// 向右变道(已经变更到右侧车道)
  TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II,
  /// 中止左变道，回到右车道（还未变更到右侧车道）
  TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I,
  /// 中止左变道，回到右车道（已经变更到右侧车道）
  TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II,
  /// 中止向右变道，回到左车道（还未变更到左侧车道）
  TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I,
  /// 中止向右变道，回到左车道（已经变更到左侧车道）
  TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II
  /// TODO: 增加靠边停车、紧急避让等类型
};

/**
 * @enum
 * @brief 事件ID
 */
enum {
  /// 无事件
  EVENT_ID_NONE = 0,
  /// 请求变道
  EVENT_ID_REQ_CHANGING_LANE,
  /// 对变道请求的响应
  EVENT_ID_RSP_CHANGING_LANE
};

/**
 * @enum
 * @brief 拒绝变道的原因
 */
enum {
  /// 未知原因
  REFUSE_CHANGINHG_LANE_REASON_NONE = 0,
  /// 有障碍物
  REFUSE_CHANGINHG_LANE_REASON_OBSTACLE,
  /// 有不确定障碍物
  REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE,
  /// 无效车道
  REFUSE_CHANGINHG_LANE_REASON_INVALID_LANE,
  /// 车道线质量差
  REFUSE_CHANGINHG_LANE_REASON_LOW_QUALITY_LANE,
  /// 实线禁止变道
  REFUSE_CHANGINHG_LANE_REASON_SOLID_BOUNDARY,
  /// 隧道内禁止变道
  REFUSE_CHANGINHG_LANE_REASON_IN_TUNNEL,
  /// 弯道禁止变道
  REFUSE_CHANGINHG_LANE_REASON_IN_CURVE,
  /// 经过匝道，不向匝道变道
  REFUSE_CHANGINHG_LANE_REASON_NEAR_RAMP,
  /// 人工请求中止变道
  REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER,
  /// 当前速度禁止变道
  REFUSE_CHANGINHG_LANE_REASON_OUT_OF_SPEEDLIMIT
};


enum {
  REQ_FallBack_NONE = 0,
  REQ_FallBack_A,
  REQ_FallBack_B,
  REQ_FallBack_C,
  REQ_FallBack_D
};

enum {
  REQ_FALLBACK_ACTION_NONE = 0,
  REQ_FALLBACK_ACTION_B_II,
  REQ_FALLBACK_ACTION_C_I,
  REQ_FALLBACK_ACTION_C_II
};

/**
 * @struct ActionPlanningConfig
 * @brief 行为规划配置项
 */
struct ActionPlanningConfig {
  /// 允许自主变道
  bool enable_auto_changing_lane;
  /// 允许强制变道
  bool enable_action_planning_force_changing_lane;

  /**
   * @brief 构造函数
   */
  ActionPlanningConfig() {
    enable_auto_changing_lane = false;
    enable_action_planning_force_changing_lane = false;
  }
};


/**
 * @struct TrajectoryPlanningConfig
 * @brief 轨迹规划配置项
 */
struct TrajectoryPlanningConfig {
  /// 允许变道
  bool enable_changing_lane;
  /// 允许车道内避让
  bool enable_avoiding_collision_in_lane;
  /// 允许跨线避让
  bool enable_avoiding_collision_over_line;
  /// 允许强制变道
  bool enable_trajectory_planning_force_changing_lane;

  /**
   * @brief 构造函数
   */
  TrajectoryPlanningConfig() {
    enable_changing_lane = true;
    enable_avoiding_collision_in_lane = false;
    enable_avoiding_collision_over_line = false;
    enable_trajectory_planning_force_changing_lane = false;
  }
};

/**
 * @struct VelocityPlanningConfig
 * @brief 速度规划配置项
 */
struct VelocityPlanningConfig {
  /// 使能AEB预减速的功能
  bool enable_aeb_pre_dec;
  /// 到静态障碍物的安全距离
  plan_var_t safe_dist_to_static_obj;
  /// 低速跟随的安全距离
  plan_var_t safe_dist_for_low_speed_following;
  /// 到动态障碍物的安全距离
  plan_var_t safe_dist_to_dynamic_obj;
  /// 到障碍物的安全距离 (In backing mode)
  plan_var_t safe_distance_in_backing_mode;
  /// 精准停车
  struct StopAccurately {
    /// 使能
    bool enable;
    /// 制动开始距离
    plan_var_t braking_distance;
    /// 建议的减速度
    plan_var_t proposed_decelaration;

    /**
     * @brief 构造函数
     */
    StopAccurately() {
      enable = false;
      braking_distance = 0.8F;
      proposed_decelaration = 3.0F;
    }
  } stop_accurately;

  /// 侧向加速度限制
  plan_var_t lat_acc_limit;

  /// 非地图模式下红绿灯停车使能
  bool enable_stop_by_traffic_light;

  /**
   * @brief 构造函数
   */
  VelocityPlanningConfig() {
    enable_aeb_pre_dec = false;
    safe_dist_to_static_obj = 8.0F;
    safe_dist_for_low_speed_following = 5.0F;
    safe_dist_to_dynamic_obj = 8.0F;
    safe_distance_in_backing_mode = 2.0F;
    lat_acc_limit = 1.0F;
    enable_stop_by_traffic_light = false;
  }
};


/**
 * @struct EventPlanning
 * @brief 规划事件
 */
class EventPlanning {
public:
  explicit EventPlanning(Int32_t id) : id_(id) {}
  virtual ~EventPlanning() {
    // nothing to do
  }

  EventPlanning(const EventPlanning& other) {
    id_ = other.id_;
  }

  void operator =(const EventPlanning& other) {
    id_ = other.id_;
  }

  Int32_t id() const { return (id_); }

private:
  Int32_t id_;
};

/**
 * @struct ChangingLaneReq
 * @brief 变道请求
 */
class ChangingLaneReq : public EventPlanning {
public:
  enum {
    REQ_CHANGING_LANE_NONE,
    REQ_CHANGING_LANE_LEFT,
    REQ_CHANGING_LANE_RIGHT,
    REQ_CHANGING_LANE_ABORT
  };

public:
  ChangingLaneReq() :
    EventPlanning(EVENT_ID_REQ_CHANGING_LANE) {
    Clear();
  }

  void Clear() {
    request_ = REQ_CHANGING_LANE_NONE;
    sequence_ = -1;
    refuse_reason_ = REFUSE_CHANGINHG_LANE_REASON_NONE;
    allow_auto_changing_to_left_ = false;
    allow_auto_changing_to_right_ = false;
  }

  Int32_t request() const { return (request_); }
  void set_request(Int32_t req) { request_ = req; }

  Int32_t sequence() const { return (sequence_); }
  void set_sequence(Int32_t seq) { sequence_ = seq; }

  Int32_t refuse_reason() const { return ( refuse_reason_); }
  void set_refuse_reason(Int32_t reason) { refuse_reason_ = reason; }

  bool allow_auto_changing_to_left() const {
    return (allow_auto_changing_to_left_);
  }
  void set_allow_auto_changing_to_left(bool allow) {
    allow_auto_changing_to_left_ = allow;
  }
  bool allow_auto_changing_to_right() const {
    return (allow_auto_changing_to_right_);
  }
  void set_allow_auto_changing_to_right(bool allow) {
    allow_auto_changing_to_right_ = allow;
  }

private:
  Int32_t request_;
  Int32_t sequence_;
  Int32_t refuse_reason_;
  bool allow_auto_changing_to_left_;
  bool allow_auto_changing_to_right_;
};

/**
 * @struct ChangingLaneRsp
 * @brief 对变道请求的响应
 */
class ChangingLaneRsp : public EventPlanning {
public:
  enum {
    RSP_CHANGING_LANE_NONE,
    RSP_CHANGING_LANE_ACCEPT,
    RSP_CHANGING_LANE_REFUSE
  };

  enum {
    STATUS_CHANGING_LANE_NONE = 0,
    STATUS_CHANGING_LANE_LEFT_STAGE_I,
    STATUS_CHANGING_LANE_LEFT_STAGE_II,
    STATUS_CHANGING_LANE_RIGHT_STAGE_I,
    STATUS_CHANGING_LANE_RIGHT_STAGE_II,
    STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I,
    STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II,
    STATUS_ABORT_CHANGING_LANE_TO_LEFT_I,
    STATUS_ABORT_CHANGING_LANE_TO_LEFT_II,
  };

public:
  ChangingLaneRsp() :
    EventPlanning(EVENT_ID_RSP_CHANGING_LANE) {
    Clear();
  }

  void Clear() {
    response_ = RSP_CHANGING_LANE_NONE;
    requset_sequence_ = -1;
    status_ = STATUS_CHANGING_LANE_NONE;
    refuse_reason_ = REFUSE_CHANGINHG_LANE_REASON_NONE;
  }

  Int32_t response() const { return (response_); }
  void set_response(Int32_t rsp) { response_ = rsp; }

  Int32_t requset_sequence() const { return (requset_sequence_); }
  void set_requset_sequence(Int32_t seq) { requset_sequence_ = seq; }

  Int32_t status() const { return (status_); }
  void set_status(Int32_t stu) { status_ = stu; }

  Int32_t refuse_reason() const { return (refuse_reason_); }
  void set_refuse_reason(Int32_t reason) { refuse_reason_ = reason; }

private:
  Int32_t response_;
  Int32_t requset_sequence_;
  Int32_t status_;
  Int32_t refuse_reason_;
};

/**
 * @struct ActionPlanningResult
 * @brief 行为规划结果
 */
struct ActionPlanningResult {
  /// 消息头
  ad_msg::MsgHead msg_head;

  /// ADAS 使能
  bool enable_adas;
  /// AEB 使能
  bool enable_aeb;
  /// ACC 使能
  bool enable_acc;
  /// LKA 使能
  bool enable_lka;

  /// 变道功能 使能
  bool enable_alc;
  /// 智能限速功能 使能
  bool enable_isl;

  /// Navigation Guided Pilot 使能
  bool enable_ngp;
  /// 降级 使能
  bool enable_fallback;
  /// 节油 使能
  bool enable_pcc;
  
  /// 目标车速, [m/s]
  Float32_t v_setting;
  /// 目标加速度, [m/s^2]
  Float32_t a_setting;
  /// 目标降级 [-]
  Int8_t level_setting;
  /// 降级行为 [-]
  Int8_t fallback_action;
  /// 预设 time gap 时间, [s]
  Float32_t time_gap_setting;

  /// 指令驾驶模式
  Int8_t driving_mode;
  /// 指令档位
  Int8_t gear;
  /// 指令转向灯
  Int8_t turn_lamp;

  /// 避让请求
  struct {
    /// 允许允许向左避让
    bool allow_turning_left;
    /// 允许向右避让
    bool allow_turning_right;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      allow_turning_left = false;
      allow_turning_right = false;
    }
  } bypassing_req;
  /// 变道请求
  ChangingLaneReq changing_lane_req;
  
  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    enable_adas = false;
    enable_aeb = true;
    enable_acc = true;
    enable_lka = true;

    enable_alc = false;
    enable_isl = false;

    enable_ngp = false;
    enable_fallback = false;
    enable_pcc = false;
    
    v_setting = 0.0F;
    a_setting = 0.0F;
    level_setting = 0;
    fallback_action = 0;
    time_gap_setting = 3.0F;

    driving_mode = ad_msg::VEH_DRIVING_MODE_INVALID;
    gear = ad_msg::VEH_GEAR_INVALID;
    turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;

    bypassing_req.Clear();
    changing_lane_req.Clear();
  }

  /**
   * @brief 构造函数
   */
  ActionPlanningResult() {
    Clear();
  }
};

/**
 * @struct TrajectoryPlanningResult
 * @brief 轨迹规划结果
 */
struct TrajectoryPlanningResult {
  /// 定义轨迹方向
  enum TrjDirection {
    TRJ_DIRECTION_FORWARD = 0,
    TRJ_DIRECTION_BACKWARD
  };

  /// 消息头
  ad_msg::MsgHead msg_head;

  /// 道路类型
  Int32_t road_type;
  /// 轨迹状态
  Int32_t trj_status;

  /// 当前车辆位置 & 先导点位置
  struct {
    /// 坐标x
    plan_var_t x;
    /// 坐标y
    plan_var_t y;
    /// 航向角, Range: (-pi, pi rad)
    plan_var_t heading;
    /// 曲率
    plan_var_t curvature;
    /// 相对于参考线的路径长
    plan_var_t s;
    /// 相对于参考线的横向偏移
    plan_var_t l;
  } curr_pos, leading_pos;

  /// 横向误差
  struct {
    /// 目标轨迹是否处于避障或变道状态
    Int8_t moving_flag;
    struct {
      /// 横向误差
      plan_var_t lat_err;
      /// 横向误差变化速率
      plan_var_t lat_err_chg_rate;
      /// 角度误差 Range: (-pi, pi rad)
      plan_var_t yaw_err;
      /// 角度误差变化速率
      plan_var_t yaw_err_chg_rate;
    } samples[2];
  } lat_err;

  /// 目标轨迹的方向
  Int32_t trj_direction;
  /// 目标轨迹
  common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> target_trajectory;
  /* k006 pengc 2023-01-06 (begin) */
  /// 轨迹上的坡度
  struct SlopeSamplePoint {
    /// 相对于参考线的路径长
    plan_var_t s;
    /// 坡度值
    plan_var_t slope;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      s = 0.0F;
      slope = 0.0F;
    }

    /**
     * @brief 构造函数
     */
    SlopeSamplePoint() {
      Clear();
    }
  } slope_point;
  common::StaticVector<SlopeSamplePoint,
      common::Path::kMaxPathPointNum> target_trajectory_slope;
  /* k006 pengc 2023-01-06 (end) */
  /* k001 pengc 2022-04-13 (begin) */
  // 优化变道时减速的问题
  /// 指出目标轨迹正在改变
  struct {
    /// 目标轨迹是否正在改变
    bool is_changing;
    /// 目标轨迹距离最终轨迹的横向偏差
    plan_var_t lat_offset;
    /// 轨迹规划选中的轨迹
    common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> selected_trajectory;
    /// 轨迹规划返回原车道的轨迹
    common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> return_trajectory;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      is_changing = false;
      lat_offset = 0.0F;
      selected_trajectory.Clear();
      return_trajectory.Clear();
    }
  } trj_changing;
  /* k001 pengc 2022-04-13 (end) */

  /// 保持方向盘不动
  bool hold_steering_wheel;
  /// 方向盘转速限制 (rad/s)
  Float32_t steering_wheel_speed;
  /// 主动请求变道
  ChangingLaneReq changing_lane_req;
  /// 对变道指令的响应
  ChangingLaneRsp changing_lane_rsp;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    road_type = ROAD_BUILED_TYPE_INVALID;
    trj_status = TRJ_STATUS_INVALID;

    curr_pos.x = 0.0F;
    curr_pos.y = 0.0F;
    curr_pos.heading = 0.0F;
    curr_pos.curvature = 0.0F;
    curr_pos.s = 0.0F;
    curr_pos.l = 0.0F;
    leading_pos.x = 0.0F;
    leading_pos.y = 0.0F;
    leading_pos.heading = 0.0F;
    leading_pos.curvature = 0.0F;
    leading_pos.s = 0.0F;
    leading_pos.l = 0.0F;

    common::com_memset(&lat_err, 0, sizeof(lat_err));

    trj_direction = TRJ_DIRECTION_FORWARD;
    target_trajectory.Clear();
    /* k006 pengc 2023-01-06 (begin) */
    target_trajectory_slope.Clear();
    /* k006 pengc 2023-01-06 (end) */
    trj_changing.Clear();

    hold_steering_wheel = false;
    steering_wheel_speed = 0.0F;
    changing_lane_req.Clear();
    changing_lane_rsp.Clear();
  }

  /**
   * @brief 构造函数
   */
  TrajectoryPlanningResult() {
    Clear();
  }
};


/**
 * @struct
 * @brief 速度控制目标类型
 */
enum {
  /// 无效类型
  VELOCITY_PLANNING_TARGET_TYPE_INVALID = 0,
  /// 根据用户设置控制车速
  VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS,
  /// 根据道路曲率控制车速
  VELOCITY_PLANNING_TARGET_TYPE_CURVATURE,
  /// 根据障碍物控制车速
  VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE,
  /// 跟随前车
  VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING,
  /// 低速跟随
  VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING,
  /// AEB告警
  VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN,
  /// 执行AEB
  VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION,
  /// 根据场景任务控制车速
  VELOCITY_PLANNING_TARGET_TYPE_SCENE_STORY,
  /// 根据交通灯控制车速
  VELOCITY_PLANNING_TARGET_TYPE_TRAFFIC_LIGHT,
  /// 根据限速牌控制车速
  VELOCITY_PLANNING_TARGET_TYPE_ISL,
  /// 根据隧道场景控制车速
  VELOCITY_PLANNING_TARGET_TYPE_TUNNEL,
  /// 根据降级控制车速
  VELOCITY_PLANNING_TARGET_TYPE_FALLBACK, //12
  /// 根据匝道场景控制车速
  VELOCITY_PLANNING_TARGET_TYPE_RAMP,
  /// 根据道路限速控制车速
  VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT,
  /// 开启节油模式
  VELOCITY_PLANNING_TARGET_TYPE_PCC
};

/**
 * @struct VehicleDynamicParam
 * @brief Define some constant values for calculating vehicle dynamics parameters
 * 
 */
struct VehicleDynamicParam
{
  /// 空气密度 [kg/m^3]
  Float32_t air_density;
  /// 风阻系数 [-]
  Float32_t Cd;
  /// 滚动阻力系数
  Float32_t f;
  /// 重力加速度
  Float32_t g;
  /// 机械传动效率 [-]
  Float32_t eta;
  /// 车轮滚动半径 [m]
  Float32_t Rw;
  /// 主减速器传动比 [-]
  Float32_t If;
  /// 迎风面积 [m^2]
  Float32_t Ad;
  
  // 变速箱传动比 - Vector
  common::StaticVector<Float32_t, 14> Ig;

  void Init() {
    air_density = 1.225;
    Cd = 0.5;
    f = 0.005;
    g = 9.8;    
    eta = 0.98;
    Rw = 0.5276;
    Ad = 8.1;
  #if (TM_YD)
    If = 3.42; 
  
    Ig.Clear();
    Ig.PushBack(0.0f);
    Ig.PushBack(14.43);
    Ig.PushBack(11.05);
    Ig.PushBack(8.44);
    Ig.PushBack(6.46);
    Ig.PushBack(4.95);
    Ig.PushBack(3.79);
    Ig.PushBack(2.91);
    Ig.PushBack(2.23);
    Ig.PushBack(1.70);
    Ig.PushBack(1.30);
    Ig.PushBack(1.0);
    Ig.PushBack(0.77);
#endif

#if (TM_ZF)
    If = 2.69;

    Ig.Clear();
    Ig.PushBack(0.0);
    Ig.PushBack(16.688);
    Ig.PushBack(12.924);
    Ig.PushBack(9.926);
    Ig.PushBack(7.688);
    Ig.PushBack(5.895);
    Ig.PushBack(4.565);
    Ig.PushBack(3.655);
    Ig.PushBack(2.831);
    Ig.PushBack(2.174);
    Ig.PushBack(1.684);
    Ig.PushBack(1.291);
    Ig.PushBack(1.0);
#endif
  }

  VehicleDynamicParam() {
    Init();
  }
};

/**
 * @struct
 * @brief 节油规划目标类型
*/
enum {
  // 无效类型
  VELOCITY_PLANNING_PCC_TYPE_INVALID = 0,
  // 纯巡航(不踩刹车)
  VELOCITY_PLANNING_PCC_TYPE_CRUISE,
  // 巡航(需踩刹车)
  VELOCITY_PLANNING_PCC_TYPE_CRUISE_BRAKE,
  // 跟车模式(不踩刹车)
  VELOCITY_PLANNING_PCC_TYPE_FOLLOW,
  // 滑行模式(不踩刹车、不踩油门)
  VELOCITY_PLANNING_PCC_TYPE_SLIDE, //slide
  // 制动模式
  VELOCITY_PLANNING_PCC_TYPE_BRAKE
};

/**
 * @struct PACCChassisInput
 * @brief PCC需要的底盘数据
 */
struct PACCChassisInput
{
  // 油门系统控制状态
  Int8_t throttle_sys_status;
  // 制动系统控制状态
  Int8_t ebs_status;
  // 油门踏板深度，%
  Int8_t acc_pedal_value;
  // 制动踏板深度，%
  Int8_t brake_pedal_value;
  // 最低巡航速度，m/s
  Float32_t cruise_speed;
  // 发动机转速，rpm
  Float32_t engine_speed;
  // 发动机转矩, nm
  // TODO: 此处的扭矩是否为减去摩擦扭矩后的输出扭矩？？？
  Float32_t engine_torque;

  Float32_t engine_torque_limit;
  // 当前档位 Number
  Int8_t gear_number;
  // 自车速度，m/s
  Float32_t ego_speed;
  // 自车加速度,m/s^2
  Float32_t ego_accel;
  // 总质量，kg
  Float32_t ego_mass;

  /**
   * @brief 清除内部数据
   */
  void Clear(){
    throttle_sys_status = 0;
    ebs_status = 0;
    acc_pedal_value = 0;
    brake_pedal_value = 0;
    engine_speed = 0.0F;
    engine_torque = 0.0F;
    gear_number = 0;
    ego_speed = 0.0F;
    ego_accel = 0.0F;
    ego_mass = 0.0F;
    engine_torque_limit = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  PACCChassisInput(){
    Clear();
  }
};

/**
 * @struct PACCPointDebug
 * @brief PCC地图数据
 */
struct PACCPointDebug {
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

  PACCPointDebug() {
    Clear();
  }

};

/**
 * @struct PACCMapInput
 * @brief PCC需要的地图数据
 */
struct PACCMapInput
{
  // Max PACC point number
  static const Uint16_t PACC_MAX_POINT_NUM = 500;
  // GNSS 状态
  Int8_t gnss_status;
  // 经度，[degree] [-180, 180]
  Float64_t longitude;
  // 纬度，[degree] [-90, 90]
  Float64_t latitude;
  // 高度，m
  Float64_t altitude;
  // 俯仰角，rad
  Float64_t pitch;
  // 横摆角，rad
  Float64_t yaw;
  // 横滚角，rad
  Float64_t roll;

  // MPU 状态
  Int8_t odom_status;
  // MPU x [m]
  Float64_t x_odom;
  // MPU y [m]
  Float64_t y_odom;
  // MPU z [m]
  Float64_t z_odom;
  // MPU heading [rad]
  Float64_t heading_odom;
  // PCC地图数据是否有效
  Int8_t valid;
  // 数据来源，0：无效，1：TBOX；2：MPU
  Int8_t pacc_points_source;
  // 采样步长
  Float32_t pacc_point_s_step;
  // 采样点数
  Int16_t pacc_point_num;
  // PCC地图数据
  common::StaticVector<PACCPointDebug, PACC_MAX_POINT_NUM> pacc_points;
  // 当前周期对应的坡度，%
  Float32_t cur_slope;
  // 当前周期预测坡度，%
  Float32_t preview_slope;

  Float32_t prediction_map_length;

  Float64_t total_map_length;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    gnss_status = 0;
    longitude = 0.0F;
    latitude = 0.0F;
    altitude = 0.0F;
    pitch = 0.0F;
    yaw = 0.0F;
    roll = 0.0F;

    odom_status = 0.0;
    x_odom = 0.0F;
    y_odom = 0.0F;
    z_odom = 0.0F;
    heading_odom = 0.0F;

    valid = 0;
    pacc_points_source = 0;
    pacc_point_s_step = 5.0F;
    pacc_point_num = 0;
    pacc_points.Clear();
    cur_slope = 0.0F;
    preview_slope = 0.0F;

    prediction_map_length = 0.0F;
    
    total_map_length = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  PACCMapInput() {
    Clear();
  }
};

/**
 * @struct MPCBestIteration
 * @brief 最优预测
 */
struct MPCBestIteration {
  Float32_t station;
  Float32_t slope_alpha;
  Float32_t ft;

  // 最优预测点的速度，m/s
  Float32_t velocity;
  // 最优预测点的加速度，m/s^2
  Float32_t accel;
  // 最优预测点的扭矩，nm
  Float32_t te;
  // 哈密顿协态量
  Float32_t lambda;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    station = 0.0F;
    slope_alpha = 0.0F;
    ft = 0.0F;

    velocity = 0.0F;
    accel = 0.0F;
    te = 0.0F;
    lambda = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  MPCBestIteration() {
    Clear();
  }
};

/**
 * @struct PACCInternal
 * @brief PCC内部计算数据
 */
struct PACCInternal
{
  // 最优预测最大点数
  static const uint16_t PCC_BEST_POINTS_NUM = 500;
  // mpc求解的最优的协状态变量
  Float32_t best_lambda;
  // 二分法迭代次数
  Uint8_t iteration_count;
  // 当前扭矩下的发动机最大转速
  Float32_t torque_max_base_cur_engine_speed;
  // 当前扭矩下的发动机最小转速
  Float32_t torque_min_base_cur_engine_speed;
  // 最优预测点数
  Int16_t pcc_best_point_num;
  // 最优预测
  common::StaticVector<MPCBestIteration, PCC_BEST_POINTS_NUM> pcc_best_points;
  // 最終的J
  Float32_t Final_J;
  // 最終的Q 
  Float32_t Final_Q;

  // MPC预测步长中间采样点数据
  Float32_t tar_sample_te;
  Float32_t tar_sample_ft;
  Float32_t tar_sample_a;
  Float32_t tar_sample_v;
  Float32_t tar_sample_ne;
  Float32_t tar_sample_throttle;

  Float32_t tar_throttle_from_table;

  /**
   * @brief 清除内部数据
   */
  void Clear(){
    best_lambda = 0.0F;
    Final_J = 0.0F;
    Final_Q = 0.0F;
    iteration_count = 0;
    torque_max_base_cur_engine_speed = 0.0F;
    torque_min_base_cur_engine_speed = 0.0F;
    pcc_best_point_num = 0;
    pcc_best_points.Clear();

    tar_sample_te = 0.0F;
    tar_sample_ft = 0.0F;
    tar_sample_a = 0.0F;
    tar_sample_v = 0.0F;
    tar_sample_ne = 0.0F;
    tar_sample_throttle = 0.0F;
    
    tar_throttle_from_table = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  PACCInternal(){
    Clear();
  }
};

/**
 * @struct PACCOutput
 * @brief PCC计算输出数据
 */
struct PACCOutput
{
  // PCC计算的目标速度，m/s
  Float32_t tar_v;
  Float32_t target_unlimit_vel;
  // PCC计算的目标加速度，m/s
  Float32_t tar_a;
  // PCC计算的目标挡位号
  Int8_t tar_gear;
  // PCC计算的目标扭矩 N/m
  Float32_t tar_engine_torque;
  // PCC计算的目标转速 rpm
  Float32_t tar_engine_ne;
  // PCC计算目标驱动力 N
  Float32_t tar_drive_force;
  // PCC当前阻力 N
  Float32_t cur_resistance;
  // PCC当前滚动阻力 N
  Float32_t cur_Ff;
  // PCC当前风阻 N
  Float32_t cur_Fw;
  // PCC当前坡阻 N
  Float32_t cur_Fi;
  // 目标发动机油门开度 [%]
  Float32_t tar_throttle;
  // PCC计算的速度规划类型
  Int8_t pcc_vel_plan_type;
  // 二分法计算状态
  Int8_t dichotomy_solution_state;
  // MPC计算状态
  Int8_t mpc_solution_state;

  Int8_t pcc_result_vaild;

  /**
   * @brief 清除内部数据
   */
  void Clear(){
    tar_v = 0.0F;
    tar_a = 0.0F;
    tar_gear = 0;
    tar_engine_torque = 0.0F;
    tar_drive_force = 0.0F;
    cur_resistance = 0.0F;
    cur_Ff = 0.0;
    cur_Fw = 0.0;
    cur_Fi = 0.0;
    tar_engine_ne = 0.0F;
    tar_throttle = 0.0F;
    pcc_vel_plan_type = 0;
    dichotomy_solution_state = 0;
    mpc_solution_state = 0;
    pcc_result_vaild = 0;
    target_unlimit_vel = 0.0;
  }

  /**
   * @brief 构造函数
   */
  PACCOutput(){
    Clear();
  }
};

/**
 * @struct
 * @brief PACC 相关的所有数据集合结构体
 */
struct PACC
{
  // PACC 功能使能与否
  Int8_t enable;
  // PACC 功能激活与否
  Int8_t valid;
  // PACC 收到道路类型 
  Int8_t road_type;
  // PCC需要的底盘数据
  PACCChassisInput chassis_input;
  // PCC需要的地图数据
  PACCMapInput map_input;
  // PCC内部计算数据
  PACCInternal internal;
  // PCC计算输出数据
  PACCOutput output;
  // 待定义
  PACCInternal inner;

  Float32_t slope_dec;

  Float32_t slope_length;
  Float32_t slope_smooth;
  Float32_t distance_to_slope;
  Float32_t cc_upper_limit_v;

  Float32_t tar_throttle_ufilter;

  Float32_t cur_slope_ufilter;
  
  Float32_t engine_speed_ufilter;
  /**
   * @brief 清除内部数据
   */
  void Clear(){
    enable = 0;
    valid = 0;
    road_type = 0;
    tar_throttle_ufilter = 0.0;
    engine_speed_ufilter = 0.0;
    cur_slope_ufilter = 0.0;
    chassis_input.Clear();
    map_input.Clear();
    internal.Clear();
    output.Clear();
    inner.Clear();
    slope_dec = 0.0f;
    slope_smooth = 0.0f;
    slope_length = 0.0f;
    distance_to_slope = 0.0f;
  }

  /**
   * @brief 构造函数
   */
  PACC(){
    Clear();
  }
};

/**
 * @struct ObstacleDecision
 * @brief 障碍物相关属性
 */
struct ObstacleDecision
{
  // 是否是风险障碍物
  bool is_risk;
  // 对应的感知模块的障碍物信息
  ad_msg::Obstacle perception_obstacle;
  // 障碍物按照9宫格划分的位置
  Int32_t obj_position;
  // 障碍物行驶方向(相对自车)
  Int32_t obj_direction;
  // 是否满足cutin定义
  bool is_cutin;
  // 等价于OBB碰撞检测的static_distance
  Float32_t obb_distance;

  // 是否处理该障碍物
  bool need_process;

  // 可跟随的障碍物，满足跟车要求：TYPE=4|5
  bool can_following;
  // 是否是CIPV，最终跟车目标
  bool is_cipv;

  // 是否是需要减速避免碰撞的障碍物：TYPE=3
  bool need_avoiding;
  // 是否是最优先需要减速避障的障碍物
  bool is_mio;

  // 是否是需要执行aeb warning的障碍物：TYPE=6
  bool need_aeb_warning;
  // 是否是被最终选择的执行aeb_warning的障碍物
  bool is_aeb_warning_target;

  // 是否是需要执行aeb full brake的障碍物：TYPE=7
  bool need_aeb_full_brake;
  // 是否是被最终选择的执行aeb full brake的障碍物
  bool is_aeb_full_brake_target;

  // 感知检测的障碍物距自车的纵向距离
  Float32_t dist_to_obj;
  // 不同场景下需要与障碍物保持的安全距离
  Float32_t safe_dist;
  // 规划算法的初始v
  Float32_t v_for_plan;
  // 时距，s
  Float32_t time_gap;
  // 相对时距 ttc，s
  Float32_t ttc;
  // 规划算法的终状态v，m/s
  Float32_t planning_target_v;
  // 规划算法的终状态s，m
  Float32_t planning_target_s;

  /**
   * @brief 清除内部数据
   */
  void Clear(){
    is_risk = false;
    perception_obstacle.Clear();
    obj_position = 0;
    obj_direction = 0;
    is_cutin = false;
    obb_distance = 0.0F;
    need_process = false;
    can_following = false;
    is_cipv = false;
    need_avoiding = false;
    is_mio = false;
    need_aeb_warning = false;
    is_aeb_warning_target = false;
    need_aeb_full_brake = false;
    is_aeb_full_brake_target = false;

    dist_to_obj = 0.0F;
    safe_dist = 0.0F;
    v_for_plan = 0.0F;
    time_gap = 0.0F;
    ttc = 100.0F;
    planning_target_v = 0.0F;
    planning_target_s = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  ObstacleDecision(){
    Clear();
  }
};

/**
 * @struct VelocityPlanningInternal
 * @brief 速度规划内部计算数据
 */
struct VelocityPlanningInternal {
  // 轨迹上的最大路点数
  static const Uint16_t MAX_PATH_POINT_NUM = 500;
  // 开始执行速度规划计算时的时间戳
  Int64_t timestamp;

  // 开始执行速度规划计算时的自车位置
  common::TrajectoryPoint planning_pos;

  // 路径规划的路径点数
  Int16_t planning_path_point_num;

  //路径规划的结果经过坐标变换后的路径
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> planning_path;

  //上一周期AD使能与否
  bool adas_enable_pre;
  //AD使能与否
  bool adas_enable;

  // 主参考线路径点个数
  Int16_t major_ref_line_point_num;
  //主参考线路径
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> major_ref_line;

  // 驾驶员设置的目标巡航速度，m/s
  Float32_t user_setting_v;
  // 基于规划计算的本周期的规划速度，m/s
  Float32_t current_plan_v;
  // 道路限速
  Float32_t upper_limit_v_road;
  // 隧道限速
  Float32_t upper_limit_v_tunnel;
  // 匝道限速
  Float32_t upper_limit_v_ramp;
  // 曲率限速
  Float32_t upper_limit_v_curvature;
  // 变道限速
  Float32_t upper_limit_v_lane_change;
  // 最终巡航限速
  Float32_t upper_limit_v_final;
  // 无障碍物场景，根据最终的限速计算的目标速度，m/s
  Float32_t tar_v_cruise;
  //无障碍物场景，根据最终的限速计算的目标加速度，m/s^2
  Float32_t tar_a_cruise;

  // 根据相机检测的车道线最长距离范围内的最大曲率
  Float32_t target_path_max_curvature;
  // 弯道场景，根据曲率计算的速度规划类型
  Int8_t tar_type_curvature;
  // 弯道场景，根据曲率计算的目标速度，m/s
  Float32_t tar_v_curvature;
  //弯道场景，根据曲率计算的目标加速度，m/s^2
  Float32_t tar_a_curvature;

  // 变道路径点个数
  Int16_t changlane_path_point_num;
  // 变道路径
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> changlane_path;

  /// 坡度限速 TODO:当前尚未定义专门的坡道场景
  // 目标坡度 %
  Float32_t slope;
  // 距离坡道的距离
  Float32_t distance_to_slope;
  // 坡道场景 计算出的速度规划类型
  Int8_t tar_type_slope;
  // 坡道场景 计算出的目标速度，m/s
  Float32_t tar_v_slope;
  // 坡道场景 计算出的目标加速度, m/s^2
  Float32_t tar_a_slope;

  /// 交通规则：红绿灯
  // 红绿灯类型，红、黄、绿
  Int8_t traffic_light_type;
  // 距离红绿灯的距离
  Float32_t distance_to_traffic_light;
  // 红绿灯场景 计算出的速度规划类型
  Int8_t tar_type_traffic_light;
  // 红绿灯场景 计算出的目标速度，m/s
  Float32_t tar_v_traffic_light;
  // 红绿灯场景 计算出的目标加速度, m/s^2
  Float32_t tar_a_traffic_light;

  /// 交通规则：限速标识牌
  // 限速类型 限速牌限速值
  Int8_t traffic_signal_type;
  // 距离限速牌的距离 m
  Float32_t distance_to_traffic_signal;
  // 限速标识牌场景 计算出的速度规划类型 
  Int8_t tar_type_traffic_signal;
  // 限速标识牌场景 计算出的目标速度，m/s
  Float32_t tar_v_traffic_signal;
  // 限速标识牌场景 计算出的目标加速度, m/s^2
  Float32_t tar_a_traffic_signal;

  /// 交通规则：匝道
  // 匝道场景 限速，m/s
  Float32_t ramp_velocity_limit;
  // 匝道场景 距离，m
  Float32_t distance_to_ramp;
  // 匝道场景，计算的速度规划类型
  Int8_t tar_type_ramp;
  // 匝道场景，计算的目标速度，m/s
  Float32_t tar_v_ramp;
  // 匝道场景，计算的目标加速度，m/s^2
  Float32_t tar_a_ramp;

  /// 交通规则：隧道
  // 隧道场景 限速，m/s
  Float32_t tunnel_velocity_limit;
  // 隧道场景 距离，m
  Float32_t distance_to_tunnel;
  // 隧道场景，计算的速度规划类型
  Int8_t tar_type_tunnel;
  // 隧道场景，计算的目标速度，m/s
  Float32_t tar_v_tunnel;
  // 隧道场景，计算的目标加速度，m/s^2
  Float32_t tar_a_tunnel;

  // 车道限速，m/s
  Float32_t lane_velocity_limit;
  // 距离道路限速点的距离，m
  Float32_t distance_to_lane_velocity_limit;
  // 车道限速场景，计算的速度规划类型
  Int8_t tar_type_lane;
  // 车道限速场景，计算的目标速度，m/s
  Float32_t tar_v_lane;
  // 车道限速场景，计算的目标加速度，m/s^2
  Float32_t tar_a_lane;

  /// 交通规则：final，将之视为虚拟障碍物
  // 虚拟障碍物类型
  Int8_t tar_type_virtual_obstacle;
  // 计算的目标速度，m/s
  Float32_t tar_v_virtual_obstacle;
  // 计算的目标加速度，m/s^2
  Float32_t tar_a_virtual_obstacle;

  // ODD Fallback场景，触发的ODD类型
  Int8_t fallback_type;
  // ODD Fallback场景场景，计算的速度规划类型
  Int8_t tar_type_fallback;
  // ODD Fallback场景场景，计算的目标速度，m/s
  Float32_t tar_v_fallback;
  // ODD Fallback场景场景，计算的目标加速度，m/s^2
  Float32_t tar_a_fallback;

  /// 速度规划：变道场景
  // 速度规划：障碍物
  Int16_t collision_test_path_point_num;
  // 碰撞检测生成路径
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> collision_test_path;
  
  Int16_t collision_test_samples_point_num;
  // 碰撞检测路径采样点s
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> collision_test_samples;

  // 速度规划：跟车
  // 跟车目标的所有数据
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

  // 速度规划：障碍物
  Int16_t collision_test_path_point_num_lca;
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> collision_test_path_lca;

  Int16_t collision_test_samples_point_num_lca;
  common::StaticVector<common::PathPoint, MAX_PATH_POINT_NUM> collision_test_samples_lca;
  // CollisionTestOnPathResult collision_test_result

  // 速度规划：跟车 ACC
  ObstacleDecision cipv_lca;

  Int8_t tar_type_following_lca;

  Float32_t tar_v_following_lca;

  Float32_t tar_a_following_lca;

  // 跟车场景五次多项式补丁状态
  Int8_t fsm_follow_state_lca;

  // 速度规划：避障 obstacle avoidance
  ObstacleDecision avoidance_lca;

  Int8_t tar_type_obstacle_lca;

  Float32_t tar_v_obstacle_lca;

  Float32_t tar_a_obstacle_lca;

  Int8_t fsm_obstacle_state_lca;

  // 速度规划：紧急制动 AEB
  // 最可能碰撞的障碍物
  ObstacleDecision mio_lca;

  Int8_t tar_type_aeb_lca;

  Float32_t tar_v_aeb_lca;

  Float32_t tar_a_aeb_lca;

  // 障碍物处理：final
  Int8_t tar_type_real_obstacle;

  Float32_t tar_v_real_obstacle;

  Float32_t tar_a_real_obstacle;

  //低速/拥堵跟车 TJA
  ObstacleDecision cipv_tja; // CIPV
  // 低速/拥堵跟车场景，计算的速度规划类型
  Int8_t tar_type_tja;
  // 低速/拥堵跟车场景，计算的目标速度，m/s
  Float32_t tar_v_tja;
  // 低速/拥堵跟车场景，计算的目标加速度，m/s^2
  Float32_t tar_a_tja;

  // 平滑前确定的规划最终类型
  Int8_t tar_type_before_final;
  // 平滑前确定的规划速度
  Float32_t tar_v_before_final;
  // 平滑前确定的规划加速度
  Float32_t tar_a_before_final;
  // pacc是否使能
  bool pacc_enable;
  // 平滑规划速度和加速度
  // 用于平滑速度结果数组是否初始化
  bool tar_v_his_init;
  // 速度平滑历史数据
  Float32_t tar_v_history[3];
  // 加速度平滑历史数据
  Float32_t tar_a_history[3];
  // 平滑后目标速度，m/s
  Float32_t tar_v_smoothed;
  // 平滑后的目标加速度，m/s^2
  Float32_t tar_a_smoothed;

  // 速度规划DTC 
  Int32_t error_code;
  // 速度规划最终类型
  Int8_t tar_type_final;
  // 速度规划最终速度
  Float32_t tar_v_final;
  // 速度规划最终加速度
  Float32_t tar_a_final;

  // PACC 相关数据
  PACC pacc;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    timestamp = 0;
    planning_pos.Clear();
    planning_path_point_num = 0;
    planning_path.Clear();

    adas_enable_pre = false;
    adas_enable = false;

    major_ref_line_point_num = 0;
    major_ref_line.Clear();
  
    user_setting_v = 0.0F;
    current_plan_v = 0.0F;
    upper_limit_v_road = 0.0F;
    upper_limit_v_tunnel = 0.0F;
    upper_limit_v_ramp = 0.0F;
    upper_limit_v_curvature = 0.0F;
    upper_limit_v_lane_change = 0.0F;
    upper_limit_v_final = 0.0F;

    tar_v_cruise = 0.0F;
    tar_a_cruise = 0.0F;

    target_path_max_curvature = 0.0F;
    tar_type_curvature = 0;
    tar_v_curvature = 0.0F;
    tar_a_curvature = 0.0F;

    changlane_path_point_num = 0;
    changlane_path.Clear();
    
    slope = 0.0F;

    ramp_velocity_limit = 0.0F;
    distance_to_ramp = 0.0F;
    tar_type_ramp = 0;
    tar_v_ramp = 0.0F;
    tar_a_ramp = 0.0F;

    tunnel_velocity_limit = 0.0F;
    distance_to_tunnel = 0.0F;
    tar_type_tunnel = 0;
    tar_v_tunnel = 0.0F;
    tar_a_tunnel = 0.0F;

    lane_velocity_limit = 0.0F;
    distance_to_lane_velocity_limit = 0.0F;
    tar_type_lane = 0;
    tar_v_lane = 0.0F;
    tar_a_lane = 0.0F;

    tar_type_virtual_obstacle = 0;
    tar_v_virtual_obstacle = 0.0;
    tar_a_virtual_obstacle = 0.0;

    fallback_type = 0;
    tar_type_fallback = 0;
    tar_v_fallback = 0.0F;
    tar_a_fallback = 0.0F;
    pacc_enable = false;
    collision_test_path.Clear();
    collision_test_samples.Clear();
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

    mio.Clear();
    tar_type_aeb = 0;
    tar_v_aeb = 0.0F;
    tar_a_aeb = 0.0F;

    collision_test_path_point_num_lca = 0;
    collision_test_path_lca.Clear();
    collision_test_samples_point_num_lca =0;
    collision_test_samples_lca.Clear();

    cipv_lca.Clear();
    tar_type_following_lca = 0;
    tar_v_following_lca = 0.0F;
    tar_a_following_lca = 0.0F;
    fsm_follow_state_lca = 0;

    avoidance_lca.Clear();
    tar_type_obstacle_lca = 0;
    tar_v_obstacle_lca = 0.0F;
    tar_a_obstacle_lca = 0.0F;
    fsm_obstacle_state_lca = 0;

    mio_lca.Clear();
    tar_type_aeb_lca = 0;
    tar_v_aeb_lca = 0.0F;
    tar_a_aeb_lca = 0.0F;

    tar_type_real_obstacle = 0;
    tar_v_real_obstacle = 0.0F;
    tar_a_real_obstacle = 0.0F;

    cipv_tja.Clear();
    tar_type_tja = 0;
    tar_v_tja = 0.0F;
    tar_a_tja = 0.0F;

    tar_type_before_final = 0.0F;
    tar_v_before_final = 0.0F;
    tar_a_before_final = 0.0F;

    tar_v_his_init = false;
    common::com_memset(tar_v_history, 0.0F, sizeof(tar_v_history));
    common::com_memset(tar_a_history, 0.0F, sizeof(tar_a_history));
    tar_v_smoothed = 0.0F;
    tar_a_smoothed = 0.0F;

    error_code = 0;
    tar_type_final = 0;
    tar_v_final = 0.0F;
    tar_a_final = 0.0F;

    pacc.Clear();
  }

  /**
   * @brief 构造函数
   */
  VelocityPlanningInternal() {
    Clear();
  }
};
/**
 * @struct VelocityPlanningResult
 * @brief 速度规划结果
 */
struct VelocityPlanningResult {
  /// 消息头
  ad_msg::MsgHead msg_head;

  /// 速度控制目标类型
  Int32_t tar_type;

  /// 目标车速
  plan_var_t tar_v;
  /// 目标加速度
  plan_var_t tar_a;

  /// 速度控制目标位置
  struct {
    /// 坐标x
    plan_var_t x;
    /// 坐标y
    plan_var_t y;
    /// 航向角, Range: (-pi, pi rad)
    plan_var_t heading;
    /// 相对于参考线的路径长
    plan_var_t s;
  } tar_pos;

  /// 释放油门
  bool release_throttle;
  /// 目标油门量
  plan_var_t tar_throttle;

  /// AEB预警标志
  bool aeb_warning;
  /// AEB响应标记
  bool aeb_action;

  /// 目标障碍物信息
  struct {
    /// 是否有效
    bool valid;
    /// 减速状态(减速/加速/等待)
    Int32_t obj_dec_status;
    /// 到目标距离
    plan_var_t dist_to_obj;
    /// 时距
    plan_var_t time_gap;
    /// 相对速度
    plan_var_t relative_v;
    /// 碰撞时间
    plan_var_t ttc;
    /// 障碍物速度
    plan_var_t obj_v;
    /// 距离等级
    Int32_t dist_gap_level;
    /// 速度差等级
    Int32_t rel_spd_level;
  } tar_obj;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    tar_v = 0.0F;
    tar_a = 0.0F;

    release_throttle = false;
    tar_throttle = 0.0F;
    tar_pos.x = 0.0F;
    tar_pos.y = 0.0F;
    tar_pos.heading  = 0.0F;
    tar_pos.s = 0.0F;

    tar_obj.valid = false;
    tar_obj.dist_to_obj = 0.0F;
    tar_obj.time_gap = 0.0F;
    tar_obj.relative_v = 0.0F;
    tar_obj.ttc = 100.0F;
    tar_obj.obj_v = 0.0F;
    tar_obj.dist_gap_level = -1;
    tar_obj.rel_spd_level = -1;
  }

  /**
   * @brief 构造函数
   */
  VelocityPlanningResult() {
    Clear();
  }
};

/**
 * @struct VelocityPlanningDebug
 * @brief 速度规划内部数据和输出数据
 */
struct VelocityPlanningDebug
{
  VelocityPlanningInternal velocity_plan_internal;
  VelocityPlanningResult velocity_plan_output;

  /**
   * @brief 清除内部数据
   */
  void Clear(){
    velocity_plan_internal.Clear();
    velocity_plan_output.Clear();
  }

  /**
   * @brief 构造函数
   */
  VelocityPlanningDebug(){
    Clear();
  }
};


/**
 * @struct ActionPlanningDataSource
 * @brief 行为规划输入数据
 */
struct ActionPlanningDataSource {
  /// 时间戳
  Int64_t timestamp;
  /// 驾驶地图信息
  const driv_map::DrivingMapWrapper* driving_map;
  /// 来自人机交互模块的设置信息
  const ad_msg::PlanningSettings* planning_settings;
  /// 车身信息
  const ad_msg::Chassis* chassis;
  /// 其它车身信息
  const ad_msg::SpecialChassisInfo* special_chassis_info;
  /// 交通标志信息
  const ad_msg::TrafficSignalList* traffic_signal_list;

  /// 上一次轨迹规划中，轨迹规划模块主动请求变道
  const ChangingLaneReq* changing_lane_req_from_trj_planning;
  /// 上一次轨迹规划中，轨迹规划模块对变道请求的反馈
  const ChangingLaneRsp* changing_lane_rsp;

  /// 上一次速度规划的结果
  const VelocityPlanningResult* result_of_velocity_planning;

  /**
   * @brief 构造函数
   */
  ActionPlanningDataSource() {
    timestamp = 0;
    driving_map = Nullptr_t;
    planning_settings = Nullptr_t;
    chassis = Nullptr_t;
    special_chassis_info = Nullptr_t;
    traffic_signal_list = Nullptr_t;

    changing_lane_req_from_trj_planning = Nullptr_t;
    changing_lane_rsp = Nullptr_t;

    result_of_velocity_planning = Nullptr_t;
  }
};

/**
 * @struct TrajectoryPlanningDataSource
 * @brief 轨迹规划输入数据
 */
struct TrajectoryPlanningDataSource {
  /// 时间戳
  Int64_t timestamp;
  /// 驾驶地图信息
  const driv_map::DrivingMapWrapper* driving_map;
  /// 行为规划的结果
  const ActionPlanningResult* result_of_action_planning;
  /// 车身信息
  const ad_msg::Chassis* chassis;

  /**
   * @brief 构造函数
   */
  TrajectoryPlanningDataSource() {
    timestamp = 0;
    driving_map = Nullptr_t;
    result_of_action_planning = Nullptr_t;
    chassis = Nullptr_t;
  }
};

/**
 * @struct VelocityPlanningDataSource
 * @brief 速度规划输入数据
 */
struct VelocityPlanningDataSource {
  /// 时间戳
  Int64_t timestamp;
  /// 驾驶地图信息
  const driv_map::DrivingMapWrapper* driving_map;
  /// 行为规划的结果
  const ActionPlanningResult* result_of_action_planning;
  /// 轨迹规划的结果
  const TrajectoryPlanningResult* result_of_trajectory_planning;
  /// 车身信息
  const ad_msg::Chassis* chassis;
  /// 特殊的底盘信息
  const ad_msg::SpecialChassisInfo* special_chassis;
  /// 特定型号车辆的信息
  const ad_msg::ChassisCtlCmd* chassis_ctl_cmd;
  /// 红绿灯信息
  const ad_msg::TrafficLightList* traffic_light_list;
  /// 交通标志信息
  const ad_msg::TrafficSignalList* traffic_signal_list;

#if (ENABLE_ROS_NODE)
  ::planning::plan_debug* debug_msg;
#endif

#ifdef ENABLE_PACC_ADASIS
  /// 从ADASIS v2 Horizon Provider获取的horizon数据
  const adasisv2::Horizon* adasisv2_horizon;
#endif

  /**
   * @brief 构造函数
   */
  VelocityPlanningDataSource() {
    timestamp = 0;
    driving_map = Nullptr_t;
    result_of_action_planning = Nullptr_t;
    result_of_trajectory_planning = Nullptr_t;
    chassis = Nullptr_t;
    special_chassis = Nullptr_t;
    chassis_ctl_cmd = Nullptr_t;
    traffic_light_list = Nullptr_t;
    traffic_signal_list = Nullptr_t;
  }
};

/**
 * @struct TrajectoryPlanningInfo
 * @brief 轨迹规划调试信息
 */
struct TrajectoryPlanningInfo {
  /// 消息头
  ad_msg::MsgHead msg_head;

  /// 路网构建类型
  Int32_t road_builed_type;
  /// 车道保持时的预瞄距离
  plan_var_t leading_length_for_lka;
  /// 路网分段长度
  common::StaticVector<plan_var_t, MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM>
      lon_graph_samples_step_len;
  /// 路网信息
  struct RoadGraph {
    /// link信息
    struct Link {
      /// link的采样点
      common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> sample_points;
      /// 是否与道路边界有干涉
      bool is_collision_with_road_boundary;

      /**
       * @brief 清除内部数据
       */
      void Clear() {
        sample_points.Clear();
        is_collision_with_road_boundary = false;
      }
    };

    /// 路网中的link列表
    common::StaticVector<Link, MAX_GRAPH_LINK_NUM> road_graph_links;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      for (Int32_t i = 0; i < road_graph_links.Size(); ++i) {
        road_graph_links[i].Clear();
      }
      road_graph_links.Clear();
    }
  } road_graph;

  /// 最优轨迹
  common::StaticVector<common::PathPoint,
  common::Path::kMaxPathPointNum> optimal_trajectory_sample_points;
  /// 之前的参考线
  common::StaticVector<common::PathPoint,
  common::Path::kMaxPathPointNum> prev_ref_line_sample_points;

  /// 横向偏差信息
  struct LatErr {
    /// 横向偏差
    plan_var_t lat_err;
    /// 平滑后的横向偏差
    plan_var_t lat_err_smooth;
    /// 横向偏差变化率
    plan_var_t lat_err_v;
    /// 平滑后的横向偏差变化率
    plan_var_t lat_err_v_smooth;
    plan_var_t lat_err_a;
    /// 角度偏差
    plan_var_t yaw_err;
    /// 平滑后的角度偏差
    plan_var_t yaw_err_smooth;
    /// 角度偏差变化率
    plan_var_t yaw_err_v;
    /// 平滑后的角度偏差变化率
    plan_var_t yaw_err_v_smooth;
  } lat_err_sample[2];

  struct {
    plan_var_t x;
    plan_var_t y;
    plan_var_t h;
  } lat_err_pred_point;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    road_builed_type = ROAD_BUILED_TYPE_INVALID;
    leading_length_for_lka = 0.0F;
    lon_graph_samples_step_len.Clear();

    road_graph.Clear();
    optimal_trajectory_sample_points.Clear();
    prev_ref_line_sample_points.Clear();

    common::com_memset(lat_err_sample, 0, sizeof(lat_err_sample));
    common::com_memset(&lat_err_pred_point, 0, sizeof(lat_err_pred_point));
  }

  /**
   * @brief 构造函数
   */
  TrajectoryPlanningInfo() {
    Clear();
  }
};

enum class PCC_VEL_SOURCE : uint8_t {
  DEFAULT = 0,
  USER_SETTING = 1,
  GLOBAL_VEL = 2,
  LANE_LIMIT = 3,
  ERROR
};
class PccModeChangeAccordingToVelSource
{
private:
  static const int MAX_RECORD_SIZE = 100; 
  static const int MAX_DEC_COUNT = 50;
  bool is_dec_;
  bool pre_v_is_user_setting_;
  int dec_count;
  PCC_VEL_SOURCE vel_source;
  common::StaticVector<plan_var_t, MAX_RECORD_SIZE> user_setting_v_list_;

public:
  PccModeChangeAccordingToVelSource()
    : is_dec_(false), pre_v_is_user_setting_(false), dec_count(0), vel_source(PCC_VEL_SOURCE::DEFAULT)
  {
    user_setting_v_list_.Clear();
  }

  const bool GetDecState() const { return is_dec_;}
  const bool SetDecState(const bool state) {
    is_dec_ = state;
    return is_dec_;
  }

  const bool GetPreVelState() const { return pre_v_is_user_setting_;}
  const bool SetPreVelState(const bool state) {
    pre_v_is_user_setting_ = state;
    return pre_v_is_user_setting_;
  }

  const int GetMaxDecCount() const { return MAX_DEC_COUNT; }
  const int GetDecCount() const { return dec_count;}
  const int StartDecCount() { return dec_count++;} 
  const int SetDecCount(const int value) { return dec_count;}


  const plan_var_t GetVelListBackElement() const { 
    if (user_setting_v_list_.Empty()) {
      return 0.0f;
    } else {
      return user_setting_v_list_.Back();
    }
  }
  const bool AddElementToVelList(const plan_var_t vel) {
    if (user_setting_v_list_.Full()) { // 用户设置速度的频率可能低于规控的周期
        for (int i = 0; i < (MAX_RECORD_SIZE - 1); i++)
        {
          user_setting_v_list_[i] = user_setting_v_list_[i+1];
        }
        user_setting_v_list_.PushBack(vel);
      return false;
    } else {
      user_setting_v_list_.PushBack(vel);
      return true;
    } 
  }
  void ClearVelList() { user_setting_v_list_.Clear();}

  const PCC_VEL_SOURCE GetVelSource() const { return vel_source;}
  const PCC_VEL_SOURCE SetVelSource(const PCC_VEL_SOURCE& sys_vel_source ) { 
    vel_source = sys_vel_source;
    return vel_source;
  }
};

} // namespace phoenix
} // namespace planning


#endif // PHOENIX_PLANNING_MOTION_PLANNING_H_

