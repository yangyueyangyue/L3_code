# 速度规划内部数据

# 速度规划计算时时间戳
int64 timestamp

# 速度规划计算时自车位置估算
trajectory_point planning_pos

# 路径规划路径点坐标变换？
int16 planning_path_point_num

path_point[] planning_path

bool adas_enable_pre
bool adas_enable

# 主参考线
int16 major_ref_line_point_num

path_point[] major_ref_line

# 巡航：用户设置目标车速 m/s
float32 user_setting_v

float32 current_plan_v

float32 upper_limit_v_road

float32 upper_limit_v_tunnel

float32 upper_limit_v_ramp

float32 upper_limit_v_curvature

float32 upper_limit_v_lane_change

float32 upper_limit_v_final

float32 tar_v_cruise

float32 tar_a_cruise

# 限速：弯道 : 相机/地图
float32 target_path_max_curvature

int16 tar_type_curvature

float32 tar_v_curvature

float32 tar_a_curvature

# 限速：变道
int16 changlane_path_point_num
path_point[] changlane_path

# TODO: 限速：坡道，上下坡速度/加速度约束：地图
float32 slope
float32 distance_to_slope

int8 tar_type_slope

float32 tar_v_slope

float32 tar_a_slope

# 交通规则：红绿灯
int8 traffic_light_type
float32 distance_to_traffic_light

int8 tar_type_traffic_light

float32 tar_v_traffic_light

float32 tar_a_traffic_light

# 交通规则：限速标识牌
int8 traffic_signal_type
float32 distance_to_traffic_signal

int8 tar_type_traffic_signal

float32 tar_v_traffic_signal

float32 tar_a_traffic_signal

# 交通规则：匝道
float32 ramp_velocity_limit
float32 distance_to_ramp

int8 tar_type_ramp

float32 tar_v_ramp

float32 tar_a_ramp

# 交通规则：隧道
float32 tunnel_velocity_limit
float32 distance_to_tunnel

int8 tar_type_tunnel

float32 tar_v_tunnel

float32 tar_a_tunnel

# 交通规则：道路限速
float32 lane_velocity_limit

float32 distance_to_lane_velocity_limit

int8 tar_type_lane

float32 tar_v_lane

float32 tar_a_lane

# 交通规则：final，将之当做虚拟障碍物

int8 tar_type_virtual_obstacle

float32 tar_v_virtual_obstacle

float32 tar_a_virtual_obstacle

# 限速：ODD Fallback
int8 fallback_type
int8 tar_type_fallback

float32 tar_v_fallback

float32 tar_a_fallback

# 速度规划：障碍物
int16 collision_test_path_point_num
path_point[] collision_test_path

int16 collision_test_samples_point_num
path_point[] collision_test_samples
# CollisionTestOnPathResult collision_test_result

## 速度规划：跟车 ACC
obstacle_decision cipv # CIPV

int8 tar_type_following

float32 tar_v_following

float32 tar_a_following

# 跟车场景五次多项式补丁状态
int8 fsm_follow_state

## 速度规划：避障 obstacle avoidance
obstacle_decision avoidance # 目标障碍物

int8 tar_type_obstacle

float32 tar_v_obstacle

float32 tar_a_obstacle

int8 fsm_obstacle_state

## 速度规划：紧急制动 AEB
obstacle_decision mio # 最可能碰撞的障碍物

int8 tar_type_aeb

float32 tar_v_aeb

float32 tar_a_aeb

# 变道-速度规划：障碍物
int16 collision_test_path_point_num_lca
path_point[] collision_test_path_lca

int16 collision_test_samples_point_num_lca
path_point[] collision_test_samples_lca
# CollisionTestOnPathResult collision_test_result

## 变道-速度规划：跟车 ACC
obstacle_decision cipv_lca # CIPV

int8 tar_type_following_lca

float32 tar_v_following_lca

float32 tar_a_following_lca

# 变道-跟车场景五次多项式补丁状态
int8 fsm_follow_state_lca

## 变道-速度规划：避障 obstacle avoidance
obstacle_decision avoidance_lca # 目标障碍物

int8 tar_type_obstacle_lca

float32 tar_v_obstacle_lca

float32 tar_a_obstacle_lca

int8 fsm_obstacle_state_lca

## 变道-速度规划：紧急制动 AEB
obstacle_decision mio_lca # 最可能碰撞的障碍物

int8 tar_type_aeb_lca

float32 tar_v_aeb_lca

float32 tar_a_aeb_lca

# 障碍物处理：final
int8 tar_type_real_obstacle

float32 tar_v_real_obstacle

float32 tar_a_real_obstacle

# 平滑前确定的规划最终类型
int8 tar_type_before_final
# 平滑前确定的规划速度
float32 tar_v_before_final
# 平滑前确定的规划加速度
float32 tar_a_before_final
# pcc_使能
bool is_pcc_from_road
# 平滑规划速度和加速度
bool tar_v_his_init

# 速度平滑历史数据
float32[3] tar_v_history
# 加速度平滑历史数据
float32[3] tar_a_history
# 平滑后目标速度，m/s
float32 tar_v_smoothed
# 平滑后的目标加速度，m/s^2
float32 tar_a_smoothed

# 速度规划DTC
int32 error_code
# 速度规划最终类型
int8 tar_type_final
# 速度规划最终速度
float32 tar_v_final
# 速度规划最终加速度
float32 tar_a_final

# PACC
pacc pacc
