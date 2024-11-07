# mpc求解的最优的协状态变量
float32 best_lambda

# 二分法迭代次数
int8  iteration_count

# 当前扭矩下的发动机最大转速  
float32 torque_max_base_cur_engine_speed

# 当前扭矩下的发动机最小转速
float32 torque_min_base_cur_engine_speed

# 最优预测
int16 pcc_best_point_num
mpc_best_iteration_point[] pcc_best_points

# 最终目标函数值
float32 Final_J

float32 Final_Q

# 中间预测点 Te, Ft, a, v, ne, throttle
float32 tar_sample_te
float32 tar_sample_ft
float32 tar_sample_a
float32 tar_sample_v
float32 tar_sample_ne
float32 tar_sample_throttle

# 根据扭矩+转速查表得到的油门开度
float32 tar_throttle_from_table
