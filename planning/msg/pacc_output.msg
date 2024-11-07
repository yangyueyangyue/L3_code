# 目标车速 [m/s]
float32 tar_v

float32 target_unlimit_vel
# 目标加速度 [m/s^2]
float32 tar_a

# 目标档位 [-]
int8 tar_gear

# 目标发动机扭矩 [N/m]
float32 tar_engine_torque

# pcc目标驱动力 [N]
float32 tar_drive_force

# 当前阻力和 [N]
float32 cur_resistance

# 当前滚阻 [N]
float32 cur_Ff

# 当前风阻 [N]
float32 cur_Fw
# 当前坡阻 [N]
float32 cur_Fi

# 目标发动机转速 [rpm]
float32 tar_engine_ne

# 目标发动机油门开度[%]
float32 tar_throttle

# pcc 规划状态
int8 pcc_vel_plan_type

# 二分法求解状态 [-]
int8 dichotomy_solution_state

# Mpc求解状态 [-]
int8 mpc_solution_state

# PCC结果是否有效 [-]
int8 pcc_result_vaild
