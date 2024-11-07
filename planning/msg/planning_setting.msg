# ADAS功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 start_adas
# LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_lka
# ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_acc
# AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_aeb

# 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_alc
# 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_isl

# Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
int8 enable_ngp

# 目标车速[m/s]
int8 target_velocity_valid
float32 target_velocity

# 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
int32 changing_lane_req
