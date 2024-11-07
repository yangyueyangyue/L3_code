int8 gnss_status

# 经度 [degree] [-180, 180]
float64 longitude

# 纬度 [degree] [-90, 90]
float64 latitude

# 高度 [m]
float64 altitude

# 俯仰角（-PI/2～PI/2弧度）
float64 pitch

# 偏航角（-PI～PI弧度）, 正东向为0度，逆时针为正方向(右手迪卡尔坐标系)
float64 yaw

# 横滚角（-PI～PI弧度）
float64 roll

# MPU 
int8 odom_status

# MPU x [m]
float64 x_odom

# MPU y [m]
float64 y_odom

# MPU z [m]
float64 z_odom

# MPU heading [rad]
float64 heading_odom

# PCC地图数据是否有效
int8 valid

# 数据来源，0：无效，1：TBOX；2：MPU
int8 pacc_points_source

# 采样步长
float32 pacc_point_s_step

# 采样点数
int16 pacc_point_num

# PCC地图数据
pcc_point[] pacc_points

# 当前周期对应的坡度，%
float32 cur_slope

# 当前周期预测坡度，%
float32 preview_slope

# 地图提供的预测距离，基于此距离做优化
float32 prediction_map_length

# 沿着地图路径累计的地图长度
float64 total_map_length
