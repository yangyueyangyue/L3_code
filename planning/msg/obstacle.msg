# 障碍物ID
int32 id

# 障碍物位置x坐标
float32 x

# 障碍物位置y坐标
float32 y

# 障碍物包围盒
obbox obb

# 障碍物高度
float32 height

# 离地面高度
float32 height_to_ground

# 障碍物类型
int8 type

# 是否是动态障碍物
int8 dynamic

# 障碍物存在的置信度
int8 confidence

# 感知类别
int8 perception_type

# 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
float32 v_x

# 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
float32 v_y

# 障碍物绝对速度，单位：米/秒
float32 v

# 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
float32 a_x

# 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
float32 a_y

# 障碍物绝对加速度，单位：米^2/秒
float32 a

# 障碍物在主参考线上的投影信息
proj_on_major_ref_line proj_on_major_ref_line

# 预测的轨迹的数量
int8 pred_path_num

# 预测的轨迹中点的数量
int8[] pred_path_point_num

# 对动态障碍物预测的轨迹
pred_path_1d[] pred_path_list

# 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
int8 tracked_path_point_num

# 障碍物跟踪轨迹轨迹
tracked_point[] tracked_path
