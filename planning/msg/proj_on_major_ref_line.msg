# 此信息是否有效 (0 ~ 无效, 1 ~ 有效)
int8 valid

# 障碍物在主参考线上的投影点的x坐标
float32 x

# 障碍物在主参考线上的投影点的y坐标
float32 y

# 障碍物在主参考线上的投影点的航向角（单位：弧度）
float32 heading

# 障碍物在主参考线上的投影点的曲率（单位：1/米）
float32 curvature

# 障碍物在主参考线上的投影点沿着路径的弧长（单位：米）
float32 s

# 障碍物与主参考线之间的侧向距离（单位：米），左正右负
float32 l
