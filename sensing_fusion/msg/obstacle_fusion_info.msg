# /// 障碍物ID
int32 id
# /// 障碍物位置x坐标
float32 x
# /// 障碍物位置y坐标
float32 y
# /// 障碍物包围盒
obbox_info obb
# /// 障碍物高度
float32 height
# /// 离地面高度
float32 height_to_ground
# /// 障碍物类型
int8 type
# /// 是否是动态障碍物
int8 dynamic
# /// 障碍物存在的置信度
int8 confidence
# /// 感知类别
int8 perception_type
# /// 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
float32 v_x
# /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
float32 rel_v_x
# /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
float32 v_y
# /// 障碍物绝对速度，单位：米/秒
float32 v
# /// 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
float32 a_x
# /// 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
float32 a_y
# /// 障碍物绝对加速度，单位：米^2/秒
float32 a

# // 生命值，若小于等于0，则将从列表删除
int32 life

# // 跟踪（匹配）上的次数
int32 matched

# // 最近一次跟踪（匹配）上的标记
# /*
# age_实际作用就是用来保存上一次匹配上时duration_的值。
# */
int32 age

# // 障碍在列表中存在的时间（次数）
int32 duration




