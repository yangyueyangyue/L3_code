#  /// 障碍物ID
int32 id
#   /// 障碍物的类型
int8 type
#   /// 障碍物跟踪的状态
int32 track_status
#   /// 障碍物融合的类型
int32 merged_status
#   /// 障碍物是否是迎面驶来的
bool oncomming
#   /// 此障碍物是否是bridge
bool bridge
#   /// 障碍物距离传感器的距离（极坐标）(m)
float32 range
#   /// 障碍物与传感器中心纵轴之间的角度（极坐标）(rad)
float32 angle
#   /// 障碍物距离传感器的距离的变化率（极坐标）(m/sec)
float32 range_rate
#   /// 障碍物距离传感器的距离变化率的变化率（极坐标）(m/sec^2)
float32 range_acceleration
#   /// 障碍物的侧向速度（左正右负）(m/sec)
float32 lateral_rate
#   /// 障碍物长度 (m)
float32 length
#   /// 障碍宽度 (m)
float32 width
#   /// 障碍物位置x坐标 (m)
float32 x
#   /// 障碍物位置y坐标 (m)
float32 y
#   /// 障碍物的x向速度 (m/sec)
float32 v_x
#   /// 障碍物的x向速度 (m/sec)
float32 rel_v_x
#   /// 障碍物的y向速度 (m/sec)
float32 v_y
#   /// 障碍物的x向加速度 (m/sec^2)
float32 accel_x
#   /// 障碍物的y向加速度 (m/sec^2)
float32 accel_y
#   /// 障碍物的角速度 (rad/sec)
float32 yaw_rate
#   /// 障碍物横向（y）测距标准差(m)
float32 obj_std_dis_y
#   /// 障碍物纵向（x）测距标准差(m)
float32 obj_std_dis_x
#   /// 障碍物横向（y）速度标准差(m/s^2)
float32 obj_std_vel_y
#   /// 障碍物纵向（x）速度标准差(m/s^2)
float32 obj_std_vel_x

