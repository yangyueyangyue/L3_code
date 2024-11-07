  # 是否是风险障碍物
  bool is_risk

  # 对应的感知模块的障碍物信息
  obstacle perception_obstacle

  # 障碍物按照9宫格划分的位置
  int32 obj_position

  # 障碍物行驶方向(相对自车)
  int32 obj_direction

  # 是否满足cutin定义
  bool is_cutin

  # 等价于OBB碰撞检测的static_distance
  float32 obb_distance

  # 是否处理该障碍物
  bool need_process

  # 可跟随的障碍物，满足跟车要求
  bool can_following

  # 是否是CIPV，最终跟车目标
  bool is_cipv

  # 是否是需要减速避免碰撞的障碍物
  bool need_avoiding

  # 是否是最优先需要减速避障的障碍物
  bool is_mio

  # 是否是需要执行aeb warning的障碍物
  bool need_aeb_warning

  # 是否是被最终选择的执行aeb_warning的障碍物
  bool is_aeb_warning_target

  # 是否是需要执行aeb full brake的障碍物
  bool need_aeb_full_brake

  # 是否是被最终选择的执行aeb full brake的障碍物
  bool is_aeb_full_brake_target

  # 感知检测的障碍物距自车的纵向距离
  float32 dist_to_obj

  # 不同场景下需要与障碍物保持的安全距离
  float32 safe_dist

  # 规划算法的初始v
  float32 v_for_plan

  # 时距，s
  float32 time_gap

  # 相对时距 ttc，s
  float32 ttc

  # 规划算法的终状态v，m/s
  float32 planning_target_v

  # 规划算法的终状态s，m
  float32 planning_target_s
