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
  
  # 障碍物绝对加速度，单位：米^2/秒
  float32 a
