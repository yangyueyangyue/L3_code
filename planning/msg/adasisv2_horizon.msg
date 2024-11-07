# @ref adasis_v2.h
adasisv2_position position
uint16 position_offset_from_start
uint16 position_offset_cyclic_num

# 当前坡度
adasisv2_profileshort profile_slope
uint16 profile_slope_offset_from_start
uint16 profile_slope_offset_cyclic_num

# 当前曲率
adasisv2_profileshort profile_curvatue
uint16 profile_curvatue_offset_from_start
uint16 profile_curvatue_offset_cyclic_num

bool is_ready

# 以自车为原点的坡度列表
int32 slope_profile_num
adasisv2_horizon_profile[] slope_list

# 以自车为原点的
int32 curvature_profile_num
adasisv2_horizon_profile[] curvature_list
