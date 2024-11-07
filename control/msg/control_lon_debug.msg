# control : longitudinal
# 本车加速度计算值
float64 control_long_VecDa_aCalcd_mp 
# 旋转质量换算系数 kg*m^2
float64 control_long_VDC_facDrvTrmJ_mp
# unit : m/s^2 速度闭环积分反馈
float64 control_long_speed_integ_value              
# unit : m/s^2 速度闭环比例反馈
float64 control_long_speed_propo_value               
# unit : m/s^2 速度闭环补偿值
float64 control_long_speed_value_a_req             
# 0 : unfreezed, 1 : freezed 速度闭环积分冻结状态
uint8 control_long_speed_integ_freeze_status       
# 0 : uninitialized, 1 : initialized 速度闭环积分初始化状态
uint8 control_long_speed_inieg_init_status       
# unit : kph 车速偏差kph
float64 control_long_speed_error                   
# 0 ： 未激活纵向控制 | 1 ： 触发紧急制动 | 2 ： 常规驱动控制
# 3 ： 常规制动控制 | 4：起步 | 5：停车 | 6：临时驻车 
# 7：精准停车测试 | 8：油门透传模式 | 9：引擎外控测试(扭矩) | 10：制动系统外控测试
# 11：引擎外控测试（油门开度）
uint8 control_long_longitudinal_control_status
# unit : % 发动机请求扭矩百分比 为处理的 
float64 control_long_engine_torque_raw
# unit : % 发动机请求扭矩百分比
float64 control_long_engine_torque                
# unit : N*m 发动机请求扭矩
float64 control_long_engine_torque_nfm               
# unit : % 油门开度请求
float64 control_long_Eng_rAccrPedlReq  

# unit : % z真实的油门开度请求
float64 control_long_Eng_throttle_real 

# unit : mps2 制动减速度请求
float64 control_long_EBS_aReq                          
# unit : m/s^2 最大允许的加速度
float64 control_long_vehicle_max_acce               
# 加速换挡状态
uint8 control_long_speedup_shift_status          
# 变速箱起步限扭状态
uint8 control_long_tcu_torque_status             
# 负扭矩请求状态，0：驱动，1：制动
uint8 control_long_cochs_stnegtrqreq_mp         
# 滚动阻力
float64 control_long_fr_rolling_res_mp           
# 空气阻力
float64 control_longg_fr_air_drag_mp
# 坡度阻力                    
float64 control_long_fr_slp_res_mp               
# 加速阻力
float64 control_long_fr_acc_res_mp                 
# 补偿驱动力
float64 control_longg_fr_cmp_mp                   
# 发动机摩擦力
float64 control_long_fr_eng_fric_mp               

uint8 control_long_AD_Status
# 驻车起步状态
uint8 control_long_Cochs_stStarting               
# 速度闭环微分反馈
float64 control_long_Vmc_aAccnGvnrD_mp
# 名义加速度 m/s^2             
float64 control_long_Vdc_aReqNom
# 速度限制状态         
uint8 control_long_Tra_stShiftWhtSpdUp
# TCU限扭状态
uint8 control_long_Tra_stTrqLimWthTcu               
# 变速箱需求扭矩
float64 control_long_Tra_trqReq                  
# 加速PID
float64 control_long_acc_pid                           
