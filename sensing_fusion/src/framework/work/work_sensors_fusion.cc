#include "work_sensors_fusion.h"

#include "communication/shared_data.h"
#include "perception/sensors_fusion/multi_constructure_classes_repo.h"
#include "sensors_fusion/goyu/sensors_fusion_mainstream_impl.h"



namespace phoenix {
namespace framework {


    

WorkSensorsFusion::WorkSensorsFusion()
{
    driving_map_.SetDrivingMapImpl(driv_map::GetDrivingMapImpl(0));
    objs_sensors_fusion_ptr_ = Nullptr_t;
    driv_map::DrivingMapConfig conf;
    driving_map_.Configurate(conf);

    Initialize();
    // around_radar_handler_ = Nullptr_t;
    around_radar_handler_ = new AroundRadarHandler(); 

    visual_control_handler_ = new VisualControlHandler();

    main_camera_handler_  = new MainCameraHandler();

    main_radar_handler_  = new MainRadarHandler();

    

#if DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM 
    test_radar_count = -1; 
    test_flag = 0;
#endif

}

WorkSensorsFusion::~WorkSensorsFusion()
{
    delete around_radar_handler_;

    delete visual_control_handler_;

    delete main_camera_handler_;
}

void WorkSensorsFusion::Initialize()
{
    objs_sensors_fusion_ptr_.reset(new perception::sensorsfusion::goyu::SensorsFusionMainstreamImpl());
}

Int32_t WorkSensorsFusion::DoWork()
{

    SharedData* shared_data = SharedData::instance();
    bool ret = false;
    Int32_t status = INTERNAL_MODULE_STATUS_OK;
    
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);
    perception_module_status_.Clear();
    shared_data->SetPerceptionModuleStatus(perception_module_status_);

    // 获取GNSS数据
    shared_data->GetGnssData(&gnss_info_);
    // 获取IMU数据
    shared_data->GetImuData(&imu_info_);
    // 获取车身数据
    shared_data->GetChassis(&chassis_);
    // 获取相机识别的车道线数据
    shared_data->GetLaneMarkCameraList(&lane_mark_main_forward_camera_list_);
    // 获取相机识别的车道线数据
    shared_data->GetLaneCurbCameraList(&lane_curb_main_forward_camera_list_);
    // 获取相机识别的障碍物信息
    shared_data->GetObstacleMaxieyeCameraList(&obstacle_main_forward_camera_list_);
    // 获取430识别的障碍物信息
    shared_data->GetObstacleEsrFrontList(&obstacle_main_forward_radar_list_);

    // 获取Lidar识别的障碍物信息
    shared_data->GetObstacleLidarFrontList(&obstacle_main_forward_lidar_list_);

    // 获取地理围栏信息
    shared_data->GetMapGeofenceInfo(&geofence_info_);

    //获取左前侧雷达障碍物信息
    shared_data->GetObstacleRadarLeftFwFilterList(&obstacle_left_forward_radar_list_);
    //获取右前侧雷达障碍物信息
    shared_data->GetObstacleRadarRightFwFilterList(&obstacle_right_forward_radar_list_);
    //获取左后侧雷达障碍物信息
    shared_data->GetObstacleRadarLeftBwFilterList(&obstacle_left_backward_radar_list_);
    //获取右后侧雷达障碍物信息
    shared_data->GetObstacleRadarRightBwFilterList(&obstacle_right_backward_radar_list_);
  
#if(ENABLE_VISUAL_CONTROL_INPUT_FUSION)
    //获取环视处理器障碍物信息
    //获取前向视觉障碍物信息
    shared_data->GetObstacleVisualControlFrontList(&obstacle_center_forward_camera_list_);
    //获取左前视觉障碍物信息
    shared_data->GetObstacleVisualControlFrontLeftList(&obstacle_left_side_camera_list_);
    //获取右前视觉障碍物信息
    shared_data->GetObstacleVisualControlFrontRightList(&obstacle_right_side_camera_list_);
    //获取左后视觉障碍物信息
    shared_data->GetObstacleVisualControlRearLeftList(&obstacle_left_backward_camera_list_);
    //获取右后视觉障碍物信息
    shared_data->GetObstacleVisualControlRearRightList(&obstacle_right_backward_camera_list_);
#endif

#if (ENABLE_VISUAL_CONTROL_PRE_PROCESS)

    //前向视觉处理器进行数据预处理
    visual_control_handler_->UpdateCenterFwCameraList(obstacle_center_forward_camera_list_);

    //左前向视觉处理器进行数据预处理
    visual_control_handler_->UpdateLeftSideCameraList(obstacle_left_side_camera_list_);

    //右前向视觉处理器进行数据预处理
    visual_control_handler_->UpdateRightSideCameraList(obstacle_right_side_camera_list_);

    //左后向视觉处理器进行数据预处理
    visual_control_handler_->UpdateLeftBwCameraList(obstacle_left_backward_camera_list_);

    //右后向视觉处理器进行数据预处理
    visual_control_handler_->UpdateRightBwCameraList(obstacle_right_backward_camera_list_);

    shared_data->SetObstacleJMFrontList(obstacle_center_forward_camera_list_);

    shared_data->SetObstacleJMFrontLeftList(obstacle_left_side_camera_list_);

    shared_data->SetObstacleJMFrontRightList(obstacle_right_side_camera_list_);

    shared_data->SetObstacleJMRearLeftList(obstacle_left_backward_camera_list_);

    shared_data->SetObstacleJMRearRightList(obstacle_right_backward_camera_list_);

#endif 


#if ENABLE_CARRYING_STANDARD_TRAILER
/*
*济南测试场所用的挂宽度是2.55m 
*其挂车尺寸信息 长X宽X高 = 13.00m X 2.55m X 4.00m
*/
//使用标准挂
    chassis_.trailer_status = 1;
    chassis_.trailer_w = 2.5;
    chassis_.trailer_l = 13;
    chassis_.trailer_h = 3.5;
#endif

    // 获取交通标志及交通信号数据
    shared_data->GetTrafficSignalList(&traffic_signal_list_);
    shared_data->GetTrafficLightList(&traffic_light_list_);


    // LOG_INFO(5) << "[time] chassis:"<< chassis_.msg_head.timestamp << " now:" << common::GetClockNowMs() << " left fw radar:" << obstacle_left_forward_radar_list_.msg_head.timestamp;


    driv_map::DrivingMapDataSource map_data_source;
    map_data_source.chassis =  &chassis_;
    map_data_source.gnss =  &gnss_info_;
    map_data_source.imu =  &imu_info_;
    


    /// 对当前车辆位置进行平滑滤波
    pos_filter::PosFilterDataSource pos_filter_data_src;
    pos_filter_data_src.timestamp = common::GetClockNowMs();
    pos_filter_data_src.gnss = &gnss_info_;
    pos_filter_data_src.imu = &imu_info_;
    pos_filter_data_src.chassis = &chassis_;
    pos_filter_data_src.lane_mark_camera_list = &lane_mark_main_forward_camera_list_;
    ret = pos_filter_.Update(pos_filter_data_src);
    pos_filter_.GetRelativePosList(&relative_pos_list_);

    // 获取感知模块状态值
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);
    perception_module_status_.timestamp = pos_filter_data_src.timestamp;
    shared_data->SetPerceptionModuleStatus(perception_module_status_);

    const ad_msg::LaneInfoCameraList& lane_info_list = pos_filter_.GetLaneInfoCameraList();
    map_data_source.camera_lane_list = &lane_info_list;
    map_data_source.rel_pos_list = &relative_pos_list_;
    map_data_source.timestamp = pos_filter_data_src.timestamp;

    if (false == ret) {
        LOG_ERR << "Failed to filter position.";
        status = PERCEPTION_MODULE_STATUS_ERR_POS_FILTER_FAULT;
        return (status);    
    }

    ret = driving_map_.Update(map_data_source);

    if (false == ret) {
        LOG_ERR << "Failed to Update Driving Map.";
        // status = PERCEPTION_MODULE_STATUS_ERR_DRIVING_MAP_FAULT;
        // return (status);
    }


/**
 * @brief 以下是前视一体机的数据预处理
 * 
 */
#if (ENABLE_MAIN_CAMERA_PRE_PROCESS)
    // 定义拆解参数
    vector<Int32_t> dis_param_vector{80, 200};
    // 存储拆解前视一体机数据
    vector<ad_msg::ObstacleCameraList> dis_camera_vector;

    main_camera_handler_->SetRelativePosList(&relative_pos_list_);
    main_camera_handler_->SetMapGeofenceInfo(&geofence_info_);
    // main_camera_handler_->UpdateMainCameraList(obstacle_main_forward_camera_list_, lane_curb_main_forward_camera_list_);
    main_camera_handler_->CheckCameraDataStatus(obstacle_main_forward_camera_list_);
    main_camera_handler_->UpdateMainCameraObjectSpeed(obstacle_main_forward_camera_list_, chassis_);
    Uint8_t ret_1 = main_camera_handler_->DisMainCameraObject(obstacle_main_forward_camera_list_, dis_param_vector, dis_camera_vector);
    if (ret_1 == 0) {
        printf("No Dis Camera Param!\n");
    } else if (ret_1 == 1) {
        printf("Dis Camera Success!\n");
    } else if (ret_1 == 2) {
        printf("No Camera List!\n");
    }
    
    shared_data->SetObstacleMaxieyeCameraListAfterPreprocess(obstacle_main_forward_camera_list_);

#endif
/**
 * @brief 下面的代码进行侧雷达的预处理，需要用到relative_pos_list_，所以将相应的代码
 * 放到这个位置。
 */
#if (ENABLE_AROUND_RADAR_PRE_PROCESS)

    Int64_t t0 = common::GetClockNowUs();

    around_radar_handler_->SetRelativePosList(&relative_pos_list_);

    //对其左前侧雷达进行数据预处理
    around_radar_handler_->UpdateLeftFwRadarList(obstacle_left_forward_radar_list_, chassis_);
    //对其右前侧雷达进行数据预处理
    around_radar_handler_->UpdateRightFwRadarList(obstacle_right_forward_radar_list_, chassis_);
    //对其左后侧雷达进行数据预处理
    around_radar_handler_->UpdateLeftBwRadarList(obstacle_left_backward_radar_list_, chassis_);
    //对其右后侧雷达进行数据预处理
    around_radar_handler_->UpdateRightBwRadarList(obstacle_right_backward_radar_list_, chassis_);

    shared_data->SetObstacleAnngicRadarFrontLeftList(obstacle_left_forward_radar_list_);
    shared_data->SetObstacleAnngicRadarFrontRightList(obstacle_right_forward_radar_list_);
    shared_data->SetObstacleAnngicRadarRearLeftList(obstacle_left_backward_radar_list_);
    shared_data->SetObstacleAnngicRadarRearRightList(obstacle_right_backward_radar_list_);

    Int64_t t1 = common::GetClockNowUs();

#if ENABLE_SHOW_RADAR_PRE_COST_TIME
    
    LOG_INFO(5) << "<<<<<<<cost time (ms)>>>>>>----> " << (double)((t1 - t0)/1000.0); 

#endif  //ENABLE_SHOW_RADAR_PRE_COST_TIME


#endif //(ENABLE_AROUND_RADAR_PRE_PROCESS)


#if  (ENABLE_MAIN_RADAR_PRE_PROCESS)
    main_radar_handler_-> UpdateMainRadarList(pos_filter_data_src.timestamp, &driving_map_ , obstacle_main_forward_radar_list_);
#endif

    perception::sensorsfusion::ObjsFilterDataSource source_data;
    ad_msg::ObstacleList objs_sensors_fusion_res;
    source_data.curb_list_cam_ = &lane_curb_main_forward_camera_list_;
    source_data.timestamp_ = pos_filter_data_src.timestamp;
    source_data.rel_pos_list_ = &relative_pos_list_;
#if (ENABLE_MAIN_CAMERA_PRE_PROCESS)
    if (!dis_camera_vector.size()) {
        source_data.objs_list_main_forward_cam_80_ = Nullptr_t;
        source_data.objs_list_main_forward_cam_150_ = Nullptr_t;
    } else {
        source_data.objs_list_main_forward_cam_80_ = &dis_camera_vector[0];
        source_data.objs_list_main_forward_cam_150_ = &dis_camera_vector[1];
    }
#else
    source_data.objs_list_main_forward_cam_ = &obstacle_main_forward_camera_list_;
#endif
    

#if DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM
    static int32_t start = 60;
    static int32_t test_count = 0;
    static int32_t duration = 1;
    test_radar_count++;
    if (test_radar_count >= start && test_radar_count < start + duration && test_count < 15){//消失4个周期
        test_flag = 20;
        start += 2;
        if (test_count >= 15)
        {
            duration = 1;
        }
        test_count ++;
    }
    else{
        test_flag = 0;
        source_data.objs_list_main_forward_radar_ = &obstacle_main_forward_radar_list_;
    }
#else
    source_data.objs_list_main_forward_radar_ = &obstacle_main_forward_radar_list_;
#endif 


    source_data.objs_list_main_forward_lidar_ = &obstacle_main_forward_lidar_list_;
    source_data.driving_map_ = &driving_map_;

#if (ENABLE_AROUND_RADAR_INPUT_FUSION && !ENABLE_AROUND_RADAR_SKIP_FUSION)
    source_data.objs_list_left_side_radar_ = &obstacle_left_forward_radar_list_;
    source_data.objs_list_right_side_radar_ = &obstacle_right_forward_radar_list_;
    source_data.objs_list_left_backward_radar_ = &obstacle_left_backward_radar_list_;
    source_data.objs_list_right_backward_radar_ = &obstacle_right_backward_radar_list_;
#endif
   
#if ENABLE_VISUAL_CONTROL_INPUT_FUSION
    
    #if ENABLE_CENTER_FORWARD_VISUAL_CONTROL_INPUT_FUSION
    source_data.objs_list_center_forward_cam_ = &obstacle_center_forward_camera_list_;
    #endif

    source_data.objs_list_left_side_cam_ = &obstacle_left_side_camera_list_;
    source_data.objs_list_right_side_cam_ = &obstacle_right_side_camera_list_;
    source_data.objs_list_left_backward_cam_ = &obstacle_left_backward_camera_list_;
    source_data.objs_list_right_backward_cam_ = &obstacle_right_backward_camera_list_;
#endif

    status = objs_sensors_fusion_ptr_->ProcessSensorsFusionPipeline(source_data);

    objs_sensors_fusion_ptr_->GetObstaclesLists(&objs_sensors_fusion_res);


#if ENABLE_AROUND_RADAR_SKIP_FUSION
    around_radar_handler_->CombineAroundRadarToFusedListDirectly(obstacle_left_forward_radar_list_,
                                                                obstacle_right_forward_radar_list_,
                                                                obstacle_left_backward_radar_list_,
                                                                obstacle_right_backward_radar_list_,
                                                                objs_sensors_fusion_res);
#endif


    

    if(objs_sensors_fusion_res.obstacle_num >= 0) {
       LOG_INFO(3) << "fusion_object_num: " << objs_sensors_fusion_res.obstacle_num;
       shared_data->SetObstaclesFusionList(objs_sensors_fusion_res);
       //printf("half_length:%f, half_width:%f, x:%f, y:%f\n", objs_sensors_fusion_res.obstacles[0].obb.half_length, objs_sensors_fusion_res.obstacles[0].obb.half_width, objs_sensors_fusion_res.obstacles[0].obb.x, objs_sensors_fusion_res.obstacles[0].obb.y);   
    }

#if ENABLE_AROUND_RADAR_PRE_PROCESS
    // 执行雷达预处理模块的周期任务
    around_radar_handler_->Clock(source_data.timestamp_);
#endif

#if ENABLE_PUBLISH_PERCEPTION_DEBUG
    ConstitutePerceptionDebug(objs_sensors_fusion_res);
#endif


    if (status != PERCEPTION_MODULE_STATUS_OK){
        return status;
    }
    
    //检查输入输出是否相符
    status = objs_sensors_fusion_ptr_->CheckSourceDataAndTrackedInfoList(source_data, objs_sensors_fusion_res);

    return (status);
}

const ad_msg::ObstacleList &WorkSensorsFusion::GetSensorsFusionObjsResult()
{
    return obstacle_list_;
}

void WorkSensorsFusion::ConstitutePerceptionDebug(ad_msg::ObstacleList &obstacle_list)
{
    /**
     * @brief （target_id，target_camera_id） (33,2) 为bug1266_47_1007 对应的误检测障碍物。
     * (80,2) 为“2023-01-08-09-33-43-60跟车-前车缓停-感知有问题-没刹住.bag” 对应目标车的id组合。
     * (1,96) 为“2023-01-07-16-41-21-40跟停-前方挂车-失败”  对应目标车的id组合。
     * 
     * 
     * 770bug:
     * <4> (99,1)
     * <5> (96,1)
     * <6> (68,1)
     * <8> (29,1)
     * <9> (4,1)
     * <10> (2,1)
     * <12> (7,1)
     * <13> (29,1)
     * 
     * 
     * 
     * 
     */

    int target_id = 21;//这里的target_id针对rosbag包回放的实际值进行设定,这里是前向毫米波雷达的ID
    int target_radar_id = 21;
    int target_camera_id = 0;//这里的target_camera_id针对rosbag包回放的实际值进行设定
    
    int target_id_1 = 21;//这里的target_id针对rosbag包回放的实际值进行设定,这里是前向毫米波雷达的ID
    int target_radar_id_1 = 21;
    int target_camera_id_1 = 0;//这里的target_camera_id针对rosbag包回放的实际值进行设定

/*
侧雷达预处理前后对比，以下是指定的ID
*/
    int target_radar_left_fw_before_id = 113;
    int target_radar_left_fw_after_id = 113;

    int target_radar_right_fw_before_id = 0;
    int target_radar_right_fw_after_id = 0;

    int target_radar_left_bw_before_id = 197;
    int target_radar_left_bw_after_id = 197;

    int target_radar_right_bw_before_id = 0;
    int target_radar_right_bw_after_id = 0;

    memset(&perception_debug_.fusion_list, 0, sizeof(perception_debug_.fusion_list)); 
    // perception_debug_.fusion_list.target_obstacle_1
    // lane Ros debug message
    perception_debug_.lane_input.lane_mark_num = lane_mark_main_forward_camera_list_.lane_mark_num;
    perception_debug_.lane_input.lane_camera.clear();
    perception_debug_.lane_input.lane_camera.reserve(lane_mark_main_forward_camera_list_.lane_mark_num);
    ::sensing_fusion::laneline_info lanelineInfo;
    for (Int32_t i = 0; i < lane_mark_main_forward_camera_list_.lane_mark_num; ++i) {
        lanelineInfo.lane_id = lane_mark_main_forward_camera_list_.lane_marks[i].id;
        lanelineInfo.c0 = lane_mark_main_forward_camera_list_.lane_marks[i].c0;
        lanelineInfo.c1 = lane_mark_main_forward_camera_list_.lane_marks[i].c1;
        lanelineInfo.c2 = lane_mark_main_forward_camera_list_.lane_marks[i].c2;
        lanelineInfo.c3 = lane_mark_main_forward_camera_list_.lane_marks[i].c3;
        lanelineInfo.lane_mark_type = lane_mark_main_forward_camera_list_.lane_marks[i].lane_mark_type;
        lanelineInfo.quality = lane_mark_main_forward_camera_list_.lane_marks[i].quality;
        perception_debug_.lane_input.lane_camera.push_back(lanelineInfo);
    }

    // Obs ROS debug message
    
    perception_debug_.fusion_list.timestamp = obstacle_list.msg_head.timestamp;
    perception_debug_.fusion_list.sequence = obstacle_list.msg_head.sequence;
    perception_debug_.fusion_list.obstacle_num = obstacle_list.obstacle_num;
    perception_debug_.fusion_list.obstacles.clear();
    perception_debug_.fusion_list.obstacles.reserve(obstacle_list.obstacle_num);
    
   

    ::sensing_fusion::obstacle_fusion_info obstacleInfo;
    for (Int32_t i = 0; i < obstacle_list.obstacle_num; ++i) {
    // 障碍物ID
        obstacleInfo.id = obstacle_list.obstacles[i].id;
        obstacleInfo.x = obstacle_list.obstacles[i].x;
        obstacleInfo.y = obstacle_list.obstacles[i].y;
        obstacleInfo.height = obstacle_list.obstacles[i].height;
        obstacleInfo.height_to_ground = obstacle_list.obstacles[i].height_to_ground;
        obstacleInfo.type = obstacle_list.obstacles[i].type;
        obstacleInfo.dynamic = obstacle_list.obstacles[i].dynamic;
        obstacleInfo.confidence = obstacle_list.obstacles[i].confidence;
        obstacleInfo.perception_type = obstacle_list.obstacles[i].perception_type;
        obstacleInfo.v_x = obstacle_list.obstacles[i].v_x;
        obstacleInfo.v_y = obstacle_list.obstacles[i].v_y;
        obstacleInfo.v = obstacle_list.obstacles[i].v;
        obstacleInfo.a_x = obstacle_list.obstacles[i].a_x;
        obstacleInfo.a_y = obstacle_list.obstacles[i].a_y;
        obstacleInfo.a = obstacle_list.obstacles[i].a;

#if (DEBUG_FUSION_OBJ_CONFIDENCE)

        obstacleInfo.life = obstacle_list.obstacles[i].life;
        obstacleInfo.matched = obstacle_list.obstacles[i].matched;
        obstacleInfo.age = obstacle_list.obstacles[i].age;
        obstacleInfo.duration = obstacle_list.obstacles[i].duration;
    
#endif

        obstacleInfo.obb.x = obstacle_list.obstacles[i].obb.x;
        obstacleInfo.obb.y = obstacle_list.obstacles[i].obb.y;
        obstacleInfo.obb.heading = obstacle_list.obstacles[i].obb.heading;
        obstacleInfo.obb.half_width = obstacle_list.obstacles[i].obb.half_width;
        obstacleInfo.obb.half_length = obstacle_list.obstacles[i].obb.half_length;

        perception_debug_.fusion_list.obstacles.push_back(obstacleInfo);

        if(target_id == obstacle_list.obstacles[i].id)
        {
            perception_debug_.fusion_list.target_obstacle.id = obstacle_list.obstacles[i].id;
            perception_debug_.fusion_list.target_obstacle.x = obstacle_list.obstacles[i].x;
            perception_debug_.fusion_list.target_obstacle.y = obstacle_list.obstacles[i].y;
            perception_debug_.fusion_list.target_obstacle.height = obstacle_list.obstacles[i].height;
            perception_debug_.fusion_list.target_obstacle.height_to_ground = obstacle_list.obstacles[i].height_to_ground;
            perception_debug_.fusion_list.target_obstacle.type = obstacle_list.obstacles[i].type;
            perception_debug_.fusion_list.target_obstacle.dynamic = obstacle_list.obstacles[i].dynamic;
            perception_debug_.fusion_list.target_obstacle.confidence = obstacle_list.obstacles[i].confidence;
            perception_debug_.fusion_list.target_obstacle.perception_type = obstacle_list.obstacles[i].perception_type;
            perception_debug_.fusion_list.target_obstacle.v_x = obstacle_list.obstacles[i].v_x;
            perception_debug_.fusion_list.target_obstacle.v_y = obstacle_list.obstacles[i].v_y;
            perception_debug_.fusion_list.target_obstacle.v = obstacle_list.obstacles[i].v;
            perception_debug_.fusion_list.target_obstacle.a_x = obstacle_list.obstacles[i].a_x;
            perception_debug_.fusion_list.target_obstacle.a_y = obstacle_list.obstacles[i].a_y;
            perception_debug_.fusion_list.target_obstacle.a = obstacle_list.obstacles[i].a;

            perception_debug_.fusion_list.target_obstacle.obb.x = obstacle_list.obstacles[i].obb.x;
            perception_debug_.fusion_list.target_obstacle.obb.y = obstacle_list.obstacles[i].obb.y;
            perception_debug_.fusion_list.target_obstacle.obb.heading = obstacle_list.obstacles[i].obb.heading;
            perception_debug_.fusion_list.target_obstacle.obb.half_width = obstacle_list.obstacles[i].obb.half_width;
            perception_debug_.fusion_list.target_obstacle.obb.half_length = obstacle_list.obstacles[i].obb.half_length;

#if (DEBUG_FUSION_OBJ_CONFIDENCE)

            perception_debug_.fusion_list.target_obstacle.life = obstacle_list.obstacles[i].life;
            perception_debug_.fusion_list.target_obstacle.matched = obstacle_list.obstacles[i].matched;
            perception_debug_.fusion_list.target_obstacle.age = obstacle_list.obstacles[i].age;
            perception_debug_.fusion_list.target_obstacle.duration = obstacle_list.obstacles[i].duration;
    
#endif


        }

        if(target_id_1 == obstacle_list.obstacles[i].id)
        {
            perception_debug_.fusion_list.target_obstacle_1.id = obstacle_list.obstacles[i].id;
            perception_debug_.fusion_list.target_obstacle_1.x = obstacle_list.obstacles[i].x;
            perception_debug_.fusion_list.target_obstacle_1.y = obstacle_list.obstacles[i].y;
            perception_debug_.fusion_list.target_obstacle_1.height = obstacle_list.obstacles[i].height;
            perception_debug_.fusion_list.target_obstacle_1.height_to_ground = obstacle_list.obstacles[i].height_to_ground;
            perception_debug_.fusion_list.target_obstacle_1.type = obstacle_list.obstacles[i].type;
            perception_debug_.fusion_list.target_obstacle_1.dynamic = obstacle_list.obstacles[i].dynamic;
            perception_debug_.fusion_list.target_obstacle_1.confidence = obstacle_list.obstacles[i].confidence;
            perception_debug_.fusion_list.target_obstacle_1.perception_type = obstacle_list.obstacles[i].perception_type;
            perception_debug_.fusion_list.target_obstacle_1.v_x = obstacle_list.obstacles[i].v_x;
            perception_debug_.fusion_list.target_obstacle_1.v_y = obstacle_list.obstacles[i].v_y;
            perception_debug_.fusion_list.target_obstacle_1.v = obstacle_list.obstacles[i].v;
            perception_debug_.fusion_list.target_obstacle_1.a_x = obstacle_list.obstacles[i].a_x;
            perception_debug_.fusion_list.target_obstacle_1.a_y = obstacle_list.obstacles[i].a_y;
            perception_debug_.fusion_list.target_obstacle_1.a = obstacle_list.obstacles[i].a;

            perception_debug_.fusion_list.target_obstacle_1.obb.x = obstacle_list.obstacles[i].obb.x;
            perception_debug_.fusion_list.target_obstacle_1.obb.y = obstacle_list.obstacles[i].obb.y;
            perception_debug_.fusion_list.target_obstacle_1.obb.heading = obstacle_list.obstacles[i].obb.heading;
            perception_debug_.fusion_list.target_obstacle_1.obb.half_width = obstacle_list.obstacles[i].obb.half_width;
            perception_debug_.fusion_list.target_obstacle_1.obb.half_length = obstacle_list.obstacles[i].obb.half_length;


#if (DEBUG_FUSION_OBJ_CONFIDENCE)

            perception_debug_.fusion_list.target_obstacle_1.life = obstacle_list.obstacles[i].life;
            perception_debug_.fusion_list.target_obstacle_1.matched = obstacle_list.obstacles[i].matched;
            perception_debug_.fusion_list.target_obstacle_1.age = obstacle_list.obstacles[i].age;
            perception_debug_.fusion_list.target_obstacle_1.duration = obstacle_list.obstacles[i].duration;
    
#endif

        }

    }

    //前向毫米波雷达
    perception_debug_.front_radar_list.timestamp = obstacle_main_forward_radar_list_.msg_head.timestamp;
    perception_debug_.front_radar_list.sequence = obstacle_main_forward_radar_list_.msg_head.sequence;
    perception_debug_.front_radar_list.obstacle_num = obstacle_main_forward_radar_list_.obstacle_num;
    perception_debug_.front_radar_list.obstacles.clear();
    perception_debug_.front_radar_list.obstacles.reserve(obstacle_main_forward_radar_list_.obstacle_num);
    ::sensing_fusion::obstacle_radar obstacle_radar_info;
    for (Int32_t i = 0; i < obstacle_main_forward_radar_list_.obstacle_num; ++i) {
    // 障碍物ID
        obstacle_radar_info.id = obstacle_main_forward_radar_list_.obstacles[i].id;
        obstacle_radar_info.type = obstacle_main_forward_radar_list_.obstacles[i].type;
        obstacle_radar_info.track_status = obstacle_main_forward_radar_list_.obstacles[i].track_status;
        obstacle_radar_info.merged_status = obstacle_main_forward_radar_list_.obstacles[i].merged_status;
        obstacle_radar_info.oncomming = obstacle_main_forward_radar_list_.obstacles[i].oncomming;
        obstacle_radar_info.bridge = obstacle_main_forward_radar_list_.obstacles[i].bridge;
        obstacle_radar_info.range = obstacle_main_forward_radar_list_.obstacles[i].range;
        obstacle_radar_info.angle = obstacle_main_forward_radar_list_.obstacles[i].angle;
        obstacle_radar_info.range_rate = obstacle_main_forward_radar_list_.obstacles[i].range_rate;
        obstacle_radar_info.range_acceleration = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;
        obstacle_radar_info.lateral_rate = obstacle_main_forward_radar_list_.obstacles[i].lateral_rate;
        obstacle_radar_info.length = obstacle_main_forward_radar_list_.obstacles[i].length;
        obstacle_radar_info.width = obstacle_main_forward_radar_list_.obstacles[i].width;
        obstacle_radar_info.x = obstacle_main_forward_radar_list_.obstacles[i].x;
        obstacle_radar_info.y = obstacle_main_forward_radar_list_.obstacles[i].y;
        obstacle_radar_info.v_x = obstacle_main_forward_radar_list_.obstacles[i].v_x;
        obstacle_radar_info.v_y = obstacle_main_forward_radar_list_.obstacles[i].v_y;
        obstacle_radar_info.accel_x = obstacle_main_forward_radar_list_.obstacles[i].accel_x;
        obstacle_radar_info.accel_y = obstacle_main_forward_radar_list_.obstacles[i].accel_y;
        obstacle_radar_info.yaw_rate = obstacle_main_forward_radar_list_.obstacles[i].yaw_rate;

        obstacle_radar_info.obj_std_dis_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_y;
        obstacle_radar_info.obj_std_dis_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_x;
        obstacle_radar_info.obj_std_vel_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_y;
        obstacle_radar_info.obj_std_vel_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_x;

        //使用range_acceleration 字段暂存相对速度
        obstacle_radar_info.rel_v_x = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;

        perception_debug_.front_radar_list.obstacles.push_back(obstacle_radar_info);

        if(target_radar_id == obstacle_main_forward_radar_list_.obstacles[i].id)
        {//
            perception_debug_.front_radar_list.target_obstacle.id = obstacle_main_forward_radar_list_.obstacles[i].id;
            perception_debug_.front_radar_list.target_obstacle.type = obstacle_main_forward_radar_list_.obstacles[i].type;
            perception_debug_.front_radar_list.target_obstacle.track_status = obstacle_main_forward_radar_list_.obstacles[i].track_status;
            perception_debug_.front_radar_list.target_obstacle.merged_status = obstacle_main_forward_radar_list_.obstacles[i].merged_status;
            perception_debug_.front_radar_list.target_obstacle.oncomming = obstacle_main_forward_radar_list_.obstacles[i].oncomming;
            perception_debug_.front_radar_list.target_obstacle.bridge = obstacle_main_forward_radar_list_.obstacles[i].bridge;
            perception_debug_.front_radar_list.target_obstacle.range = obstacle_main_forward_radar_list_.obstacles[i].range;
            perception_debug_.front_radar_list.target_obstacle.angle = obstacle_main_forward_radar_list_.obstacles[i].angle;
            perception_debug_.front_radar_list.target_obstacle.range_rate = obstacle_main_forward_radar_list_.obstacles[i].range_rate;
            perception_debug_.front_radar_list.target_obstacle.range_acceleration = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;
            perception_debug_.front_radar_list.target_obstacle.lateral_rate = obstacle_main_forward_radar_list_.obstacles[i].lateral_rate;
            perception_debug_.front_radar_list.target_obstacle.length = obstacle_main_forward_radar_list_.obstacles[i].length;
            perception_debug_.front_radar_list.target_obstacle.width = obstacle_main_forward_radar_list_.obstacles[i].width;
            perception_debug_.front_radar_list.target_obstacle.x = obstacle_main_forward_radar_list_.obstacles[i].x;
            perception_debug_.front_radar_list.target_obstacle.y = obstacle_main_forward_radar_list_.obstacles[i].y;
            perception_debug_.front_radar_list.target_obstacle.v_x = obstacle_main_forward_radar_list_.obstacles[i].v_x;
            perception_debug_.front_radar_list.target_obstacle.v_y = obstacle_main_forward_radar_list_.obstacles[i].v_y;
            perception_debug_.front_radar_list.target_obstacle.accel_x = obstacle_main_forward_radar_list_.obstacles[i].accel_x;
            perception_debug_.front_radar_list.target_obstacle.accel_y = obstacle_main_forward_radar_list_.obstacles[i].accel_y;
            perception_debug_.front_radar_list.target_obstacle.yaw_rate = obstacle_main_forward_radar_list_.obstacles[i].yaw_rate;

            perception_debug_.front_radar_list.target_obstacle.obj_std_dis_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_y;
            perception_debug_.front_radar_list.target_obstacle.obj_std_dis_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_x;
            perception_debug_.front_radar_list.target_obstacle.obj_std_vel_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_y;
            perception_debug_.front_radar_list.target_obstacle.obj_std_vel_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_x;

            //使用range_acceleration 字段暂存纵向相对速度
        #if (DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM)
            perception_debug_.front_radar_list.target_obstacle.rel_v_x = (float)test_flag;
        #else
            perception_debug_.front_radar_list.target_obstacle.rel_v_x = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;
        #endif                
            

        }

        if(target_radar_id_1 == obstacle_main_forward_radar_list_.obstacles[i].id)
        {//
            perception_debug_.front_radar_list.target_obstacle_1.id = obstacle_main_forward_radar_list_.obstacles[i].id;
            perception_debug_.front_radar_list.target_obstacle_1.type = obstacle_main_forward_radar_list_.obstacles[i].type;
            perception_debug_.front_radar_list.target_obstacle_1.track_status = obstacle_main_forward_radar_list_.obstacles[i].track_status;
            perception_debug_.front_radar_list.target_obstacle_1.merged_status = obstacle_main_forward_radar_list_.obstacles[i].merged_status;
            perception_debug_.front_radar_list.target_obstacle_1.oncomming = obstacle_main_forward_radar_list_.obstacles[i].oncomming;
            perception_debug_.front_radar_list.target_obstacle_1.bridge = obstacle_main_forward_radar_list_.obstacles[i].bridge;
            perception_debug_.front_radar_list.target_obstacle_1.range = obstacle_main_forward_radar_list_.obstacles[i].range;
            perception_debug_.front_radar_list.target_obstacle_1.angle = obstacle_main_forward_radar_list_.obstacles[i].angle;
            perception_debug_.front_radar_list.target_obstacle_1.range_rate = obstacle_main_forward_radar_list_.obstacles[i].range_rate;
            perception_debug_.front_radar_list.target_obstacle_1.range_acceleration = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;
            perception_debug_.front_radar_list.target_obstacle_1.lateral_rate = obstacle_main_forward_radar_list_.obstacles[i].lateral_rate;
            perception_debug_.front_radar_list.target_obstacle_1.length = obstacle_main_forward_radar_list_.obstacles[i].length;
            perception_debug_.front_radar_list.target_obstacle_1.width = obstacle_main_forward_radar_list_.obstacles[i].width;
            perception_debug_.front_radar_list.target_obstacle_1.x = obstacle_main_forward_radar_list_.obstacles[i].x;
            perception_debug_.front_radar_list.target_obstacle_1.y = obstacle_main_forward_radar_list_.obstacles[i].y;
            perception_debug_.front_radar_list.target_obstacle_1.v_x = obstacle_main_forward_radar_list_.obstacles[i].v_x;
            perception_debug_.front_radar_list.target_obstacle_1.v_y = obstacle_main_forward_radar_list_.obstacles[i].v_y;
            perception_debug_.front_radar_list.target_obstacle_1.accel_x = obstacle_main_forward_radar_list_.obstacles[i].accel_x;
            perception_debug_.front_radar_list.target_obstacle_1.accel_y = obstacle_main_forward_radar_list_.obstacles[i].accel_y;
            perception_debug_.front_radar_list.target_obstacle_1.yaw_rate = obstacle_main_forward_radar_list_.obstacles[i].yaw_rate;

            perception_debug_.front_radar_list.target_obstacle_1.obj_std_dis_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_y;
            perception_debug_.front_radar_list.target_obstacle_1.obj_std_dis_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_dis_x;
            perception_debug_.front_radar_list.target_obstacle_1.obj_std_vel_y = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_y;
            perception_debug_.front_radar_list.target_obstacle_1.obj_std_vel_x = obstacle_main_forward_radar_list_.obstacles[i].obj_std_vel_x;

            //使用range_acceleration 字段暂存纵向相对速度
        #if (DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM)
            perception_debug_.front_radar_list.target_obstacle.rel_v_x = (float)test_flag;
        #else
            perception_debug_.front_radar_list.target_obstacle_1.rel_v_x = obstacle_main_forward_radar_list_.obstacles[i].range_acceleration;
        #endif
        }

    }

    //前视一体机
    perception_debug_.main_camera_list.timestamp = obstacle_main_forward_camera_list_.msg_head.timestamp;
    perception_debug_.main_camera_list.sequence = obstacle_main_forward_camera_list_.msg_head.sequence;
    perception_debug_.main_camera_list.obstacle_num = obstacle_main_forward_camera_list_.obstacle_num;
    perception_debug_.main_camera_list.obstacles.clear();
    perception_debug_.main_camera_list.obstacles.reserve(obstacle_main_forward_camera_list_.obstacle_num);
    ::sensing_fusion::obstacle_camera obstacle_camera_info;
    for (Int32_t i = 0; i < obstacle_main_forward_camera_list_.obstacle_num; ++i) {
    // 障碍物ID
        obstacle_camera_info.id = obstacle_main_forward_camera_list_.obstacles[i].id;
        obstacle_camera_info.type = obstacle_main_forward_camera_list_.obstacles[i].type;
        obstacle_camera_info.status = obstacle_main_forward_camera_list_.obstacles[i].status;
        obstacle_camera_info.cut_in = obstacle_main_forward_camera_list_.obstacles[i].cut_in;
        obstacle_camera_info.blinker = obstacle_main_forward_camera_list_.obstacles[i].blinker;
        obstacle_camera_info.brake_lights = obstacle_main_forward_camera_list_.obstacles[i].brake_lights;
        obstacle_camera_info.age = obstacle_main_forward_camera_list_.obstacles[i].age;
        obstacle_camera_info.lane = obstacle_main_forward_camera_list_.obstacles[i].lane;
        obstacle_camera_info.length = obstacle_main_forward_camera_list_.obstacles[i].length;
        obstacle_camera_info.width = obstacle_main_forward_camera_list_.obstacles[i].width;
        obstacle_camera_info.height = obstacle_main_forward_camera_list_.obstacles[i].height;
        obstacle_camera_info.x = obstacle_main_forward_camera_list_.obstacles[i].x;
        obstacle_camera_info.y = obstacle_main_forward_camera_list_.obstacles[i].y;
        obstacle_camera_info.heading = obstacle_main_forward_camera_list_.obstacles[i].heading;
        obstacle_camera_info.v_x = obstacle_main_forward_camera_list_.obstacles[i].v_x;
        obstacle_camera_info.v_y = obstacle_main_forward_camera_list_.obstacles[i].v_y;
        obstacle_camera_info.accel_x = obstacle_main_forward_camera_list_.obstacles[i].accel_x;
        obstacle_camera_info.accel_y = obstacle_main_forward_camera_list_.obstacles[i].accel_y;
        obstacle_camera_info.yaw_rate = obstacle_main_forward_camera_list_.obstacles[i].yaw_rate;
        obstacle_camera_info.scale_change = obstacle_main_forward_camera_list_.obstacles[i].scale_change;
        //此处使用scale_change 字段暂存纵向相对速度
        obstacle_camera_info.rel_v_x = obstacle_main_forward_camera_list_.obstacles[i].scale_change;
        
        perception_debug_.main_camera_list.obstacles.push_back(obstacle_camera_info);



        if(target_camera_id == obstacle_main_forward_camera_list_.obstacles[i].id)
        {//
            perception_debug_.main_camera_list.target_obstacle.id = obstacle_main_forward_camera_list_.obstacles[i].id;
            perception_debug_.main_camera_list.target_obstacle.type = obstacle_main_forward_camera_list_.obstacles[i].type;
            perception_debug_.main_camera_list.target_obstacle.status = obstacle_main_forward_camera_list_.obstacles[i].status;
            perception_debug_.main_camera_list.target_obstacle.cut_in = obstacle_main_forward_camera_list_.obstacles[i].cut_in;
            perception_debug_.main_camera_list.target_obstacle.blinker = obstacle_main_forward_camera_list_.obstacles[i].blinker;
            perception_debug_.main_camera_list.target_obstacle.brake_lights = obstacle_main_forward_camera_list_.obstacles[i].brake_lights;
            perception_debug_.main_camera_list.target_obstacle.age = obstacle_main_forward_camera_list_.obstacles[i].age;
            perception_debug_.main_camera_list.target_obstacle.lane = obstacle_main_forward_camera_list_.obstacles[i].lane;
            perception_debug_.main_camera_list.target_obstacle.length = obstacle_main_forward_camera_list_.obstacles[i].length;
            perception_debug_.main_camera_list.target_obstacle.width = obstacle_main_forward_camera_list_.obstacles[i].width;
            perception_debug_.main_camera_list.target_obstacle.height = obstacle_main_forward_camera_list_.obstacles[i].height;
            perception_debug_.main_camera_list.target_obstacle.x = obstacle_main_forward_camera_list_.obstacles[i].x;
            perception_debug_.main_camera_list.target_obstacle.y = obstacle_main_forward_camera_list_.obstacles[i].y;
            perception_debug_.main_camera_list.target_obstacle.heading = obstacle_main_forward_camera_list_.obstacles[i].heading;
            perception_debug_.main_camera_list.target_obstacle.v_x = obstacle_main_forward_camera_list_.obstacles[i].v_x;
            perception_debug_.main_camera_list.target_obstacle.v_y = obstacle_main_forward_camera_list_.obstacles[i].v_y;
            perception_debug_.main_camera_list.target_obstacle.accel_x = obstacle_main_forward_camera_list_.obstacles[i].accel_x;
            perception_debug_.main_camera_list.target_obstacle.accel_y = obstacle_main_forward_camera_list_.obstacles[i].accel_y;
            perception_debug_.main_camera_list.target_obstacle.yaw_rate = obstacle_main_forward_camera_list_.obstacles[i].yaw_rate;
            perception_debug_.main_camera_list.target_obstacle.scale_change = obstacle_main_forward_camera_list_.obstacles[i].scale_change;  

            perception_debug_.main_camera_list.target_obstacle.rel_v_x = obstacle_main_forward_camera_list_.obstacles[i].scale_change;
        }

        if(target_camera_id_1 == obstacle_main_forward_camera_list_.obstacles[i].id)
        {//
            perception_debug_.main_camera_list.target_obstacle_1.id = obstacle_main_forward_camera_list_.obstacles[i].id;
            perception_debug_.main_camera_list.target_obstacle_1.type = obstacle_main_forward_camera_list_.obstacles[i].type;
            perception_debug_.main_camera_list.target_obstacle_1.status = obstacle_main_forward_camera_list_.obstacles[i].status;
            perception_debug_.main_camera_list.target_obstacle_1.cut_in = obstacle_main_forward_camera_list_.obstacles[i].cut_in;
            perception_debug_.main_camera_list.target_obstacle_1.blinker = obstacle_main_forward_camera_list_.obstacles[i].blinker;
            perception_debug_.main_camera_list.target_obstacle_1.brake_lights = obstacle_main_forward_camera_list_.obstacles[i].brake_lights;
            perception_debug_.main_camera_list.target_obstacle_1.age = obstacle_main_forward_camera_list_.obstacles[i].age;
            perception_debug_.main_camera_list.target_obstacle_1.lane = obstacle_main_forward_camera_list_.obstacles[i].lane;
            perception_debug_.main_camera_list.target_obstacle_1.length = obstacle_main_forward_camera_list_.obstacles[i].length;
            perception_debug_.main_camera_list.target_obstacle_1.width = obstacle_main_forward_camera_list_.obstacles[i].width;
            perception_debug_.main_camera_list.target_obstacle_1.height = obstacle_main_forward_camera_list_.obstacles[i].height;
            perception_debug_.main_camera_list.target_obstacle_1.x = obstacle_main_forward_camera_list_.obstacles[i].x;
            perception_debug_.main_camera_list.target_obstacle_1.y = obstacle_main_forward_camera_list_.obstacles[i].y;
            perception_debug_.main_camera_list.target_obstacle_1.heading = obstacle_main_forward_camera_list_.obstacles[i].heading;
            perception_debug_.main_camera_list.target_obstacle_1.v_x = obstacle_main_forward_camera_list_.obstacles[i].v_x;
            perception_debug_.main_camera_list.target_obstacle_1.v_y = obstacle_main_forward_camera_list_.obstacles[i].v_y;
            perception_debug_.main_camera_list.target_obstacle_1.accel_x = obstacle_main_forward_camera_list_.obstacles[i].accel_x;
            perception_debug_.main_camera_list.target_obstacle_1.accel_y = obstacle_main_forward_camera_list_.obstacles[i].accel_y;
            perception_debug_.main_camera_list.target_obstacle_1.yaw_rate = obstacle_main_forward_camera_list_.obstacles[i].yaw_rate;
            perception_debug_.main_camera_list.target_obstacle_1.scale_change = obstacle_main_forward_camera_list_.obstacles[i].scale_change;  

            perception_debug_.main_camera_list.target_obstacle_1.rel_v_x = obstacle_main_forward_camera_list_.obstacles[i].scale_change;
        }

    }


    // obstacles from left fw radar
    ad_msg::ObstacleRadarList obstacle_left_forward_radar_list_before;
    // obstacles from right fw radar
    ad_msg::ObstacleRadarList obstacle_right_forward_radar_list_before;
    // obstacles from left bw radar
    ad_msg::ObstacleRadarList obstacle_left_backward_radar_list_before;
    // obstacles from right bw radar
    ad_msg::ObstacleRadarList obstacle_right_backward_radar_list_before; 

    SharedData* shared_data = SharedData::instance();

    //获取左前侧雷达障碍物信息
    shared_data->GetObstacleRadarLeftFwFilterList(&obstacle_left_forward_radar_list_before);
    //获取右前侧雷达障碍物信息
    shared_data->GetObstacleRadarRightFwFilterList(&obstacle_right_forward_radar_list_before);
    //获取左后侧雷达障碍物信息
    shared_data->GetObstacleRadarLeftBwFilterList(&obstacle_left_backward_radar_list_before);
    //获取右后侧雷达障碍物信息
    shared_data->GetObstacleRadarRightBwFilterList(&obstacle_right_backward_radar_list_before);

    /**
     * @brief 侧雷达数据调试
     * 
     */
    handle_around_radar_debug_data( perception_debug_.left_forward_radar_list, \
                                    obstacle_left_forward_radar_list_before, \
                                    obstacle_left_forward_radar_list_, \
                                    target_radar_left_fw_before_id, \
                                    target_radar_left_fw_after_id);
    handle_around_radar_debug_data( perception_debug_.right_forward_radar_list, \
                                    obstacle_right_forward_radar_list_before, \
                                    obstacle_right_forward_radar_list_, \
                                    target_radar_right_fw_before_id, \
                                    target_radar_right_fw_after_id);
    handle_around_radar_debug_data( perception_debug_.left_backward_radar_list, \
                                    obstacle_left_backward_radar_list_before, \
                                    obstacle_left_backward_radar_list_, \
                                    target_radar_left_bw_before_id, \
                                    target_radar_left_bw_after_id);
    handle_around_radar_debug_data( perception_debug_.right_backward_radar_list, \
                                    obstacle_right_backward_radar_list_before, \
                                    obstacle_right_backward_radar_list_, \
                                    target_radar_right_bw_before_id, \
                                    target_radar_right_bw_after_id);

    //车身数据
    perception_debug_.chassis.timestamp = chassis_.msg_head.timestamp;
    perception_debug_.chassis.sequence = chassis_.msg_head.sequence;
    perception_debug_.chassis.driving_mode = chassis_.driving_mode;
    perception_debug_.chassis.e_stop = chassis_.e_stop;
    perception_debug_.chassis.eps_status = chassis_.eps_status;
    perception_debug_.chassis.throttle_sys_status = chassis_.throttle_sys_status;
    perception_debug_.chassis.ebs_status = chassis_.ebs_status;
    perception_debug_.chassis.steering_wheel_angle_valid = chassis_.steering_wheel_angle_valid;
    perception_debug_.chassis.steering_wheel_angle = chassis_.steering_wheel_angle;
    perception_debug_.chassis.steering_wheel_speed_valid = chassis_.steering_wheel_speed_valid;
    perception_debug_.chassis.steering_wheel_speed = chassis_.steering_wheel_speed;
    perception_debug_.chassis.steering_wheel_torque_valid = chassis_.steering_wheel_torque_valid;
    perception_debug_.chassis.steering_wheel_torque = chassis_.steering_wheel_torque;
    perception_debug_.chassis.v_valid = chassis_.v_valid;
    perception_debug_.chassis.v = chassis_.v;
    perception_debug_.chassis.a_valid = chassis_.a_valid;
    perception_debug_.chassis.a = chassis_.a;
    perception_debug_.chassis.yaw_rate_valid = chassis_.yaw_rate_valid;
    perception_debug_.chassis.yaw_rate = chassis_.yaw_rate;
    perception_debug_.chassis.ax_valid = chassis_.ax_valid;
    perception_debug_.chassis.ax = chassis_.ax;
    perception_debug_.chassis.ay_valid = chassis_.ay_valid;
    perception_debug_.chassis.ay = chassis_.ay;
    perception_debug_.chassis.wheel_speed_fl_valid = chassis_.wheel_speed_fl_valid;
    perception_debug_.chassis.wheel_speed_fl = chassis_.wheel_speed_fl;
    perception_debug_.chassis.wheel_speed_fr_valid = chassis_.wheel_speed_fr_valid;
    perception_debug_.chassis.wheel_speed_fr = chassis_.wheel_speed_fr;
    perception_debug_.chassis.wheel_speed_rl_valid = chassis_.wheel_speed_rl_valid;
    perception_debug_.chassis.wheel_speed_rl = chassis_.wheel_speed_rl;
    perception_debug_.chassis.wheel_speed_rr_valid = chassis_.wheel_speed_rr_valid;
    perception_debug_.chassis.wheel_speed_rr = chassis_.wheel_speed_rr;
    perception_debug_.chassis.wheel_speed_rl2_valid = chassis_.wheel_speed_rl2_valid;
    perception_debug_.chassis.wheel_speed_rl2 = chassis_.wheel_speed_rl2;
    perception_debug_.chassis.wheel_speed_rr2_valid = chassis_.wheel_speed_rr2_valid;
    perception_debug_.chassis.wheel_speed_rr2 = chassis_.wheel_speed_rr2;
    perception_debug_.chassis.epb_status = chassis_.epb_status;
    perception_debug_.chassis.gear = chassis_.gear;
    perception_debug_.chassis.gear_number = chassis_.gear_number;
    perception_debug_.chassis.signal_turning_indicator = chassis_.signal_turning_indicator;
    perception_debug_.chassis.signal_turn_lamp = chassis_.signal_turn_lamp;
    perception_debug_.chassis.signal_brake_lamp = chassis_.signal_brake_lamp;
    perception_debug_.chassis.brake_pedal_value = chassis_.brake_pedal_value;
    perception_debug_.chassis.acc_pedal_value = chassis_.acc_pedal_value;
    perception_debug_.chassis.engine_speed_valid = chassis_.engine_speed_valid;
    perception_debug_.chassis.engine_speed = chassis_.engine_speed;
    perception_debug_.chassis.engine_torque_valid = chassis_.engine_torque_valid;
    perception_debug_.chassis.engine_torque = chassis_.engine_torque;
    perception_debug_.chassis.gross_weight_vaild = chassis_.gross_weight_vaild;
    perception_debug_.chassis.trailer_status = chassis_.trailer_status;
    perception_debug_.chassis.trailer_l = chassis_.trailer_l;
    perception_debug_.chassis.trailer_w = chassis_.trailer_w;
    perception_debug_.chassis.trailer_h = chassis_.trailer_h;


}


void WorkSensorsFusion::handle_around_radar_debug_data(sensing_fusion::obstacle_radar_list & output_radar_list, ad_msg::ObstacleRadarList & input_radar_list,  
                              ad_msg::ObstacleRadarList & input_radar_list_preprocess, int target_id_before, int target_id_after)
{
    for(int i = 0; i < input_radar_list.obstacle_num; i++){
        if(target_id_before == input_radar_list.obstacles[i].id){
            output_radar_list.target_obstacle_before.id = input_radar_list.obstacles[i].id;
            output_radar_list.target_obstacle_before.type = input_radar_list.obstacles[i].type;
            output_radar_list.target_obstacle_before.track_status = input_radar_list.obstacles[i].track_status;
            output_radar_list.target_obstacle_before.merged_status = input_radar_list.obstacles[i].merged_status;
            output_radar_list.target_obstacle_before.oncomming = input_radar_list.obstacles[i].oncomming;
            output_radar_list.target_obstacle_before.bridge = input_radar_list.obstacles[i].bridge;
            output_radar_list.target_obstacle_before.range = input_radar_list.obstacles[i].range;
            output_radar_list.target_obstacle_before.angle = input_radar_list.obstacles[i].angle;
            output_radar_list.target_obstacle_before.range_rate = input_radar_list.obstacles[i].range_rate;
            output_radar_list.target_obstacle_before.range_acceleration = input_radar_list.obstacles[i].range_acceleration;
            output_radar_list.target_obstacle_before.lateral_rate = input_radar_list.obstacles[i].lateral_rate;
            output_radar_list.target_obstacle_before.length = input_radar_list.obstacles[i].length;
            output_radar_list.target_obstacle_before.width = input_radar_list.obstacles[i].width;
            output_radar_list.target_obstacle_before.x = input_radar_list.obstacles[i].x;
            output_radar_list.target_obstacle_before.y = input_radar_list.obstacles[i].y;
            output_radar_list.target_obstacle_before.v_x = input_radar_list.obstacles[i].v_x;
            output_radar_list.target_obstacle_before.v_y = input_radar_list.obstacles[i].v_y;
            output_radar_list.target_obstacle_before.accel_x = input_radar_list.obstacles[i].accel_x;
            output_radar_list.target_obstacle_before.accel_y = input_radar_list.obstacles[i].accel_y;
            output_radar_list.target_obstacle_before.yaw_rate = input_radar_list.obstacles[i].yaw_rate;

            output_radar_list.target_obstacle_before.obj_std_dis_y = input_radar_list.obstacles[i].obj_std_dis_y;
            output_radar_list.target_obstacle_before.obj_std_dis_x = input_radar_list.obstacles[i].obj_std_dis_x;
            output_radar_list.target_obstacle_before.obj_std_vel_y = input_radar_list.obstacles[i].obj_std_vel_y;
            output_radar_list.target_obstacle_before.obj_std_vel_x = input_radar_list.obstacles[i].obj_std_vel_x;

        }
    }
    
    for(int i = 0; i < input_radar_list_preprocess.obstacle_num; i++){ 

        if(target_id_after == input_radar_list_preprocess.obstacles[i].id)
        {//
            output_radar_list.target_obstacle_after.id = input_radar_list_preprocess.obstacles[i].id;
            output_radar_list.target_obstacle_after.type = input_radar_list_preprocess.obstacles[i].type;
            output_radar_list.target_obstacle_after.track_status = input_radar_list_preprocess.obstacles[i].track_status;
            output_radar_list.target_obstacle_after.merged_status = input_radar_list_preprocess.obstacles[i].merged_status;
            output_radar_list.target_obstacle_after.oncomming = input_radar_list_preprocess.obstacles[i].oncomming;
            output_radar_list.target_obstacle_after.bridge = input_radar_list_preprocess.obstacles[i].bridge;
            output_radar_list.target_obstacle_after.range = input_radar_list_preprocess.obstacles[i].range;
            output_radar_list.target_obstacle_after.angle = input_radar_list_preprocess.obstacles[i].angle;
            output_radar_list.target_obstacle_after.range_rate = input_radar_list_preprocess.obstacles[i].range_rate;
            output_radar_list.target_obstacle_after.range_acceleration = input_radar_list_preprocess.obstacles[i].range_acceleration;
            output_radar_list.target_obstacle_after.lateral_rate = input_radar_list_preprocess.obstacles[i].lateral_rate;
            output_radar_list.target_obstacle_after.length = input_radar_list_preprocess.obstacles[i].length;
            output_radar_list.target_obstacle_after.width = input_radar_list_preprocess.obstacles[i].width;
            output_radar_list.target_obstacle_after.x = input_radar_list_preprocess.obstacles[i].x;
            output_radar_list.target_obstacle_after.y = input_radar_list_preprocess.obstacles[i].y;
            output_radar_list.target_obstacle_after.v_x = input_radar_list_preprocess.obstacles[i].v_x;
            output_radar_list.target_obstacle_after.v_y = input_radar_list_preprocess.obstacles[i].v_y;
            output_radar_list.target_obstacle_after.accel_x = input_radar_list_preprocess.obstacles[i].accel_x;
            output_radar_list.target_obstacle_after.accel_y = input_radar_list_preprocess.obstacles[i].accel_y;
            output_radar_list.target_obstacle_after.yaw_rate = input_radar_list_preprocess.obstacles[i].yaw_rate;

            output_radar_list.target_obstacle_after.obj_std_dis_y = input_radar_list_preprocess.obstacles[i].obj_std_dis_y;
            output_radar_list.target_obstacle_after.obj_std_dis_x = input_radar_list_preprocess.obstacles[i].obj_std_dis_x;
            output_radar_list.target_obstacle_after.obj_std_vel_y = input_radar_list_preprocess.obstacles[i].obj_std_vel_y;
            output_radar_list.target_obstacle_after.obj_std_vel_x = input_radar_list_preprocess.obstacles[i].obj_std_vel_x;

        }

    }

}




} // namespace framework
} // namespace phoenix
