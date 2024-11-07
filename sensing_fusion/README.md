-----------------------------------------------20220822-----------------------------------------------------------------------
sensing_fusion使用说明：
1.启动sensing和sensing_fusion时，回放数据使用下列命令：
    rosbag play xxx.bag --topics /perception/camera_canfd_frame /perception/ars_radar_can_frame /control/chassis

2.只启动sensing_fusion时，回放数据使用下列命令：
    rosbag play xxx.bag

3.话题说明
接受的话题：
    车辆数据：
        /control/chassis：车辆数据

    解析之后的障碍物话题：
        输出前视一体机数据的话题：
            /perception/lane_mark_camera
            /perception/traffic_signal
            /perception/obstacle_maxieye_camera

        输出前向毫米波雷达数据的话题：
            /perception/obstacle_ars_radar

        输出视觉控制器数据的话题：
            /perception/obstacle_visual_control_front
            /perception/obstacle_visual_control_front_left
            /perception/obstacle_visual_control_front_right
            /perception/obstacle_visual_control_rear
            /perception/obstacle_visual_control_rear_left
            /perception/obstacle_visual_control_rear_right

        输出侧向毫米波雷达数据的话题：
            /perception/obstacle_anngic_radar_front_left
            /perception/obstacle_anngic_radar_front_right
            /perception/obstacle_anngic_radar_rear_left
            /perception/obstacle_anngic_radar_rear_right

        输出激光雷达数据的话题：
            /perception/obstacle_lidar_0

        输出给rviz的话题：
            /visualization/lane_mark：前视一体机车道线数据
            /visualization/maxieye：前视一体机障碍物数据
            /visualization/ars：前向毫米波雷达障碍物数据

发送的话题：
    /perception/obstacle_fusion

备注：
    数据流向：bag包/sensing-->发送解析之后的数据-->sensing_fusion接受解析之后的障碍物数据以及车身数据-->sensing_fusion启动融合算法

