-----------------------------------------------20230327-----------------------------------------------------------------------
csv生成步骤：
1.需要在回放rosbag时在后面添加--clock，这一步是为了获取rosbag包内部的时间戳。
2.打开csv生成的宏 ENTER_OBJ_INFO_TO_CSV，该宏在sensing/src/common/include/utils/macros.h下，0为关闭，1为打开。

-----------------------------------------------20230327-----------------------------------------------------------------------
1.车身的各个数据(定位点默认为后轴中心)
  Base::SetVehicleLength(7.0F); //车辆长度：7.0m
  Base::SetVehicleWidth(2.5F);  //车辆宽度：2.5m
  Base::SetDistOfLocalizationToFront(5.5F); //定位点到车头的距离：5.5m
  Base::SetDistOfLocalizationToRear(1.5F);  //定位点到车尾的距离：1.5m
  Base::SetDistOfLocalizationToCenter(2.0F);  //定位点到车辆中心的距离：2.0m

  Base::SetMaxSteeringAngle(common::com_deg2rad(600.0F));   //车辆最大方向盘角度：600度
  Base::SetWheelbase(4.05F);    //车辆轴距：4.05m

  steering_gear_ratio_ = 16.0F; //传动比：16.0

2.显示界面车辆中心点(在显示界面画车辆)
void WidgetMap::DrawVehicle() {
#if 0
  Float64_t heading = module_info_.filtered_pose.heading;
  phoenix::common::Vec2d pos(
        module_info_.filtered_pose.x, module_info_.filtered_pose.y);
  phoenix::common::OBB_2D obb;

  const Float64_t half_veh_width =
      module_info_.motion_plan_config.vehicle_width * 0.5;
  const Float64_t half_veh_length =
      module_info_.motion_plan_config.vehicle_length * 0.5;
#else
  /*
  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  const Float64_t half_veh_width = 1.25;
  const Float64_t half_veh_length = 3.0;
  const Float64_t veh_height = 2.0;
  */

  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  veh_model::VehicleModelWrapper veh_model;
  const Float64_t half_veh_width = 0.5F * veh_model.GetVehicleWidth();
  const Float64_t half_veh_length = 0.5F * veh_model.GetVehicleLength();
  const Float64_t veh_height = 2.0;
  const Float64_t dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();
#endif

  obb.set_unit_direction(phoenix::common::CosLookUp(heading),
                         phoenix::common::SinLookUp(heading));
  /**
   * @brief 
   * 车辆中心原点为车辆中心：obb.set_center(pos + 0 * obb.unit_direction_x());
   * 将车辆中心原点偏移到后轴中心：obb.set_center(pos + dist_of_localization_to_center * obb.unit_direction_x());
   */
  obb.set_center(pos + dist_of_localization_to_center * obb.unit_direction_x());
  obb.set_extents(half_veh_length, half_veh_width);

  glLineWidth(1.0);

  // DrawOBB_2D(obb);
  phoenix::common::Vec2d corner[4];
  corner[0] = obb.center() + obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[1] = obb.center() - obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[2] = obb.center() - obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();
  corner[3] = obb.center() + obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();

  glColor3d(1.0, 1.0, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), 0.0);
  }
  glEnd();

  glColor3d(1.0, 0.5, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), veh_height);
    DrawVertex(corner[1](0), corner[1](1), veh_height);
    DrawVertex(corner[2](0), corner[2](1), veh_height);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  glColor3d(0.0, 0.5, 1.0);
  glBegin(GL_LINES);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[0](0), corner[0](1), veh_height);

    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), veh_height);

    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), veh_height);

    DrawVertex(corner[3](0), corner[3](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  glColor3d(1.0, 1.0, 0.3);
  DrawEllipse(pos.x(), pos.y(), 0.5, 0.5, heading, 0.0, 3);
}

-----------------------------------------------20230322-----------------------------------------------------------------------
如何将AD_HMI中的图片数据保存成png图片：
rosbag中的压缩图片数据话题为/rtsp/compressed,其中的数据类型为sensor_msgs/CompressedImage。
1.将压缩的图像类型sensor_msgs/CompressedImage 转换成sensor_msgs/Image 类型，输入以下命令：
rosrun image_transport republish compressed in:=/rtsp raw out:=/realsense/color/image_raw

compressed in表示这个节点要输入的是压缩的图像，然后图像话题是/rtsp。注意一般相机驱动压缩图像话题会在后面再加一个compressed，也就是实际录制的是/rtsp/compressed，但是这里输入命令的时候要把后面的compressed去掉。
raw output表示这个节点转化后要输出的是原始的图像sensor_msgs/CompressedImage，图像话题是/realsense/color/image_raw(可自由设置)。然后后面rivz或者代码中要订阅原始图像的话，直接订阅这个话题就可以了。

2.将原始图像数据sensor_msgs/CompressedImage保存为png图片
rosrun image_view image_saver image:=/realsense/color/image_raw _filename_format:="%04i.png" _encoding:=bgr8

参数分别为：图像的topic、文件名（%04i是指4位数的数字）、文件的编码32FC1和bgr8

-----------------------------------------------20230320-----------------------------------------------------------------------
1.control_planning目录说明：
  control:控制程序，主要是输出车身数据，已注释下发控制指令的函数，可与ADECU同时启动互不干扰。
  control_Add_ADHMI:在control的基础上添加王能灯AD_HMI自定义的话题，并订阅该话题并解析下发。
  planning:决策程序。

2.回放王能灯AD_HMI的rosbag包：
  前置条件：需要将sensing/src/common/include/utils/macros.h和control_Add_ADHMI/src/common_c/include/utils/macros.h下的宏定义ENTER_PLAYBACK_MODE和ENTER_PLAYBACK_MODE_ADHMI都设为1。
  
  需要启动的程序：
    sensing 
    -- 程序所在目录：sensing/build/devel/lib/sensing/sensing

    sensing_fusion 
    -- 程序所在目录：sensing_fusion_wei/build/devel/lib/sensing_fusion/sensing_fusion

    control_Add_ADHMI 
    -- 程序所在目录：control_Add_ADHMI/build/devel/lib/control/control

-----------------------------------------------20230315-----------------------------------------------------------------------
更新说明：
1.为了适配AD_HMI的ros话题，添加了AD_HMI自定义的msg文件，并订阅AD_HMI /can_0(激光雷达) /can_1(前毫米波雷达430) /can_2(相机) /can_3(侧向雷达) 话题，并解析下发到sensing_fusion。
2.回放AD_HMI ros设置：
  将sensing/src/common/include/utils/macros.h中ENTER_PLAYBACK_MODE和ENTER_PLAYBACK_MODE_ADHMI都设为1。

-----------------------------------------------20220905-----------------------------------------------------------------------
更新说明：
一 安装protobuf 3.9.1后，编译过程中出现缺失stringprintf.h的解决方案：
    将protubuf 3.9.1源码中的stringprintf.h文件(路径：protobuf-3.9.1/src/google/protobuf/stubs)，复制到protobuf 3.9.1安装之后的文件夹中(例：/usr/local/protobuf-3.9.1/include/google/protobuf/stubs)。

-----------------------------------------------20220830-----------------------------------------------------------------------
更新说明：
一 添加其他版本的发布chassis话题工具：
1.将之前的20.04版本的工具移动到control/control_20.04中。
2.添加16.04和18.04版本的工具，使用方法与20.04版本的工具一样。

二 更改侧向毫米波雷达和环视相机障碍物的颜色
    侧向毫米波雷达为绿色渐变，环视相机为红色渐变。

-----------------------------------------------20220829-----------------------------------------------------------------------
发布chassis话题工具使用说明：
一 工具内容及所在目录
    工具在sensing/control文件夹内，包含一个可执行文件control以及配置文件conf/control_config.xml。

二 工具使用说明
1.接线：
    将车身的PCAN/BCAN/CCAN/RCAN接到ZLG CAN的CAN4/5/6/7上

2.使用：
    将ZLG CAN上的网线连到电脑上，并将IP改为192.168.1.xx，并启动可执行文件control即可。

备注：
    当前sensing中的工具是在Ubuntu20.04上进行编译的，所以可能只有Ubuntu20.04才能使用。如果需要在其他版本的Ubuntu上运行，需要找周耀阳要源码进行编译。

-----------------------------------------------20220822-----------------------------------------------------------------------
sensing使用说明：
一 如何启动sensing工程
1.编译sensing工程：
    命令：bash build.sh

2.启动sensing工程：
    命令：bash run.sh

二 传感器接受解析模块
1.前视一体机/视觉控制器：
    原始数据格式：CANFD     ADECU通道：CANFD7     所在文件：task_recv_maxieye_data
2.前向毫米波雷达：
    原始数据格式：CAN       ADECU通道：CAN6       所在文件：task_recv_ars_data_front
3.侧向毫米波雷达：
    原始数据格式：CANFD     ADECU通道：CANFD8     所在文件：task_recv_anngic_radar_data
4.激光雷达：
    原始数据格式：CANFD     ADECU通道：CANFD5     所在文件：task_recv_lidar_data_front

三 如何连接ZLG CAN/CANFD
1.ZLG CAN/CANFD的配置
    ZLG CAN各个通道的配置及网线端口IP：
        网线端口IP：192.168.1.177
        CAN0：channel = 0，bit_rate = CAN_BIT_RATE_500K，port = 4001，notify_port = 6001
        CAN1：channel = 1，bit_rate = CAN_BIT_RATE_500K，port = 4002，notify_port = 6002
        CAN2：channel = 2，bit_rate = CAN_BIT_RATE_500K，port = 4003，notify_port = 6003
        CAN3：channel = 3，bit_rate = CAN_BIT_RATE_500K，port = 4004，notify_port = 6004
        CAN4：channel = 4，bit_rate = CAN_BIT_RATE_500K，port = 4005，notify_port = 6005
        CAN5：channel = 5，bit_rate = CAN_BIT_RATE_500K，port = 4006，notify_port = 6006
        CAN6：channel = 6，bit_rate = CAN_BIT_RATE_500K，port = 4007，notify_port = 6007
        CAN7：channel = 7，bit_rate = CAN_BIT_RATE_500K，port = 4008，notify_port = 6008

    备注：要想使用ZLG CAN/CANFD接受数据，需要将电脑IP改为192.168.1.xx。

    ZLG CANFD各个通道的配置及网线端口IP：
        网线端口：192.168.1.178
        CAN0：channel = 0，port = 8000 (CANFD数据)
        CAN1：channel = 1，port = 8001 (CANFD数据)
        CAN2：channel = 2，port = 8002 (CAN数据)
        CAN3：channel = 3，port = 8003 (CAN数据)
    备注：ZLG CANFD CAN2/CAN3被设置为了CAN通道，所以要想接受到CANFD数据，只能插CAN0/CAN1。
        
2.用ZLG CAN/CANFD接受前向毫米波雷达Can数据
    用ZLG CAN接受前向毫米波雷达CAN数据的相关配置：
        头文件：#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
        类：can_channel_ = new can_dev::CanDriverZlgCanNet();
        通道配置：can_dev::CanChannelParam can_channel_param;
                can_channel_param.channel = 0;
                can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
                com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.177");
                can_channel_param.can_net.port = 4001;
                can_channel_param.can_net.notify_port = 6001;
        所接的ZLG CAN通道：CAN0
        备注：如果需要接其他的ZLG CAN通道，只需要将channel，port，notify_port改为对应的值即可。
    
    用ZLG CANFD接受前向毫米波雷达CAN数据的相关配置：
        头文件：#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
        类：can_channel_ = new can_dev::CanDriverZlgCanNetFd();
        通道配置：can_dev::CanChannelParam can_channel_param;
                can_channel_param.channel = 2;
                can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
                com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
                can_channel_param.can_net.port = 8002;
        所接的ZLG CANFD通道：CAN2
        备注：如果需要接其他的ZLG CANFD通道，只需要将channel，port改为对应的值即可。

3.用ZLG CANFD接受前视一体机/视觉控制器/侧向毫米波雷达/激光雷达CANFD数据
    以前视一体机为例：
        头文件：#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
        类：can_channel_ = new can_dev::CanDriverZlgCanNetFd();
        通道配置：can_dev::CanChannelParam can_channel_param;
                can_channel_param.channel = 0;
                can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
                com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
                can_channel_param.can_net.port = 8000;
        所接的ZLG CANFD通道：CAN0
        备注：如果需要接其他的ZLG CANFD通道，只需要将channel，port改为对应的值即可。

四 如何录制前视一体机/视觉控制器/前向毫米波雷达/侧向毫米波雷达/激光雷达的CAN/CANFD数据和解析之后的数据
1.启动sensing后，用rostopic list查看是否有对应的话题
    发送的话题：
        原始数据话题：
            /perception/camera_canfd_frame：前视一体机/视觉控制器CANFD数据。
            /perception/ars_radar_can_frame：前向毫米波雷达CAN数据。
            /perception/anngic_radar_canfd_frame：侧向毫米波雷达CANFD数据。
            /perception/lidar_canfd_frame：激光雷达CANFD数据。

        解析之后的障碍物话题：
            输出前视一体机数据的话题：
                /perception/lane_mark_camera
                /perception/lane_curb_camera
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


    接受的话题：
        /control/chassis：车辆数据

2.录制话题数据
    命令：rosbag record -a

备注：
    数据流向：传感器-->发送CAN/CANFD数据-->sensing接受CAN/CANFD数据-->把CAN/CANFD数据转换为话题数据并发送/解析CAN/CANFD并把解析之后的数据转换为话题数据并发送-->录制数据

五 如何回放录制的数据
1.让sensing工程进入回放模式
    修改sensing/src/common/include/utils/macros.h中ENTER_PLAYBACK_MODE宏定义的值，使其为1，并进行编译。

2.启动sensing工程和打开rviz
    命令：bash run.sh -u
    备注：添加-u后，在打开sensing的同时也会启动rviz

3.回放录制的数据
    rosbag play xxx.bag --topics /perception/camera_canfd_frame /perception/ars_radar_can_frame /control/chassis

    备注：为什么不rosbag play xxx.bag?
        因为回放的时候sensing在发送障碍物话题，bag包也在发送障碍物话题，两者会对fusion节点产生干扰。

4.话题说明
    接受的话题：
        CAN/CANFD数据话题：
            /perception/camera_canfd_frame：包含前视一体机/视觉控制器CANFD数据。
            /perception/ars_radar_can_frame：前向毫米波雷达CAN数据。

        包含融合障碍物数据的话题：
            /perception/obstacle_fusion
            
    发送的话题：
        包含前视一体机车道线/交通标志(限速标牌)/障碍物数据的话题：
            /perception/lane_mark_camera
            /perception/lane_curb_camera
            /perception/traffic_signal
            /perception/obstacle_maxieye_camera

        包含前向毫米波障碍物数据的话题：   
            /perception/obstacle_ars_radar

        包含侧向毫米波雷达障碍物数据的话题：
            /perception/obstacle_anngic_radar_front_left
            /perception/obstacle_anngic_radar_front_right
            /perception/obstacle_anngic_radar_rear_left
            /perception/obstacle_anngic_radar_rear_right
        
        包含视觉控制器障碍物数据的话题：
            /perception/obstacle_visual_control_front
            /perception/obstacle_visual_control_front_left
            /perception/obstacle_visual_control_front_right
            /perception/obstacle_visual_control_rear
            /perception/obstacle_visual_control_rear_left
            /perception/obstacle_visual_control_rear_right

        包含激光雷达障碍物数据的话题：
            /perception/obstacle_lidar_0
        
        给rviz数据的话题：
            /visualization/lane_mark：前视一体机车道线数据
            /visualization/maxieye：前视一体机障碍物数据
            /visualization/ars：前向毫米波雷达障碍物数据

备注：
    数据流向：xxx.bag-->发送包含CAN/CANFD数据的话题-->sensing接受话题-->把CAN/CANFD数据提取出来进行解析成车道线/障碍物数据-->发送车道线/障碍物数据给sensing_fusion和rviz

六 录制车身数据
    PCAN BCAN CCAN RCAN接ZLG CAN CAN4 CAN5 CAN6 CAN7，然后启动control程序

七 存在的问题
    因为现在视觉控制器和侧向毫米波雷达硬件还未接通，所以解析部分还未实车测试，可能存在问题。
    普通模式下：
        视觉控制器和侧向毫米波雷达解析可能有问题。
    回放模式下：
        未添加视觉控制器和侧向毫米波雷达原始数据的接受及解析。

-----------------------------------------------20220705-----------------------------------------------------------------------
代码22#实车测试结果：
    1）前视一体机无交通标志报文输出
    2）激光雷达控制器接上激光雷达时，能正常接受解析显示障碍物数据

-----------------------------------------------20220704-----------------------------------------------------------------------
代码22#实车测试结果：
    1）后左侧向毫米波雷达显示不出来(显示为点，不容易辨别)
    2）后左侧向毫米波雷达障碍物数据有问题(障碍物出现在右后方)
    3）激光雷达有通信，但无数据(激光雷达未接上控制器)

-----------------------------------------------20220701-----------------------------------------------------------------------
代码更新点：
    1）新增视觉控制器(CANFD)目标障碍物数据解析及显示

代码更新位置：
    1）视觉控制器解析模块在src/framework/sensor/task_recv_visual_control_data中。

变更测试：
    未进行测试。

变更人：
    李维龙

-----------------------------------------------20220630-----------------------------------------------------------------------
代码更新点：
    1）新增侧向毫米波雷达(CANFD)目标障碍物数据解析及显示

代码更新位置：
    1）侧向毫米波雷达解析模块在src/framework/sensor/task_recv_anngic_radar_data中。

变更测试：
    未进行测试。

变更人：
    李维龙

-----------------------------------------------20220629-----------------------------------------------------------------------
代码更新点：
    1）新增前向激光雷达(CANFD)目标障碍物数据解析及显示
    2）新增前视一体机(CANFD)交通标志数据解析

代码更新位置：
    1）前向激光雷达解析模块在src/framework/sensor/task_recv_lidar_data_front中。
    2）前视一体机交通标志数据解析在src/framework/sensor/task_recv_maxieye_data中。

变更测试：
    已在台架上进行模拟数据测试，通过。

变更人：
    李维龙
