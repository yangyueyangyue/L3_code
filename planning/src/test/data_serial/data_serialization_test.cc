//
#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "utils/serialization_utils.h"
#include "data_serialization.h"



#if (ENABLE_GTEST)

namespace phoenix {
namespace data_serial {


static ad_msg::LaneMarkCamera lanemarkcamera[4];
static ad_msg::LaneBoundaryLineCamera laneboundaryline[4];
static ad_msg::LaneCentralLineCamera lanecentralline[3];
static ad_msg::Obstacle obstacle[3];
static ad_msg::ObstacleTrackedInfo obstacleinfolist[3];


TEST(EncodeObstacleRadarListArray, Case_01) {
  static const Int32_t s_data_array_size = 1;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleRadarList)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleRadarList)="
            << sizeof(ad_msg::ObstacleRadarList) << " Bytes" << std::endl;

  ad_msg::ObstacleRadarList data_array;
  ad_msg::ObstacleRadarList decoded_data_array;
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleRadarList& data = data_array;
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.obstacle_num = i+3;
    for (Int32_t j = 0; j < data.obstacle_num; ++j) {
      ad_msg::ObstacleRadar& data_radar = data.obstacles[j];
      data_radar.id = i + 2;
      data_radar.type = ad_msg::ObstacleRadar::OBJ_TYPE_PASSENGER_VEHICLE;
      data_radar.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
      data_radar.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
      data_radar.oncomming = 4;
      data_radar.bridge = 5;
      data_radar.range = 6.1F*(j+1);
      data_radar.angle = 7.1F*(j+1);
      data_radar.range_rate = 8.1F*(j+1);
      data_radar.range_acceleration =9.3F*(j+1);
      data_radar.lateral_rate =10.22F*(j+1);
      data_radar.length = 11.1F*(j+1);
      data_radar.width = 12.02F*(j+1);
      data_radar.x = 13.03F*(j+1);
      data_radar.y = 14.004F*(j+1);
      data_radar.rel_v_x = 15.01F*(j+1);
      data_radar.rel_v_y = 16.02F*(j+1);
      data_radar.accel_x = 17.02F*(j+1);
      data_radar.accel_y = 18.11F*(j+1);
      data_radar.yaw_rate =19.12F*(j+1);
    }
  }
  Int32_t encoded_size = EncodeObstacleRadarListArray(data_buffer, 0, s_buff_size, &data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleRadarListArray(data_buffer, 0, s_buff_size, &decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = data_array;

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "{"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].track_status
                << ", " << data.obstacles[j].merged_status
                << ", " << data.obstacles[j].oncomming
                << ", " << data.obstacles[j].bridge
                << ", " << data.obstacles[j].range
                << ", " << data.obstacles[j].angle
                << ", " << data.obstacles[j].range_rate
                << ", " << data.obstacles[j].range_acceleration
                << ", " << data.obstacles[j].lateral_rate
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate                             
                << "}"              
                << std::endl;  
    }
    std::cout<< "  }"<<std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = decoded_data_array;

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "{"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].track_status
                << ", " << data.obstacles[j].merged_status
                << ", " << data.obstacles[j].oncomming
                << ", " << data.obstacles[j].bridge
                << ", " << data.obstacles[j].range
                << ", " << data.obstacles[j].angle
                << ", " << data.obstacles[j].range_rate
                << ", " << data.obstacles[j].range_acceleration
                << ", " << data.obstacles[j].lateral_rate
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate                             
                << "}"              
                << std::endl;  
  }
      std::cout<< "  }"<<std::endl;
  }
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = data_array;
    const ad_msg::ObstacleRadarList& decoded_data = decoded_data_array;
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::ObstacleRadar& data_obj = data.obstacles[j];
      const ad_msg::ObstacleRadar& dedata_obj = decoded_data.obstacles[j];
      EXPECT_EQ(dedata_obj.id, data_obj.id);
      EXPECT_EQ(dedata_obj.type, data_obj.type);
      EXPECT_EQ(dedata_obj.track_status, data_obj.track_status);
      EXPECT_EQ(dedata_obj.merged_status, data_obj.merged_status);
      EXPECT_EQ(dedata_obj.oncomming, data_obj.oncomming);
      EXPECT_EQ(dedata_obj.bridge, data_obj.bridge);
      EXPECT_EQ(dedata_obj.range, data_obj.range);
      EXPECT_EQ(dedata_obj.angle, data_obj.angle);
      EXPECT_EQ(dedata_obj.range_rate, data_obj.range_rate);
      EXPECT_EQ(dedata_obj.range_acceleration, data_obj.range_acceleration);
      EXPECT_EQ(dedata_obj.lateral_rate, data_obj.lateral_rate);
      EXPECT_EQ(dedata_obj.length, data_obj.length);
      EXPECT_EQ(dedata_obj.width, data_obj.width);
      EXPECT_EQ(dedata_obj.x, data_obj.x);
      EXPECT_EQ(dedata_obj.y, data_obj.y);
      EXPECT_EQ(dedata_obj.rel_v_x, data_obj.rel_v_x);
      EXPECT_EQ(dedata_obj.rel_v_y, data_obj.rel_v_y);
      EXPECT_EQ(dedata_obj.accel_x, data_obj.accel_x); 
      EXPECT_EQ(dedata_obj.accel_y, data_obj.accel_y);
      EXPECT_EQ(dedata_obj.yaw_rate, data_obj.yaw_rate);
    } 
  }
}


TEST(TrajectoryPlanningInfo, Case_01) {
 static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(planning::TrajectoryPlanningInfo)*s_data_array_size;
  std::cout << "sizeof(planning::TrajectoryPlanningInfo)="
            << sizeof(planning::TrajectoryPlanningInfo) << " Bytes" << std::endl;

 static planning::TrajectoryPlanningInfo data_array[s_data_array_size];
 static planning::TrajectoryPlanningInfo decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    planning::TrajectoryPlanningInfo& data = data_array[i];
        data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    
    for (Int32_t j = 0; j <3; ++j) {
      planning::TrajectoryPlanningInfo::RoadGraph::Link link;
      common::PathPoint point;
      point.point.set_x(i+6.4 +j);
      point.point.set_y(i+7.4 +j);
      point.heading = i+8.4+j;
      point.curvature = i +19.2+j;
      point.s = i +10.1+j;
      point.l = i +11.2+j;
      link.sample_points.PushBack(point);
      data.road_graph.road_graph_links.PushBack(link);
    }
  
      common::PathPoint point;
      point.point.set_x(i+16.4 );
      point.point.set_y(i+17.4);
      point.heading = i+18.4;
      point.curvature = i +19.2;
      point.s = i +20.1;
      point.l = i +21.2;

      data.optimal_trajectory_sample_points.PushBack(point);
        
 }
  Int32_t encoded_size = EncodeTrajectoryPlanningInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeTrajectoryPlanningInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningInfo& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.dst_module_id
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.valid;
for (Int32_t j = 0; j < 3; ++j) {
     std::cout << "  {"
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].point.x()
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].point.y()
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].heading
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].curvature
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].s
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].l
              << "}"
              << std::endl;
}
        std::cout << "  {"      
              << ", " << data.optimal_trajectory_sample_points[0].point.x()
              << ", " << data.optimal_trajectory_sample_points[0].point.y()
              << ", " << data.optimal_trajectory_sample_points[0].heading
              << ", " << data.optimal_trajectory_sample_points[0].curvature
              << ", " << data.optimal_trajectory_sample_points[0].s
              << ", " << data.optimal_trajectory_sample_points[0].l                          
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningInfo& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.dst_module_id
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.valid;
for (Int32_t j = 0; j < 3; ++j) {
     std::cout << "  {"
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].point.x()
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].point.y()
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].heading
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].curvature
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].s
              << ", " << data.road_graph.road_graph_links[j].sample_points[0].l
              << "}"
              << std::endl;
}
        std::cout << "  {"      
              << ", " << data.optimal_trajectory_sample_points[0].point.x()
              << ", " << data.optimal_trajectory_sample_points[0].point.y()
              << ", " << data.optimal_trajectory_sample_points[0].heading
              << ", " << data.optimal_trajectory_sample_points[0].curvature
              << ", " << data.optimal_trajectory_sample_points[0].s
              << ", " << data.optimal_trajectory_sample_points[0].l                          
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningInfo& data = data_array[i];
    const planning::TrajectoryPlanningInfo& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);

    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
     
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].point.x(), 
       data.road_graph.road_graph_links[0].sample_points[0].point.x());

    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].point.y(), 
       data.road_graph.road_graph_links[0].sample_points[0].point.y());
    
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].heading, 
       data.road_graph.road_graph_links[0].sample_points[0].heading);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].curvature, 
       data.road_graph.road_graph_links[0].sample_points[0].curvature);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].s, 
       data.road_graph.road_graph_links[0].sample_points[0].s);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[0].sample_points[0].l, 
       data.road_graph.road_graph_links[0].sample_points[0].l);

    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].point.x(), 
       data.road_graph.road_graph_links[1].sample_points[0].point.x());

    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].point.y(), 
       data.road_graph.road_graph_links[1].sample_points[0].point.y());
    
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].heading, 
       data.road_graph.road_graph_links[1].sample_points[0].heading);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].curvature, 
       data.road_graph.road_graph_links[1].sample_points[0].curvature);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].s, 
       data.road_graph.road_graph_links[1].sample_points[0].s);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[1].sample_points[0].l, 
       data.road_graph.road_graph_links[1].sample_points[0].l);

   EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].point.x(), 
       data.road_graph.road_graph_links[2].sample_points[0].point.x());

    EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].point.y(), 
       data.road_graph.road_graph_links[2].sample_points[0].point.y());
    
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].heading, 
       data.road_graph.road_graph_links[2].sample_points[0].heading);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].curvature, 
       data.road_graph.road_graph_links[2].sample_points[0].curvature);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].s, 
       data.road_graph.road_graph_links[2].sample_points[0].s);
    EXPECT_EQ(decoded_data.road_graph.road_graph_links[2].sample_points[0].l, 
       data.road_graph.road_graph_links[2].sample_points[0].l);


     EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].point.x(), 
       data.optimal_trajectory_sample_points[0].point.x());

    EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].point.y(), 
       data.optimal_trajectory_sample_points[0].point.y());
    
    EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].heading, 
       data.optimal_trajectory_sample_points[0].heading);
    EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].curvature, 
       data.optimal_trajectory_sample_points[0].curvature);
    EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].s, 
       data.optimal_trajectory_sample_points[0].s);
    EXPECT_EQ(decoded_data.optimal_trajectory_sample_points[0].l, 
       data.optimal_trajectory_sample_points[0].l);

    
  } 
}

// test msg_chassis
TEST(EncodeChassisArray, Case_01) {
  ad_msg::Chassis chassis;
  ad_msg::Chassis dechassis;
  chassis.msg_head.valid = 1;
  chassis.msg_head.sequence = 1;
  chassis.msg_head.timestamp = 1413;
  chassis.msg_head.src_module_id = 117;
  chassis.msg_head.dst_module_id = 120;
  chassis.driving_mode = 11;
  chassis.e_stop = 12;
  chassis.eps_status = 13;
  chassis.steering_wheel_angle_valid =1;
  chassis.steering_wheel_angle = 14;
  chassis.steering_wheel_speed_valid = 1;
  chassis.steering_wheel_speed = 15;
  chassis.steering_wheel_torque_valid = 1;
  chassis.steering_wheel_torque = 16;
  chassis.v_valid = 1;
  chassis.v = 17;
  chassis.a_valid = 1;
  chassis.a =18;
  chassis.yaw_rate_valid = 1;
  chassis.yaw_rate =19;
  chassis.ax_valid = 1;
  chassis.ax = 20;
  chassis.ay_valid = 1;
  chassis.ay = 21;
  chassis.wheel_speed_fl_valid = 1;
  chassis.wheel_speed_fl = 22;
  chassis.wheel_speed_fr_valid = 1;
  chassis.wheel_speed_fr = 23;
  chassis.wheel_speed_rl_valid = 1;
  chassis.wheel_speed_rl = 24;
  chassis.wheel_speed_rr_valid =1;
  chassis.wheel_speed_rr=25;
  chassis.wheel_speed_rl2_valid = 1;
  chassis.wheel_speed_rl2 = 26;
  chassis.wheel_speed_rr2_valid = 1;
  chassis.wheel_speed_rr2 = 27;
  chassis.epb_status = 1;
  chassis.gear = 28;
  chassis.signal_turning_indicator = 29;
  chassis.signal_turn_lamp = 30;
  chassis.signal_brake_lamp = 31;
  chassis.brake_pedal_value = 32;
  chassis.acc_pedal_value = 33;
  chassis.engine_speed_valid = 1;
  chassis.engine_speed = 34;
  chassis.engine_torque_valid = 1;
  chassis.engine_torque = 35;
  static const Int32_t s_data_array_size = 1;
  static const Int32_t s_buff_size = sizeof(ad_msg::Chassis)*s_data_array_size;

  Uint8_t data_buffer[s_buff_size] = { 0 };
  Int32_t size = EncodeChassisArray(data_buffer,0,s_buff_size,&chassis,1);
  Int32_t decoded_size = DecodeChassisArray(data_buffer, 0, s_buff_size, &dechassis, s_data_array_size);

  EXPECT_EQ(chassis.msg_head.valid, dechassis.msg_head.valid);
  EXPECT_EQ(chassis.msg_head.sequence, dechassis.msg_head.sequence);
  EXPECT_EQ(chassis.msg_head.timestamp, dechassis.msg_head.timestamp);
  EXPECT_EQ(chassis.msg_head.src_module_id, dechassis.msg_head.src_module_id);
  EXPECT_EQ(chassis.msg_head.dst_module_id, dechassis.msg_head.dst_module_id);
  EXPECT_EQ(chassis.driving_mode, dechassis.driving_mode);
  EXPECT_EQ(chassis.e_stop, dechassis.e_stop);
  EXPECT_EQ(chassis.eps_status, dechassis.eps_status);
  EXPECT_EQ(chassis.steering_wheel_angle_valid, dechassis.steering_wheel_angle_valid);
  EXPECT_EQ(chassis.steering_wheel_angle, dechassis.steering_wheel_angle);
  EXPECT_EQ(chassis.steering_wheel_speed_valid, dechassis.steering_wheel_speed_valid);
  EXPECT_EQ(chassis.steering_wheel_speed, dechassis.steering_wheel_speed);
  EXPECT_EQ(chassis.steering_wheel_torque_valid, dechassis.steering_wheel_torque_valid);
  EXPECT_EQ(chassis.steering_wheel_torque, dechassis.steering_wheel_torque);
  EXPECT_EQ(chassis.v_valid, dechassis.v_valid);
  EXPECT_EQ(chassis.v, dechassis.v);
  EXPECT_EQ(chassis.a_valid, dechassis.a_valid);
  EXPECT_EQ(chassis.a, dechassis.a);
  EXPECT_EQ(chassis.yaw_rate_valid, dechassis.yaw_rate_valid);
  EXPECT_EQ(chassis.yaw_rate, dechassis.yaw_rate);
  EXPECT_EQ(chassis.ax_valid, dechassis.ax_valid);
  EXPECT_EQ(chassis.ax, dechassis.ax);
  EXPECT_EQ(chassis.ay_valid, dechassis.ay_valid);
  EXPECT_EQ(chassis.ay, dechassis.ay);
  EXPECT_EQ(chassis.wheel_speed_fl_valid, dechassis.wheel_speed_fl_valid);
  EXPECT_EQ(chassis.wheel_speed_fl, dechassis.wheel_speed_fl);
  EXPECT_EQ(chassis.wheel_speed_fr_valid, dechassis.wheel_speed_fr_valid);
  EXPECT_EQ(chassis.wheel_speed_fr, dechassis.wheel_speed_fr);
  EXPECT_EQ(chassis.wheel_speed_rl_valid, dechassis.wheel_speed_rl_valid);
  EXPECT_EQ(chassis.wheel_speed_rl, dechassis.wheel_speed_rl);
  EXPECT_EQ(chassis.wheel_speed_rr_valid, dechassis.wheel_speed_rr_valid);
  EXPECT_EQ(chassis.wheel_speed_rr, dechassis.wheel_speed_rr);
  EXPECT_EQ(chassis.wheel_speed_rl2_valid, dechassis.wheel_speed_rl2_valid);
  EXPECT_EQ(chassis.wheel_speed_rl2, dechassis.wheel_speed_rl2);
  EXPECT_EQ(chassis.wheel_speed_rr2_valid, dechassis.wheel_speed_rr2_valid);
  EXPECT_EQ(chassis.wheel_speed_rr2, dechassis.wheel_speed_rr2);
  EXPECT_EQ(chassis.epb_status, dechassis.epb_status);
  EXPECT_EQ(chassis.gear, dechassis.gear);
  EXPECT_EQ(chassis.signal_turning_indicator, dechassis.signal_turning_indicator);
  EXPECT_EQ(chassis.signal_turn_lamp, dechassis.signal_turn_lamp);
  EXPECT_EQ(chassis.signal_brake_lamp, dechassis.signal_brake_lamp);
  EXPECT_EQ(chassis.brake_pedal_value, dechassis.brake_pedal_value);
  EXPECT_EQ(chassis.acc_pedal_value, dechassis.acc_pedal_value);
  EXPECT_EQ(chassis.engine_speed_valid, dechassis.engine_speed_valid);
  EXPECT_EQ(chassis.engine_speed, dechassis.engine_speed);
  EXPECT_EQ(chassis.engine_torque_valid, dechassis.engine_torque_valid);
  EXPECT_EQ(chassis.engine_torque, dechassis.engine_torque);

}

TEST(EncodeChassisArray, Case_02) {
  static const Int32_t s_data_array_size = 2;
  ad_msg::Chassis chassis[s_data_array_size];
  ad_msg::Chassis dechassis[s_data_array_size];

  for (Int8_t i = 0; i < s_data_array_size; ++i) {

  chassis[i].msg_head.valid = 1;
  chassis[i].msg_head.sequence = 1;
  chassis[i].msg_head.timestamp = 1413 + i;
  chassis[i].msg_head.src_module_id = 117;
  chassis[i].msg_head.dst_module_id = 120;
  chassis[i].driving_mode = 11+i;
  chassis[i].e_stop = 12+i*2;
  chassis[i].eps_status = 13;
  chassis[i].steering_wheel_angle_valid =1;
  chassis[i].steering_wheel_angle = 14.0+0.5*i;
  chassis[i].steering_wheel_speed_valid = 1;
  chassis[i].steering_wheel_speed = 15+0.1*i;
  chassis[i].steering_wheel_torque_valid = 1;
  chassis[i].steering_wheel_torque = 16+0.88*i;
  chassis[i].v_valid = 1;
  chassis[i].v = 17.0+0.2*i*i;
  chassis[i].a_valid = 1;
  chassis[i].a =18+0.975*i;
  chassis[i].yaw_rate_valid = 1;
  chassis[i].yaw_rate =19+0.3*i;
  chassis[i].ax_valid = 1;
  chassis[i].ax = 20+0.4*i;
  chassis[i].ay_valid = 1;
  chassis[i].ay = 21+0.555*i;
  chassis[i].wheel_speed_fl_valid = 1;
  chassis[i].wheel_speed_fl = 22+0.1*i;
  chassis[i].wheel_speed_fr_valid = 1;
  chassis[i].wheel_speed_fr = 23+0.55*i;
  chassis[i].wheel_speed_rl_valid = 1;
  chassis[i].wheel_speed_rl = 24+0.55*i*i;
  chassis[i].wheel_speed_rr_valid =1;
  chassis[i].wheel_speed_rr=25+0.6*i;
  chassis[i].wheel_speed_rl2_valid = 1;
  chassis[i].wheel_speed_rl2 = 26+0.5*i;
  chassis[i].wheel_speed_rr2_valid = 1;
  chassis[i].wheel_speed_rr2 = 27+0.6*i;
  chassis[i].epb_status = 1;
  chassis[i].gear = 28;
  chassis[i].signal_turning_indicator = 29;
  chassis[i].signal_turn_lamp = 30;
  chassis[i].signal_brake_lamp = 31;
  chassis[i].brake_pedal_value = 32;
  chassis[i].acc_pedal_value = 33;
  chassis[i].engine_speed_valid = 1;
  chassis[i].engine_speed = 34;
  chassis[i].engine_torque_valid = 1;
  chassis[i].engine_torque = 35;
  }
    std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Chassis& data = chassis[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.driving_mode
              << ", " << data.e_stop
              << ", " << data.eps_status
              << ", " << data.steering_wheel_angle_valid
              << ", " << data.steering_wheel_angle
              << ", " << data.steering_wheel_speed_valid
              << ", " << data.steering_wheel_speed
              << ", " << data.steering_wheel_torque_valid
              << ", " << data.steering_wheel_torque
              << ", " << data.v_valid
              << ", " << data.v
              << ", " << data.a_valid
              << ", " << data.a
              << ", " << data.yaw_rate
              << ", " << data.yaw_rate_valid
              << ", " << data.ax_valid
              << ", " << data.ax
              << ", " << data.ay_valid
              << ", " << data.ay
              << ", " << data.wheel_speed_fl_valid
              << ", " << data.wheel_speed_fl
              << ", " << data.wheel_speed_fr_valid
              << ", " << data.wheel_speed_fr
              << ", " << data.wheel_speed_rl_valid
              << ", " << data.wheel_speed_rl
              << ", " << data.wheel_speed_rr_valid
              << ", " << data.wheel_speed_rr
              << ", " << data.wheel_speed_rl2_valid
              << ", " << data.wheel_speed_rl2
              << ", " << data.wheel_speed_rr2_valid
              << ", " << data.wheel_speed_rr2
              << ", " << data.epb_status
              << ", " << data.gear
              << ", " << data.signal_turning_indicator
              << ", " << data.signal_turn_lamp
              << ", " << data.signal_brake_lamp
              << ", " << data.brake_pedal_value
              << ", " << data.acc_pedal_value
              << ", " << data.engine_speed_valid
              << ", " << data.engine_speed
              << ", " << data.engine_torque_valid
              << ", " << data.engine_torque
              << "}"
              << std::endl;
  }

  
  static const Int32_t s_buff_size = sizeof(ad_msg::Chassis)*s_data_array_size;

  Uint8_t data_buffer[s_buff_size] = { 0 };
  Int32_t size = EncodeChassisArray(data_buffer,0,s_buff_size,&chassis[0],s_data_array_size);
  Int32_t decoded_size = DecodeChassisArray(data_buffer, 0, s_buff_size, &dechassis[0], s_data_array_size);
  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Chassis& data = dechassis[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.driving_mode
              << ", " << data.e_stop
              << ", " << data.eps_status
              << ", " << data.steering_wheel_angle_valid
              << ", " << data.steering_wheel_angle
              << ", " << data.steering_wheel_speed_valid
              << ", " << data.steering_wheel_speed
              << ", " << data.steering_wheel_torque_valid
              << ", " << data.steering_wheel_torque
              << ", " << data.v_valid
              << ", " << data.v
              << ", " << data.a_valid
              << ", " << data.a
              << ", " << data.yaw_rate
              << ", " << data.yaw_rate_valid
              << ", " << data.ax_valid
              << ", " << data.ax
              << ", " << data.ay_valid
              << ", " << data.ay
              << ", " << data.wheel_speed_fl_valid
              << ", " << data.wheel_speed_fl
              << ", " << data.wheel_speed_fr_valid
              << ", " << data.wheel_speed_fr
              << ", " << data.wheel_speed_rl_valid
              << ", " << data.wheel_speed_rl
              << ", " << data.wheel_speed_rr_valid
              << ", " << data.wheel_speed_rr
              << ", " << data.wheel_speed_rl2_valid
              << ", " << data.wheel_speed_rl2
              << ", " << data.wheel_speed_rr2_valid
              << ", " << data.wheel_speed_rr2
              << ", " << data.epb_status
              << ", " << data.gear
              << ", " << data.signal_turning_indicator
              << ", " << data.signal_turn_lamp
              << ", " << data.signal_brake_lamp
              << ", " << data.brake_pedal_value
              << ", " << data.acc_pedal_value
              << ", " << data.engine_speed_valid
              << ", " << data.engine_speed
              << ", " << data.engine_torque_valid
              << ", " << data.engine_torque
              << "}"
              << std::endl;
  }

  for (Int8_t j = 0; j < s_data_array_size; ++j) {
  EXPECT_EQ(chassis[j].msg_head.valid, dechassis[j].msg_head.valid);
  EXPECT_EQ(chassis[j].msg_head.sequence, dechassis[j].msg_head.sequence);
  EXPECT_EQ(chassis[j].msg_head.timestamp, dechassis[j].msg_head.timestamp);
  EXPECT_EQ(chassis[j].msg_head.src_module_id, dechassis[j].msg_head.src_module_id);
  EXPECT_EQ(chassis[j].msg_head.dst_module_id, dechassis[j].msg_head.dst_module_id);
  EXPECT_EQ(chassis[j].driving_mode, dechassis[j].driving_mode);
  EXPECT_EQ(chassis[j].e_stop, dechassis[j].e_stop);
  EXPECT_EQ(chassis[j].eps_status, dechassis[j].eps_status);
  EXPECT_EQ(chassis[j].steering_wheel_angle_valid, dechassis[j].steering_wheel_angle_valid);
  EXPECT_EQ(chassis[j].steering_wheel_angle, dechassis[j].steering_wheel_angle);
  EXPECT_EQ(chassis[j].steering_wheel_speed_valid, dechassis[j].steering_wheel_speed_valid);
  EXPECT_EQ(chassis[j].steering_wheel_speed, dechassis[j].steering_wheel_speed);
  EXPECT_EQ(chassis[j].steering_wheel_torque_valid, dechassis[j].steering_wheel_torque_valid);
  EXPECT_EQ(chassis[j].steering_wheel_torque, dechassis[j].steering_wheel_torque);
  EXPECT_EQ(chassis[j].v_valid, dechassis[j].v_valid);
  EXPECT_EQ(chassis[j].v, dechassis[j].v);
  EXPECT_EQ(chassis[j].a_valid, dechassis[j].a_valid);
  EXPECT_EQ(chassis[j].a, dechassis[j].a);
  EXPECT_EQ(chassis[j].yaw_rate_valid, dechassis[j].yaw_rate_valid);
  EXPECT_EQ(chassis[j].yaw_rate, dechassis[j].yaw_rate);
  EXPECT_EQ(chassis[j].ax_valid, dechassis[j].ax_valid);
  EXPECT_EQ(chassis[j].ax, dechassis[j].ax);
  EXPECT_EQ(chassis[j].ay_valid, dechassis[j].ay_valid);
  EXPECT_EQ(chassis[j].ay, dechassis[j].ay);
  EXPECT_EQ(chassis[j].wheel_speed_fl_valid, dechassis[j].wheel_speed_fl_valid);
  EXPECT_EQ(chassis[j].wheel_speed_fl, dechassis[j].wheel_speed_fl);
  EXPECT_EQ(chassis[j].wheel_speed_fr_valid, dechassis[j].wheel_speed_fr_valid);
  EXPECT_EQ(chassis[j].wheel_speed_fr, dechassis[j].wheel_speed_fr);
  EXPECT_EQ(chassis[j].wheel_speed_rl_valid, dechassis[j].wheel_speed_rl_valid);
  EXPECT_EQ(chassis[j].wheel_speed_rl, dechassis[j].wheel_speed_rl);
  EXPECT_EQ(chassis[j].wheel_speed_rr_valid, dechassis[j].wheel_speed_rr_valid);
  EXPECT_EQ(chassis[j].wheel_speed_rr, dechassis[j].wheel_speed_rr);
  EXPECT_EQ(chassis[j].wheel_speed_rl2_valid, dechassis[j].wheel_speed_rl2_valid);
  EXPECT_EQ(chassis[j].wheel_speed_rl2, dechassis[j].wheel_speed_rl2);
  EXPECT_EQ(chassis[j].wheel_speed_rr2_valid, dechassis[j].wheel_speed_rr2_valid);
  EXPECT_EQ(chassis[j].wheel_speed_rr2, dechassis[j].wheel_speed_rr2);
  EXPECT_EQ(chassis[j].epb_status, dechassis[j].epb_status);
  EXPECT_EQ(chassis[j].gear, dechassis[j].gear);
  EXPECT_EQ(chassis[j].signal_turning_indicator, dechassis[j].signal_turning_indicator);
  EXPECT_EQ(chassis[j].signal_turn_lamp, dechassis[j].signal_turn_lamp);
  EXPECT_EQ(chassis[j].signal_brake_lamp, dechassis[j].signal_brake_lamp);
  EXPECT_EQ(chassis[j].brake_pedal_value, dechassis[j].brake_pedal_value);
  EXPECT_EQ(chassis[j].acc_pedal_value, dechassis[j].acc_pedal_value);
  EXPECT_EQ(chassis[j].engine_speed_valid, dechassis[j].engine_speed_valid);
  EXPECT_EQ(chassis[j].engine_speed, dechassis[j].engine_speed);
  EXPECT_EQ(chassis[j].engine_torque_valid, dechassis[j].engine_torque_valid);
  EXPECT_EQ(chassis[j].engine_torque, dechassis[j].engine_torque);
 }
}

// test lanemark
TEST(EncodeLaneMarkCamera, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneMarkCamera) * s_data_array_size;
  ad_msg::LaneMarkCamera data_array[s_data_array_size];
  ad_msg::LaneMarkCamera decodedata_array[s_data_array_size];
    common::com_memset(lanemarkcamera, 0,
                     sizeof(lanemarkcamera));
  for (Int8_t i = 0; i < s_data_array_size; ++i) {
    data_array[i].id = i+1;
    data_array[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    data_array[i].quality = i;
    data_array[i].view_range_valid = true;
    data_array[i].mark_width = i * 0.5F;
    data_array[i].view_range_start = i  * 0.8F;
    data_array[i].view_range_end = i * 0.9F;
    data_array[i].c0 = i+1.0F;
    data_array[i].c1 = i + 1.0F;
    data_array[i].c2 = i + 3.0F;
    data_array[i].c3 = i + 4.0F ;
  }
  common::com_memcpy(lanemarkcamera,data_array,sizeof(data_array));
  Uint8_t data_buffer[s_buff_size] = { 0 };
  Int32_t size = EncodeLaneMarkCameraArray (data_buffer,0,s_buff_size,data_array,s_data_array_size);
  Int32_t desize = DecodeLaneMarkCameraArray (data_buffer,0,s_buff_size,decodedata_array,s_data_array_size);

   
    EXPECT_EQ(data_array[0].id, decodedata_array[0].id);
    EXPECT_EQ(data_array[0].lane_mark_type, decodedata_array[0].lane_mark_type);
    EXPECT_EQ(data_array[0].quality, decodedata_array[0].quality);
    EXPECT_EQ(data_array[0].view_range_valid, decodedata_array[0].view_range_valid);
    EXPECT_EQ(data_array[0].mark_width, decodedata_array[0].mark_width);
    EXPECT_EQ(data_array[0].view_range_start, decodedata_array[0].view_range_start);
    EXPECT_EQ(data_array[0].view_range_end, decodedata_array[0].view_range_end);
    EXPECT_EQ(data_array[0].c0, decodedata_array[0].c0);
    EXPECT_EQ(data_array[0].c1, decodedata_array[0].c1);
    EXPECT_EQ(data_array[0].c2, decodedata_array[0].c2);
    EXPECT_EQ(data_array[0].c3, decodedata_array[0].c3);
    
    EXPECT_EQ(data_array[1].id, decodedata_array[1].id);
    EXPECT_EQ(data_array[1].lane_mark_type, decodedata_array[1].lane_mark_type);
    EXPECT_EQ(data_array[1].quality, decodedata_array[1].quality);
    EXPECT_EQ(data_array[1].view_range_valid, decodedata_array[1].view_range_valid);
    EXPECT_EQ(data_array[1].mark_width, decodedata_array[1].mark_width);
    EXPECT_EQ(data_array[1].view_range_start, decodedata_array[1].view_range_start);
    EXPECT_EQ(data_array[1].view_range_end, decodedata_array[1].view_range_end);
    EXPECT_EQ(data_array[1].c0, decodedata_array[1].c0);
    EXPECT_EQ(data_array[1].c1, decodedata_array[1].c1);
    EXPECT_EQ(data_array[1].c2, decodedata_array[1].c2);
    EXPECT_EQ(data_array[1].c3, decodedata_array[1].c3);
    
    EXPECT_EQ(data_array[2].id, decodedata_array[2].id);
    EXPECT_EQ(data_array[2].lane_mark_type, decodedata_array[2].lane_mark_type);
    EXPECT_EQ(data_array[2].quality, decodedata_array[2].quality);
    EXPECT_EQ(data_array[2].view_range_valid, decodedata_array[2].view_range_valid);
    EXPECT_EQ(data_array[2].mark_width, decodedata_array[2].mark_width);
    EXPECT_EQ(data_array[2].view_range_start, decodedata_array[2].view_range_start);
    EXPECT_EQ(data_array[2].view_range_end, decodedata_array[2].view_range_end);
    EXPECT_EQ(data_array[2].c0, decodedata_array[2].c0);
    EXPECT_EQ(data_array[2].c1, decodedata_array[2].c1);
    EXPECT_EQ(data_array[2].c2, decodedata_array[2].c2);
    EXPECT_EQ(data_array[2].c3, decodedata_array[2].c3);

}

TEST(EncodeLaneMarkCameraListArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneMarkCameraList)*s_data_array_size;

  std::cout << "sizeof(ad_msg::LaneMarkCameraList)="
            << sizeof(ad_msg::LaneMarkCameraList) << " Bytes" << std::endl;

  ad_msg::LaneMarkCameraList data_array[s_data_array_size];
  ad_msg::LaneMarkCameraList decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::LaneMarkCameraList& data = data_array[i];

    data.msg_head.valid  = 1;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;
    data.lane_mark_num = 3;
    data.lane_marks[i] = lanemarkcamera[i];
  }
    

  Int32_t encoded_size = EncodeLaneMarkCameraListArray(data_buffer, 0, s_buff_size, &data_array[0], s_data_array_size);
  Int32_t decoded_size = DecodeLaneMarkCameraListArray(data_buffer, 0, s_buff_size, &decoded_data_array[0], s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneMarkCameraList& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.lane_mark_num
              << ", " << data.lane_marks[i].id
              << ", " << data.lane_marks[i].quality
              << ", " << data.lane_marks[i].view_range_valid
              << ", " << data.lane_marks[i].mark_width
              << ", " << data.lane_marks[i].view_range_start
              << ", " << data.lane_marks[i].view_range_end
              << ", " << data.lane_marks[i].c0
              << ", " << data.lane_marks[i].c1
              << ", " << data.lane_marks[i].c2
              << ", " << data.lane_marks[i].c3
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneMarkCameraList& data = decoded_data_array[i];


    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.lane_mark_num
              << ", " << data.lane_marks[i].id
              << ", " << data.lane_marks[i].quality
              << ", " << data.lane_marks[i].view_range_valid
              << ", " << data.lane_marks[i].mark_width
              << ", " << data.lane_marks[i].view_range_start
              << ", " << data.lane_marks[i].view_range_end
              << ", " << data.lane_marks[i].c0
              << ", " << data.lane_marks[i].c1
              << ", " << data.lane_marks[i].c2
              << ", " << data.lane_marks[i].c3
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneMarkCameraList& data = data_array[i];
    const ad_msg::LaneMarkCameraList& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int8_t j = 0; j < 3; ++j) {
     const ad_msg::LaneMarkCamera& lane_data = data.lane_marks[j]; 
     const ad_msg::LaneMarkCamera& delane_data = decoded_data.lane_marks[j]; 

     EXPECT_EQ(lane_data.id, delane_data.id);
     EXPECT_EQ(lane_data.lane_mark_type, delane_data.lane_mark_type);
     EXPECT_EQ(lane_data.quality, delane_data.quality);
     EXPECT_EQ(lane_data.view_range_valid, delane_data.view_range_valid);
     EXPECT_EQ(lane_data.mark_width, delane_data.mark_width);
     EXPECT_EQ(lane_data.view_range_start, delane_data.view_range_start);
     EXPECT_EQ(lane_data.view_range_end, delane_data.view_range_end);
     EXPECT_EQ(lane_data.c0, delane_data.c0);
     EXPECT_EQ(lane_data.c1, delane_data.c1);
     EXPECT_EQ(lane_data.c2, delane_data.c2);
     EXPECT_EQ(lane_data.c3, delane_data.c3);
    }
    EXPECT_EQ(decoded_data.lane_mark_num, data.lane_mark_num);
  }
  }

  


TEST(EncodeCurvePointArray,Case_01) {
 static const Int32_t s_data_array_size = 10;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneBoundaryLineCamera::CurvePoint)*s_data_array_size;

  std::cout << "sizeof(ad_msg::LaneBoundaryLineCamera::CurvePoint)="
            << sizeof(ad_msg::LaneBoundaryLineCamera::CurvePoint) << " Bytes" << std::endl;

  ad_msg::LaneBoundaryLineCamera::CurvePoint data_array[s_data_array_size];
  ad_msg::LaneBoundaryLineCamera::CurvePoint decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::LaneBoundaryLineCamera::CurvePoint& data = data_array[i];

    data.x = i *0.00845F;
    data.y = i * i * 0.8F + 1;
  }


  Int32_t encoded_size = EncodeCurvePointArray(data_buffer, 0, s_buff_size, &data_array[0], s_data_array_size);
  Int32_t decoded_size = DecodeCurvePointArray(data_buffer, 0, s_buff_size, &decoded_data_array[0], s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneBoundaryLineCamera::CurvePoint& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneBoundaryLineCamera::CurvePoint& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneBoundaryLineCamera::CurvePoint& data = data_array[i];
    const ad_msg::LaneBoundaryLineCamera::CurvePoint& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
  }  
}
TEST(EncodeLaneBoundaryLineCameraArray,Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneBoundaryLineCamera)*s_data_array_size;

  std::cout << "sizeof(ad_msg::LaneBoundaryLineCamera)="
            << sizeof(ad_msg::LaneBoundaryLineCamera) << " Bytes" << std::endl;

  ad_msg::LaneBoundaryLineCamera data_array[s_data_array_size];
  ad_msg::LaneBoundaryLineCamera decoded_data_array[s_data_array_size];
  common::com_memset(laneboundaryline, 0,
                     sizeof(laneboundaryline)); 
  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::LaneBoundaryLineCamera& data = data_array[i];
    data.id = i ;
    data.type  = i;
    data.curve_point_num = i*5;
    for ( Int32_t j = 0; j <  data.curve_point_num; ++j) {
      data.curve[j].x = i*1.0;
      data.curve[j].y = i*2.0;
    }
  }
 
  common::com_memcpy(laneboundaryline,data_array,sizeof(data_array));
  Int32_t encoded_size = EncodeLaneBoundaryLineCameraArray(data_buffer, 0, s_buff_size, &data_array[0], s_data_array_size);
  Int32_t decoded_size = DecodeLaneBoundaryLineCameraArray(data_buffer, 0, s_buff_size, &decoded_data_array[0], s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneBoundaryLineCamera& data = data_array[i];
    const ad_msg::LaneBoundaryLineCamera& decoded_data = decoded_data_array[i];
    
    EXPECT_EQ(encoded_size, decoded_size);
    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.type, data.type);
    EXPECT_EQ(decoded_data.curve_point_num, data.curve_point_num);
    for (Int32_t j = 0; j <  data.curve_point_num; ++j) {
      const ad_msg::LaneBoundaryLineCamera::CurvePoint& point = data_array[i].curve[j];
      const ad_msg::LaneBoundaryLineCamera::CurvePoint& depoint = decoded_data_array[i].curve[j];
      EXPECT_EQ(point.x, depoint.x);
      EXPECT_EQ(point.y, depoint.y);
    }
  }  
}

TEST(EncodeLaneCentralLineCameraArray,Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneCentralLineCamera)*s_data_array_size;

  std::cout << "sizeof(ad_msg::LaneCentralLineCamera)="
            << sizeof(ad_msg::LaneCentralLineCamera) << " Bytes" << std::endl;

  ad_msg::LaneCentralLineCamera data_array[s_data_array_size];
  ad_msg::LaneCentralLineCamera decoded_data_array[s_data_array_size];
  common::com_memset(lanecentralline, 0,
                     sizeof(lanecentralline)); 
  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::LaneCentralLineCamera& data = data_array[i];
    data.quality = i ;
    data.forward_len  = i+1.5F;
    data.id = i;
    data.left_boundary_index = 60-i;
    data.right_boundary_index = 66660-i;
    data.left_central_line_index = 60-i;
    data.right_central_line_index = 60-i;
    data.curve_point_num =499; 
    for ( Int32_t j = 0; j <  data.curve_point_num; ++j) {
      data.curve[j].x = i*1.0;
      data.curve[j].y = i*2.0;
      data.curve[j].left_width = i*0.3;
      data.curve[j].right_width = i;
    }
  }
  common::com_memcpy(lanecentralline,data_array,sizeof(data_array));

  Int32_t encoded_size = EncodeLaneCentralLineCameraArray(data_buffer, 0, s_buff_size, &data_array[0], s_data_array_size);
  Int32_t decoded_size = DecodeLaneCentralLineCameraArray(data_buffer, 0, s_buff_size, &decoded_data_array[0], s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneCentralLineCamera& data = data_array[i];
    const ad_msg::LaneCentralLineCamera& decoded_data = decoded_data_array[i];
    
    EXPECT_EQ(encoded_size, decoded_size);
    EXPECT_EQ(decoded_data.quality, data.quality);
    EXPECT_EQ(decoded_data.forward_len, data.forward_len);
    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.left_boundary_index, data.left_boundary_index);
    EXPECT_EQ(decoded_data.right_boundary_index, data.right_boundary_index);
    EXPECT_EQ(decoded_data.left_central_line_index, data.left_central_line_index);
    EXPECT_EQ(decoded_data.right_central_line_index, data.right_central_line_index);
    EXPECT_EQ(decoded_data.curve_point_num, data.curve_point_num);
    
    for (Int32_t j = 0; j <  data.curve_point_num; ++j) {
      const ad_msg::LaneCentralLineCamera::CurvePoint& point = data_array[i].curve[j];
      const ad_msg::LaneCentralLineCamera::CurvePoint& depoint = decoded_data_array[i].curve[j];
      EXPECT_EQ(point.x, depoint.x);
      EXPECT_EQ(point.y, depoint.y);
      EXPECT_EQ(point.left_width, depoint.left_width);
      EXPECT_EQ(point.right_width, depoint.right_width);
    }
  }  
}

TEST(EncodeLaneInfoCameraListArray,Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::LaneInfoCameraList)*s_data_array_size;

  std::cout << "sizeof(ad_msg::LaneInfoCameraList)="
            << sizeof(ad_msg::LaneInfoCameraList) << " Bytes" << std::endl;

  ad_msg::LaneInfoCameraList data_array[s_data_array_size];
  ad_msg::LaneInfoCameraList decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::LaneInfoCameraList& data = data_array[i];
    data.msg_head.sequence = i ;
    data.msg_head.src_module_id = 100+i;
    data.msg_head.dst_module_id = 200+i;
    data.msg_head.timestamp = 100;
    data.msg_head.valid = i;
    data.quality = i;
    data.forward_len  = i+1.5F;
    data.lane_line_num = i;
    data.central_line_num = i;
    for (Int16_t j = 0; j < data.lane_line_num; ++j) {
    data.boundary_lines[j] = laneboundaryline[j];
    data.central_lines[j] = lanecentralline[j];
    }
  }


  Int32_t encoded_size = EncodeLaneInfoCameraListArray(data_buffer, 0, s_buff_size, &data_array[0], s_data_array_size);
  Int32_t decoded_size = DecodeLaneInfoCameraListArray(data_buffer, 0, s_buff_size, &decoded_data_array[0], s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::LaneInfoCameraList& data = data_array[i];
    const ad_msg::LaneInfoCameraList& decoded_data = decoded_data_array[i];
    
    EXPECT_EQ(encoded_size, decoded_size);
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    EXPECT_EQ(decoded_data.quality, data.quality);
    EXPECT_EQ(decoded_data.forward_len, data.forward_len);
    EXPECT_EQ(decoded_data.lane_line_num, data.lane_line_num);
    for (Int16_t j = 0; j < data.lane_line_num; ++j) {
      EXPECT_EQ(decoded_data.boundary_lines[j].id, data.boundary_lines[j].id);
      EXPECT_EQ(decoded_data.boundary_lines[j].curve_point_num, data.boundary_lines[j].curve_point_num);
      for (Int32_t k = 0; k < data.boundary_lines[j].curve_point_num; ++k) {
        EXPECT_EQ(decoded_data.boundary_lines[j].curve[k].x,data.boundary_lines[j].curve[k].x);
        EXPECT_EQ(decoded_data.boundary_lines[j].curve[k].y,data.boundary_lines[j].curve[k].y); 
      }
    }
    EXPECT_EQ(decoded_data.central_line_num, data.central_line_num);
    for (Int16_t m = 0; m < data.central_line_num; ++m) {
      EXPECT_EQ(decoded_data.central_lines[m].id, data.central_lines[m].id);
      EXPECT_EQ(decoded_data.central_lines[m].quality, data.central_lines[m].quality);
      EXPECT_EQ(decoded_data.central_lines[m].forward_len, data.central_lines[m].forward_len);
      EXPECT_EQ(decoded_data.central_lines[m].left_boundary_index, data.central_lines[m].left_boundary_index);
      EXPECT_EQ(decoded_data.central_lines[m].right_boundary_index, data.central_lines[m].right_boundary_index);
      EXPECT_EQ(decoded_data.central_lines[m].left_central_line_index, data.central_lines[m].left_central_line_index);
      EXPECT_EQ(decoded_data.central_lines[m].right_central_line_index, data.central_lines[m].right_central_line_index);
      EXPECT_EQ(decoded_data.central_lines[m].curve_point_num, data.central_lines[m].curve_point_num);
    
    for (Int32_t n = 0; n < data.central_lines[m].curve_point_num; ++n) {
  
        EXPECT_EQ(decoded_data.central_lines[m].curve[n].x,data.central_lines[m].curve[n].x);
        EXPECT_EQ(decoded_data.central_lines[m].curve[n].y,data.central_lines[m].curve[n].y); 
        EXPECT_EQ(decoded_data.central_lines[m].curve[n].left_width,data.central_lines[m].curve[n].left_width);
        EXPECT_EQ(decoded_data.central_lines[m].curve[n].right_width,data.central_lines[m].curve[n].right_width);        
      }
    } 
    
  }  
}
TEST(EncodeGnssArray,Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::Gnss)*s_data_array_size;

  std::cout << "sizeof(ad_msg::Gnss)="
            << sizeof(ad_msg::Gnss) << " Bytes" << std::endl;

  ad_msg::Gnss data_array[s_data_array_size];
  ad_msg::Gnss decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::Gnss& data = data_array[i];
    data.msg_head.dst_module_id = 100;
    data.msg_head.sequence = 1000 + i;
    data.msg_head.src_module_id = 200 + i;
    data.msg_head.timestamp = 100 + i;
    data.msg_head.valid = 1;
    data.latitude = i - 64.0F;
    data.longitude = i + .5;
    data.altitude = (i+1) * 11.4F;
    data.heading_gnss = i * 0.4;;
    data.x_utm = i + 5.0F;
    data.y_utm = i + 7.0F;
    data.z_utm = i*3.0F + 7.0F;
    data.heading_utm = i*5.0F;
    data.x_odom = i*0.333;
    data.y_odom = i*0.55;
    data.z_odom = i*0.66;
    data.heading_odom = i*0.77F;
    data.roll = i*0.2F;
    data.v_e = i*0.880F;
    data.v_n = i*0.45F;
    data.v_u = i*0.12F;
    data.v_x_utm = i*1.2F + 7.0F;
    data.v_y_utm = i*3.6F + 7.0F;
    data.v_z_utm = i*1.32F;
    data.v_x_odom = i*5.6F;
    data.v_y_odom = i+0.2F;
    data.v_z_odom = i*1.66F;
    data.gnss_status = i;
    data.utm_status = i+1;
    data.odom_status = i+3;
  }


  Int32_t encoded_size = EncodeGnssArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeGnssArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Gnss& data = data_array[i];
    const ad_msg::Gnss& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    EXPECT_EQ(decoded_data.latitude,data.latitude);
    EXPECT_EQ(decoded_data.longitude,data.longitude);
    EXPECT_EQ(decoded_data.altitude,data.altitude);
    EXPECT_EQ(decoded_data.heading_gnss,data.heading_gnss);
    EXPECT_EQ(decoded_data.x_utm,data.x_utm);
    EXPECT_EQ(decoded_data.y_utm,data.y_utm);
    EXPECT_EQ(decoded_data.z_utm,data.z_utm);
    EXPECT_EQ(decoded_data.heading_utm,data.heading_utm);
    EXPECT_EQ(decoded_data.x_odom,data.x_odom);
    EXPECT_EQ(decoded_data.y_odom,data.y_odom);
    EXPECT_EQ(decoded_data.z_odom,data.z_odom);
    EXPECT_EQ(decoded_data.heading_odom,data.heading_odom);
    EXPECT_EQ(decoded_data.pitch,data.pitch);
    EXPECT_EQ(decoded_data.roll,data.roll);
    EXPECT_EQ(decoded_data.v_e,data.v_e);
    EXPECT_EQ(decoded_data.v_n,data.v_n);
    EXPECT_EQ(decoded_data.v_u,data.v_u);
    EXPECT_EQ(decoded_data.v_x_utm,data.v_x_utm);
    EXPECT_EQ(decoded_data.v_y_utm,data.v_y_utm);
    EXPECT_EQ(decoded_data.v_z_utm,data.v_z_utm);
    EXPECT_EQ(decoded_data.v_x_odom,data.v_x_odom);
    EXPECT_EQ(decoded_data.v_y_odom,data.v_y_odom);
    EXPECT_EQ(decoded_data.v_z_odom,data.v_z_odom);
    EXPECT_EQ(decoded_data.gnss_status,data.gnss_status);
    EXPECT_EQ(decoded_data.utm_status,data.utm_status);
    EXPECT_EQ(decoded_data.odom_status,data.odom_status);
       
  }  
}
TEST(EncodeImuArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::Imu)*s_data_array_size;

  std::cout << "sizeof(ad_msg::Imu)="
            << sizeof(ad_msg::Imu) << " Bytes" << std::endl;

  ad_msg::Imu data_array[s_data_array_size];
  ad_msg::Imu decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::Imu& data = data_array[i];

    data.msg_head.valid = i % 2;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;
    data.yaw_rate = i * 0.5F;
    data.pitch_rate = i + 0.8F;
    data.roll_rate = i*0.2F;
    data.accel_x = 0.1234F+i;
    data.accel_y = 0.154F+i;
    data.accel_z = 0.44F+i;
  }


  Int32_t encoded_size = EncodeImuArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeImuArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Imu& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.yaw_rate
              << ", " << data.pitch_rate
              << ", " << data.roll_rate
              << ", " << data.accel_x
              << ", " << data.accel_y
              << ", " << data.accel_z
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Imu& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

     std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.yaw_rate
              << ", " << data.pitch_rate
              << ", " << data.roll_rate
              << ", " << data.accel_x
              << ", " << data.accel_y
              << ", " << data.accel_z
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Imu& data = data_array[i];
    const ad_msg::Imu& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    EXPECT_EQ(decoded_data.yaw_rate, data.yaw_rate);
    EXPECT_EQ(decoded_data.pitch_rate, data.pitch_rate); 
    EXPECT_EQ(decoded_data.roll_rate, data.roll_rate); 
    EXPECT_EQ(decoded_data.accel_x, data.accel_x); 
    EXPECT_EQ(decoded_data.accel_y, data.accel_y); 
    EXPECT_EQ(decoded_data.accel_z, data.accel_z); 
  
  }
}
TEST(EncodeRelativePosArray, Case_01) {
  static const Int32_t s_data_array_size = 22;
  static const Int32_t s_buff_size = sizeof(ad_msg::MsgHead)*s_data_array_size;

  std::cout << "sizeof(ad_msg::RelativePos)="
            << sizeof(ad_msg::RelativePos) << " Bytes" << std::endl;

  ad_msg::RelativePos data_array[s_data_array_size];
  ad_msg::RelativePos decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::RelativePos& data = data_array[i];

    data.relative_time =4 + i * 2;
    data.x = i+1.0F;
    data.y = (i+1)*0.5F;
    data.heading = i*0.66F;
    data.yaw_rate = i*1.35F;
    data.v = i+1.35F;
  }


  Int32_t encoded_size = EncodeRelativePosArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeRelativePosArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::RelativePos& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << ", " << data.heading
              << ", " << data.y
              << ", " << data.relative_time
              << ", " << data.yaw_rate
              << ", " << data.v
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::RelativePos& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << ", " << data.heading
              << ", " << data.y
              << ", " << data.relative_time
              << ", " << data.yaw_rate
              << ", " << data.v
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::RelativePos& data = data_array[i];
    const ad_msg::RelativePos& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
    EXPECT_EQ(decoded_data.relative_time, data.relative_time);
    EXPECT_EQ(decoded_data.heading, data.heading);
    EXPECT_EQ(decoded_data.yaw_rate, data.yaw_rate);
    EXPECT_EQ(decoded_data.v, data.v);
  } 
  
  const Int8_t list_data_array_size = 3;
  const Int32_t list_buff_size = sizeof(ad_msg::RelativePosList)*list_data_array_size;
  ad_msg::RelativePosList list_data_array[list_data_array_size];
  ad_msg::RelativePosList list_decoded_data_array[list_data_array_size];

  Uint8_t data_buffer_list[list_buff_size] = { 0 }; 
  for (Int32_t j = 0; j < list_data_array_size; ++j) {
    ad_msg::RelativePosList& data = list_data_array[j];
    data.msg_head.timestamp = 1 + j;
    data.msg_head.sequence = 2 + j;
    data.msg_head.src_module_id = 3 + j;
    data.msg_head.dst_module_id = 4 + j;
    data.msg_head.valid = 1;
    data.relative_pos_num = j;
    for (Int32_t k = 0; k < data.relative_pos_num; ++k) {
      data.relative_pos[k].heading = data_array[k].heading;
      data.relative_pos[k].relative_time = data_array[k].relative_time;
      data.relative_pos[k].v = data_array[k].v;
      data.relative_pos[k].x = data_array[k].x;
      data.relative_pos[k].y = data_array[k].y;
      data.relative_pos[k].yaw_rate = data_array[k].yaw_rate;
    }
  } 

  encoded_size = EncodeRelativePosListArray(data_buffer_list, 0, list_buff_size, list_data_array, list_data_array_size);
  decoded_size = DecodeRelativePosListArray(data_buffer_list, 0, list_buff_size, list_decoded_data_array, list_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < list_data_array_size; ++i) {
    const ad_msg::RelativePosList& data = list_data_array[i];
    const ad_msg::RelativePosList& decoded_data = list_decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    EXPECT_EQ(decoded_data.relative_pos_num,data.relative_pos_num);
    for (Int32_t j = 0; j < data.relative_pos_num; ++j) {
      EXPECT_EQ(decoded_data.relative_pos[j].x, data.relative_pos[j].x);
      EXPECT_EQ(decoded_data.relative_pos[j].y, data.relative_pos[j].y);
      EXPECT_EQ(decoded_data.relative_pos[j].relative_time, data.relative_pos[j].relative_time);
      EXPECT_EQ(decoded_data.relative_pos[j].heading, data.relative_pos[j].heading);
      EXPECT_EQ(decoded_data.relative_pos[j].yaw_rate, data.relative_pos[j].yaw_rate);
      EXPECT_EQ(decoded_data.relative_pos[j].v, data.relative_pos[j].v);
    }
  } 
}

TEST(EncodeOBBoxArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::MsgHead)*s_data_array_size;
  ad_msg::OBBox data_array[s_data_array_size];
  ad_msg::OBBox decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::OBBox& data = data_array[i];

    data.x = i * 2.F;
    data.y = i+1.6F;
    data.heading = (i+1)*1.5F;
    data.half_width = i+2.1F;
    data.half_length = i+3.555F;
  }


  Int32_t encoded_size = EncodeOBBoxArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeOBBoxArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::OBBox& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << ", " << data.heading
              << ", " << data.half_width
              << ", " << data.half_length
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::OBBox& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.x
              << ", " << data.y
              << ", " << data.heading
              << ", " << data.half_width
              << ", " << data.half_length
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::OBBox& data = data_array[i];
    const ad_msg::OBBox& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
    EXPECT_EQ(decoded_data.heading, data.heading);
    EXPECT_EQ(decoded_data.half_width, data.half_width);
    EXPECT_EQ(decoded_data.half_length, data.half_length);
  }  
}
TEST(EncodeObstacleCameraArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleCamera)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleCamera)="
            << sizeof(ad_msg::ObstacleCamera) << " Bytes" << std::endl;

  ad_msg::ObstacleCamera data_array[s_data_array_size];
  ad_msg::ObstacleCamera decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleCamera& data = data_array[i];

    data.id = i + 2;
    data.type = ad_msg::ObstacleCamera::OBJ_TYPE_SPECIAL_VEHICLE;
    data.status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
    data.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE;
    data.blinker = ad_msg::ObstacleCamera::BLINKER_OFF;
    data.brake_lights = i;
    data.age = i+5;
    data.lane = i+4;
    data.length = i+3.0F;
    data.width = i+2.0F;
    data.x = i*1.1F;
    data.y = i*2.2F;
    data.rel_v_x = i*4.4F;
    data.rel_v_y = i*8.8F;
    data.accel_x = i+4.4F;
    data.accel_y = i+5.4F;
    data.yaw_rate = i+6.4F;
    data.scale_change = i*7.8F;
  }
  Int32_t encoded_size = EncodeObstacleCameraArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleCameraArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCamera& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.type
              << ", " << data.status
              << ", " << data.cut_in
              << ", " << data.blinker
              << ", " << data.brake_lights
              << ", " << data.age
              << ", " << data.lane
              << ", " << data.length
              << ", " << data.width
              << ", " << data.x
              << ", " << data.y
              << ", " << data.rel_v_x
              << ", " << data.rel_v_y
              << ", " << data.accel_x  
              << ", " << data.accel_y
              << ", " << data.yaw_rate
              << ", " << data.scale_change                            
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCamera& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.type
              << ", " << data.status
              << ", " << data.cut_in
              << ", " << data.blinker
              << ", " << data.brake_lights
              << ", " << data.age
              << ", " << data.lane
              << ", " << data.length
              << ", " << data.width
              << ", " << data.x
              << ", " << data.y
              << ", " << data.rel_v_x
              << ", " << data.rel_v_y
              << ", " << data.accel_x  
              << ", " << data.accel_y
              << ", " << data.yaw_rate
              << ", " << data.scale_change                            
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCamera& data = data_array[i];
    const ad_msg::ObstacleCamera& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.type, data.type);
    EXPECT_EQ(decoded_data.status, data.status);
    EXPECT_EQ(decoded_data.cut_in, data.cut_in);
    EXPECT_EQ(decoded_data.blinker, data.blinker);
    EXPECT_EQ(decoded_data.brake_lights, data.brake_lights);
    EXPECT_EQ(decoded_data.age, data.age);
    EXPECT_EQ(decoded_data.lane, data.lane);
    EXPECT_EQ(decoded_data.length, data.length);
    EXPECT_EQ(decoded_data.width, data.width);
    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
    EXPECT_EQ(decoded_data.rel_v_x, data.rel_v_x);
    EXPECT_EQ(decoded_data.rel_v_y, data.rel_v_y);
    EXPECT_EQ(decoded_data.accel_x, data.accel_x); 
    EXPECT_EQ(decoded_data.accel_y, data.accel_y);
    EXPECT_EQ(decoded_data.yaw_rate, data.yaw_rate);
    EXPECT_EQ(decoded_data.scale_change, data.scale_change);     
  }
}
TEST(EncodeObstacleCameraListArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleCameraList)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleCameraList)="
            << sizeof(ad_msg::ObstacleCameraList) << " Bytes" << std::endl;

  ad_msg::ObstacleCameraList data_array[s_data_array_size];
  ad_msg::ObstacleCameraList decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleCameraList& data = data_array[i];
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.obstacle_num = i+3;
    for (Int32_t j = 0; j < data.obstacle_num; ++j) {
      ad_msg::ObstacleCamera& data_camera = data.obstacles[j];
      data_camera.id = j + 2;
      data_camera.type = ad_msg::ObstacleCamera::OBJ_TYPE_SPECIAL_VEHICLE;
      data_camera.status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
      data_camera.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE;
      data_camera.blinker = ad_msg::ObstacleCamera::BLINKER_OFF;
      data_camera.brake_lights = j+1;
      data_camera.age = j+5;
      data_camera.lane = j+4;
      data_camera.length = j+3.1F;
      data_camera.width = j+2.1F;
      data_camera.x = j+1.1F;
      data_camera.y = j+2.2F;
      data_camera.rel_v_x = j+4.4F;
      data_camera.rel_v_y = j+8.8F;
      data_camera.accel_x = j+4.4F;
      data_camera.accel_y = j+5.4F;
      data_camera.yaw_rate = j+6.4F;
      data_camera.scale_change = j+7.8F;
    }
  }
  Int32_t encoded_size = EncodeObstacleCameraListArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleCameraListArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCameraList& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "{"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].status
                << ", " << data.obstacles[j].cut_in
                << ", " << data.obstacles[j].blinker
                << ", " << data.obstacles[j].brake_lights
                << ", " << data.obstacles[j].age
                << ", " << data.obstacles[j].lane
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate
                << ", " << data.obstacles[j].scale_change                              
                << "}"
                
                << std::endl;  
    }
    std::cout<< "  }"<<std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCameraList& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "  {"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].status
                << ", " << data.obstacles[j].cut_in
                << ", " << data.obstacles[j].blinker
                << ", " << data.obstacles[j].brake_lights
                << ", " << data.obstacles[j].age
                << ", " << data.obstacles[j].lane
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate
                << ", " << data.obstacles[j].scale_change                              
                << "}"
                << std::endl;  
  }
      std::cout<< "  }"<<std::endl;
  }
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleCameraList& data = data_array[i];
    const ad_msg::ObstacleCameraList& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::ObstacleCamera& data_obj = data.obstacles[j];
      const ad_msg::ObstacleCamera& dedata_obj = decoded_data.obstacles[j];
      EXPECT_EQ(dedata_obj.id, data_obj.id);
      EXPECT_EQ(dedata_obj.type, data_obj.type);
      EXPECT_EQ(dedata_obj.status, data_obj.status);
      EXPECT_EQ(dedata_obj.cut_in, data_obj.cut_in);
      EXPECT_EQ(dedata_obj.blinker, data_obj.blinker);
      EXPECT_EQ(dedata_obj.brake_lights, data_obj.brake_lights);
      EXPECT_EQ(dedata_obj.age, data_obj.age);
      EXPECT_EQ(dedata_obj.lane, data_obj.lane);
      EXPECT_EQ(dedata_obj.length, data_obj.length);
      EXPECT_EQ(dedata_obj.width, data_obj.width);
      EXPECT_EQ(dedata_obj.x, data_obj.x);
      EXPECT_EQ(dedata_obj.y, data_obj.y);
      EXPECT_EQ(dedata_obj.rel_v_x, data_obj.rel_v_x);
      EXPECT_EQ(dedata_obj.rel_v_y, data_obj.rel_v_y);
      EXPECT_EQ(dedata_obj.accel_x, data_obj.accel_x); 
      EXPECT_EQ(dedata_obj.accel_y, data_obj.accel_y);
      EXPECT_EQ(dedata_obj.yaw_rate, data_obj.yaw_rate);
      EXPECT_EQ(dedata_obj.scale_change, data_obj.scale_change);     
    } 
  }
}

TEST(EncodeObstacleRadarArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleRadar)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleRadar)="
            << sizeof(ad_msg::ObstacleRadar) << " Bytes" << std::endl;

  ad_msg::ObstacleRadar data_array[s_data_array_size];
  ad_msg::ObstacleRadar decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleRadar& data = data_array[i];
    data.id = i + 2;
    data.type = ad_msg::ObstacleRadar::OBJ_TYPE_PASSENGER_VEHICLE;
    data.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
    data.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
    data.oncomming = 4;
    data.bridge = 5;
    data.range = 6.1F*(i+1);
    data.angle = 7.1F*(i+1);
    data.range_rate = 8.1F*(i+1);
    data.range_acceleration =9.3F*(i+1);
    data.lateral_rate =10.22F*(i+1);
    data.length = 11.1F*(i+1);
    data.width = 12.02F*(i+1);
    data.x = 13.03F*(i+1);
    data.y = 14.004F*(i+1);
    data.rel_v_x = 15.01F*(i+1);
    data.rel_v_y = 16.02F*(i+1);
    data.accel_x = 17.02F*(i+1);
    data.accel_y = 18.11F*(i+1);
    data.yaw_rate =19.12F*(i+1);
  }
  Int32_t encoded_size = EncodeObstacleRadarArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleRadarArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadar& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.type
              << ", " << data.track_status
              << ", " << data.merged_status
              << ", " << data.oncomming
              << ", " << data.bridge
              << ", " << data.range
              << ", " << data.angle
              << ", " << data.range_rate
              << ", " << data.range_acceleration
              << ", " << data.lateral_rate
              << ", " << data.length
              << ", " << data.width
              << ", " << data.x
              << ", " << data.y
              << ", " << data.rel_v_x
              << ", " << data.rel_v_y
              << ", " << data.accel_x  
              << ", " << data.accel_y
              << ", " << data.yaw_rate                       
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadar& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.type
              << ", " << data.track_status
              << ", " << data.merged_status
              << ", " << data.oncomming
              << ", " << data.bridge
              << ", " << data.range
              << ", " << data.angle
              << ", " << data.range_rate
              << ", " << data.range_acceleration
              << ", " << data.lateral_rate
              << ", " << data.length
              << ", " << data.width
              << ", " << data.x
              << ", " << data.y
              << ", " << data.rel_v_x
              << ", " << data.rel_v_y
              << ", " << data.accel_x  
              << ", " << data.accel_y
              << ", " << data.yaw_rate                       
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadar& data = data_array[i];
    const ad_msg::ObstacleRadar& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.type, data.type);
    EXPECT_EQ(decoded_data.track_status, data.track_status);
    EXPECT_EQ(decoded_data.merged_status, data.merged_status);
    EXPECT_EQ(decoded_data.oncomming, data.oncomming);
    EXPECT_EQ(decoded_data.bridge, data.bridge);
    EXPECT_EQ(decoded_data.range, data.range);
    EXPECT_EQ(decoded_data.angle, data.angle);
    EXPECT_EQ(decoded_data.range_rate, data.range_rate);
    EXPECT_EQ(decoded_data.range_acceleration, data.range_acceleration);
    EXPECT_EQ(decoded_data.lateral_rate, data.lateral_rate);
    EXPECT_EQ(decoded_data.length, data.length);
    EXPECT_EQ(decoded_data.width, data.width);
    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
    EXPECT_EQ(decoded_data.rel_v_x, data.rel_v_x);
    EXPECT_EQ(decoded_data.rel_v_y, data.rel_v_y);
    EXPECT_EQ(decoded_data.accel_x, data.accel_x); 
    EXPECT_EQ(decoded_data.accel_y, data.accel_y);
    EXPECT_EQ(decoded_data.yaw_rate, data.yaw_rate);
  }
}
TEST(EncodeObstacleRadarListArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleRadarList)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleRadarList)="
            << sizeof(ad_msg::ObstacleRadarList) << " Bytes" << std::endl;

  ad_msg::ObstacleRadarList data_array[s_data_array_size];
  ad_msg::ObstacleRadarList decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleRadarList& data = data_array[i];
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.obstacle_num = i+3;
    for (Int32_t j = 0; j < data.obstacle_num; ++j) {
      ad_msg::ObstacleRadar& data_radar = data.obstacles[j];
      data_radar.id = i + 2;
      data_radar.type = ad_msg::ObstacleRadar::OBJ_TYPE_PASSENGER_VEHICLE;
      data_radar.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
      data_radar.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
      data_radar.oncomming = 4;
      data_radar.bridge = 5;
      data_radar.range = 6.1F*(j+1);
      data_radar.angle = 7.1F*(j+1);
      data_radar.range_rate = 8.1F*(j+1);
      data_radar.range_acceleration =9.3F*(j+1);
      data_radar.lateral_rate =10.22F*(j+1);
      data_radar.length = 11.1F*(j+1);
      data_radar.width = 12.02F*(j+1);
      data_radar.x = 13.03F*(j+1);
      data_radar.y = 14.004F*(j+1);
      data_radar.rel_v_x = 15.01F*(j+1);
      data_radar.rel_v_y = 16.02F*(j+1);
      data_radar.accel_x = 17.02F*(j+1);
      data_radar.accel_y = 18.11F*(j+1);
      data_radar.yaw_rate =19.12F*(j+1);
    }
  }
  Int32_t encoded_size = EncodeObstacleRadarListArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleRadarListArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "{"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].track_status
                << ", " << data.obstacles[j].merged_status
                << ", " << data.obstacles[j].oncomming
                << ", " << data.obstacles[j].bridge
                << ", " << data.obstacles[j].range
                << ", " << data.obstacles[j].angle
                << ", " << data.obstacles[j].range_rate
                << ", " << data.obstacles[j].range_acceleration
                << ", " << data.obstacles[j].lateral_rate
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate                             
                << "}"              
                << std::endl;  
    }
    std::cout<< "  }"<<std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      std::cout << "{"
                << ", " << data.obstacles[j].id
                << ", " << data.obstacles[j].type
                << ", " << data.obstacles[j].track_status
                << ", " << data.obstacles[j].merged_status
                << ", " << data.obstacles[j].oncomming
                << ", " << data.obstacles[j].bridge
                << ", " << data.obstacles[j].range
                << ", " << data.obstacles[j].angle
                << ", " << data.obstacles[j].range_rate
                << ", " << data.obstacles[j].range_acceleration
                << ", " << data.obstacles[j].lateral_rate
                << ", " << data.obstacles[j].length
                << ", " << data.obstacles[j].width
                << ", " << data.obstacles[j].x
                << ", " << data.obstacles[j].y
                << ", " << data.obstacles[j].rel_v_x
                << ", " << data.obstacles[j].rel_v_y
                << ", " << data.obstacles[j].accel_x  
                << ", " << data.obstacles[j].accel_y
                << ", " << data.obstacles[j].yaw_rate                             
                << "}"              
                << std::endl;  
  }
      std::cout<< "  }"<<std::endl;
  }
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleRadarList& data = data_array[i];
    const ad_msg::ObstacleRadarList& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::ObstacleRadar& data_obj = data.obstacles[j];
      const ad_msg::ObstacleRadar& dedata_obj = decoded_data.obstacles[j];
      EXPECT_EQ(dedata_obj.id, data_obj.id);
      EXPECT_EQ(dedata_obj.type, data_obj.type);
      EXPECT_EQ(dedata_obj.track_status, data_obj.track_status);
      EXPECT_EQ(dedata_obj.merged_status, data_obj.merged_status);
      EXPECT_EQ(dedata_obj.oncomming, data_obj.oncomming);
      EXPECT_EQ(dedata_obj.bridge, data_obj.bridge);
      EXPECT_EQ(dedata_obj.range, data_obj.range);
      EXPECT_EQ(dedata_obj.angle, data_obj.angle);
      EXPECT_EQ(dedata_obj.range_rate, data_obj.range_rate);
      EXPECT_EQ(dedata_obj.range_acceleration, data_obj.range_acceleration);
      EXPECT_EQ(dedata_obj.lateral_rate, data_obj.lateral_rate);
      EXPECT_EQ(dedata_obj.length, data_obj.length);
      EXPECT_EQ(dedata_obj.width, data_obj.width);
      EXPECT_EQ(dedata_obj.x, data_obj.x);
      EXPECT_EQ(dedata_obj.y, data_obj.y);
      EXPECT_EQ(dedata_obj.rel_v_x, data_obj.rel_v_x);
      EXPECT_EQ(dedata_obj.rel_v_y, data_obj.rel_v_y);
      EXPECT_EQ(dedata_obj.accel_x, data_obj.accel_x); 
      EXPECT_EQ(dedata_obj.accel_y, data_obj.accel_y);
      EXPECT_EQ(dedata_obj.yaw_rate, data_obj.yaw_rate);
    } 
  }
}

TEST(EncodeObstacleArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::Obstacle)*s_data_array_size;
  std::cout << "sizeof(ad_msg::Obstacle)="
            << sizeof(ad_msg::Obstacle) << " Bytes" << std::endl;
  
  ad_msg::Obstacle data_array[s_data_array_size];
  ad_msg::Obstacle decoded_data_array[s_data_array_size];
  common::com_memset(obstacle, 0,
                     sizeof(obstacle)); 
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::Obstacle& data = data_array[i];

    data.id = i + 2;
    data.obb.x = i+1.1F;
    data.obb.y = i+2.1F;
    data.obb.heading = i+3.1F;
    data.obb.half_width = i+4.1F;
    data.obb.half_length = i+5.1F;
    data.height = i+6.1F;
    data.height_to_ground = i+7.1F;
    data.type = 1;
    data.dynamic = i+1;
    data.existence = i+12220;
    data.v_x = i+11.1F;
    data.v_y = i+12.2F;
    data.v = i+13.4F;
    data.a_x = i+15.8F;
    data.a_y = i+16.4F;
    data.a = i+17.4F;
    data.pred_path_num = 2;
    data.pred_path_point_num[0] = 1;
    data.pred_path_point_num[1] = 2;
    data.pred_path[0][0].x = 11.1+i;
    data.pred_path[0][0].y = 12.2+i;
    data.pred_path[0][0].heading = 13.3+i;
    data.pred_path[0][0].s = 14.4+i;
    data.pred_path[1][0].x = 21.1+i;
    data.pred_path[1][0].y = 22.2+i;
    data.pred_path[1][0].heading = 23.3+i;
    data.pred_path[1][0].s = 24.44+i;
    data.pred_path[1][1].x = 31.666+i;
    data.pred_path[1][1].y = 32.777+i;
    data.pred_path[1][1].heading = 33.555+i;
    data.pred_path[1][1].s = 34.666+i; 
    data.tracked_path_point_num =2;
    data.tracked_path[0].x = 1.1+i;
    data.tracked_path[0].y = 1.2+i;
    data.tracked_path[1].x = 1.1+i;
    data.tracked_path[1].y = 1.2+i;
  }
  
  common::com_memcpy(obstacle,data_array,sizeof(data_array));
  Int32_t encoded_size = EncodeObstacleArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Obstacle& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.obb.x
              << ", " << data.obb.y
              << ", " << data.obb.heading
              << ", " << data.obb.half_width
              << ", " << data.obb.half_length 
              << ", " << data.height
              << ", " << data.height_to_ground
              << ", " << data.type
              << ", " << data.dynamic
              << ", " << data.existence
              << ", " << data.perception_type
              << ", " << data.v_x
              << ", " << data.v_y
              << ", " << data.v
              << ", " << data.a_x
              << ", " << data.a_y
              << ", " << data.a
              << ", " << data.pred_path_num  
              << ", " << data.pred_path_point_num[0]
              << ", " << data.pred_path_point_num[1]
              << ", " << data.pred_path[0][0].x
              << ", " << data.pred_path[0][0].y 
              << ", " << data.pred_path[0][0].heading 
              << ", " << data.pred_path[0][0].s 
              << ", " << data.pred_path[1][0].x 
              << ", " << data.pred_path[1][0].y 
              << ", " << data.pred_path[1][0].heading 
              << ", " << data.pred_path[1][0].s                                           
              << ", " << data.pred_path[1][1].x 
              << ", " << data.pred_path[1][1].y 
              << ", " << data.pred_path[1][1].heading 
              << ", " << data.pred_path[1][1].s  
              << ", " << data.tracked_path_point_num
              << ", " << data.tracked_path[0].x
              << ", " << data.tracked_path[0].y
              << ", " << data.tracked_path[1].x
              << ", " << data.tracked_path[1].y
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Obstacle& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.obb.x
              << ", " << data.obb.y
              << ", " << data.obb.heading
              << ", " << data.obb.half_width
              << ", " << data.obb.half_length 
              << ", " << data.height
              << ", " << data.height_to_ground
              << ", " << data.type
              << ", " << data.dynamic
              << ", " << data.existence
              << ", " << data.perception_type
              << ", " << data.v_x
              << ", " << data.v_y
              << ", " << data.v
              << ", " << data.a_x
              << ", " << data.a_y
              << ", " << data.a
              << ", " << data.pred_path_num  
              << ", " << data.pred_path_point_num[0]
              << ", " << data.pred_path_point_num[1]
              << ", " << data.pred_path[0][0].x
              << ", " << data.pred_path[0][0].y 
              << ", " << data.pred_path[0][0].heading 
              << ", " << data.pred_path[0][0].s 
              << ", " << data.pred_path[1][0].x 
              << ", " << data.pred_path[1][0].y 
              << ", " << data.pred_path[1][0].heading 
              << ", " << data.pred_path[1][0].s                                           
              << ", " << data.pred_path[1][1].x 
              << ", " << data.pred_path[1][1].y 
              << ", " << data.pred_path[1][1].heading 
              << ", " << data.pred_path[1][1].s  
              << ", " << data.tracked_path_point_num
              << ", " << data.tracked_path[0].x
              << ", " << data.tracked_path[0].y
              << ", " << data.tracked_path[1].x
              << ", " << data.tracked_path[1].y
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::Obstacle& data = data_array[i];
    const ad_msg::Obstacle& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.obb.x, data.obb.x);
    EXPECT_EQ(decoded_data.obb.y, data.obb.y);
    EXPECT_EQ(decoded_data.obb.half_length, data.obb.half_length);
    EXPECT_EQ(decoded_data.obb.half_width, data.obb.half_width);
    EXPECT_EQ(decoded_data.obb.heading, data.obb.heading);
    EXPECT_EQ(decoded_data.height, data.height);
    EXPECT_EQ(decoded_data.height_to_ground, data.height_to_ground);
    EXPECT_EQ(decoded_data.type, data.type);
    EXPECT_EQ(decoded_data.dynamic, data.dynamic);
    EXPECT_EQ(decoded_data.existence, data.existence);
    EXPECT_EQ(decoded_data.perception_type, data.perception_type);
    EXPECT_EQ(decoded_data.v_x, data.v_x);
    EXPECT_EQ(decoded_data.v_y, data.v_y);
    EXPECT_EQ(decoded_data.v, data.v);
    EXPECT_EQ(decoded_data.a_x, data.a_x);
    EXPECT_EQ(decoded_data.a_y, data.a_y);
    EXPECT_EQ(decoded_data.a, data.a);
    EXPECT_EQ(decoded_data.pred_path_num, data.pred_path_num); 
    EXPECT_EQ(decoded_data.pred_path_point_num[0], data.pred_path_point_num[0]);
    EXPECT_EQ(decoded_data.pred_path_point_num[1], 
             data.pred_path_point_num[1]);
    EXPECT_EQ(decoded_data.pred_path[0][0].x, data.pred_path[0][0].x);
    EXPECT_EQ(decoded_data.pred_path[0][0].y, data.pred_path[0][0].y);             
    EXPECT_EQ(decoded_data.pred_path[0][0].s, data.pred_path[0][0].s);
    EXPECT_EQ(decoded_data.pred_path[0][0].heading, data.pred_path[0][0].heading);
    EXPECT_EQ(decoded_data.pred_path[1][0].x, data.pred_path[1][0].x);
    EXPECT_EQ(decoded_data.pred_path[1][0].y, data.pred_path[1][0].y);             
    EXPECT_EQ(decoded_data.pred_path[1][0].s, data.pred_path[1][0].s);
    EXPECT_EQ(decoded_data.pred_path[1][0].heading, data.pred_path[1][0].heading);
    EXPECT_EQ(decoded_data.pred_path[1][1].x, data.pred_path[1][1].x);
    EXPECT_EQ(decoded_data.pred_path[1][1].y, data.pred_path[1][1].y);             
    EXPECT_EQ(decoded_data.pred_path[1][1].s, data.pred_path[1][1].s);
    EXPECT_EQ(decoded_data.pred_path[1][1].heading, data.pred_path[1][1].heading);     
    EXPECT_EQ(decoded_data.tracked_path_point_num, data.tracked_path_point_num);
    EXPECT_EQ(decoded_data.tracked_path[0].x, data.tracked_path[0].x);
    EXPECT_EQ(decoded_data.tracked_path[0].y, data.tracked_path[0].y);
    EXPECT_EQ(decoded_data.tracked_path[1].x, data.tracked_path[1].x);
    EXPECT_EQ(decoded_data.tracked_path[1].y, data.tracked_path[1].y);    
  }
}

TEST(EncodeObstacleListArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleList)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleList)="
            << sizeof(ad_msg::ObstacleList) << " Bytes" << std::endl;

  ad_msg::ObstacleList data_array[s_data_array_size];
  ad_msg::ObstacleList decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleList& data = data_array[i];
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.obstacle_num = 3;                                                  
    common::com_memcpy(data.obstacles,obstacle,sizeof(obstacle)); 
  }
  Int32_t encoded_size = EncodeObstacleListArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleListArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleList& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::Obstacle& data_obj = data.obstacles[j];
      std::cout << "{"
              << ", " << data_obj.id
              << ", " << data_obj.obb.x
              << ", " << data_obj.obb.y
              << ", " << data_obj.obb.heading
              << ", " << data_obj.obb.half_width
              << ", " << data_obj.obb.half_length 
              << ", " << data_obj.height
              << ", " << data_obj.height_to_ground
              << ", " << data_obj.type
              << ", " << data_obj.dynamic
              << ", " << data_obj.existence
              << ", " << data_obj.perception_type
              << ", " << data_obj.v_x
              << ", " << data_obj.v_y
              << ", " << data_obj.v
              << ", " << data_obj.a_x
              << ", " << data_obj.a_y
              << ", " << data_obj.a
              << ", " << data_obj.pred_path_num  
              << ", " << data_obj.pred_path_point_num[0]
              << ", " << data_obj.pred_path_point_num[1]
              << ", " << data_obj.pred_path[0][0].x
              << ", " << data_obj.pred_path[0][0].y 
              << ", " << data_obj.pred_path[0][0].heading 
              << ", " << data_obj.pred_path[0][0].s 
              << ", " << data_obj.pred_path[1][0].x 
              << ", " << data_obj.pred_path[1][0].y 
              << ", " << data_obj.pred_path[1][0].heading 
              << ", " << data_obj.pred_path[1][0].s                                           
              << ", " << data_obj.pred_path[1][1].x 
              << ", " << data_obj.pred_path[1][1].y 
              << ", " << data_obj.pred_path[1][1].heading 
              << ", " << data_obj.pred_path[1][1].s  
              << ", " << data_obj.tracked_path_point_num
              << ", " << data_obj.tracked_path[0].x
              << ", " << data_obj.tracked_path[0].y
              << ", " << data_obj.tracked_path[1].x
              << ", " << data_obj.tracked_path[1].y
              << "}"
              << std::endl;                
    }
    std::cout<< "  }"<<std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleList& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.obstacle_num
              <<std::endl;
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::Obstacle& data_obj = data.obstacles[j];
      std::cout << "  {"
              << ", " << data_obj.id
              << ", " << data_obj.obb.x
              << ", " << data_obj.obb.y
              << ", " << data_obj.obb.heading
              << ", " << data_obj.obb.half_width
              << ", " << data_obj.obb.half_length 
              << ", " << data_obj.height
              << ", " << data_obj.height_to_ground
              << ", " << (Int32_t)data_obj.type
              << ", " << data_obj.dynamic
              << ", " << data_obj.existence
              << ", " << data_obj.perception_type
              << ", " << data_obj.v_x
              << ", " << data_obj.v_y
              << ", " << data_obj.v
              << ", " << data_obj.a_x
              << ", " << data_obj.a_y
              << ", " << data_obj.a
              << ", " << data_obj.pred_path_num  
              << ", " << data_obj.pred_path_point_num[0]
              << ", " << data_obj.pred_path_point_num[1]
              << ", " << data_obj.pred_path[0][0].x
              << ", " << data_obj.pred_path[0][0].y 
              << ", " << data_obj.pred_path[0][0].heading 
              << ", " << data_obj.pred_path[0][0].s 
              << ", " << data_obj.pred_path[1][0].x 
              << ", " << data_obj.pred_path[1][0].y 
              << ", " << data_obj.pred_path[1][0].heading 
              << ", " << data_obj.pred_path[1][0].s                                           
              << ", " << data_obj.pred_path[1][1].x 
              << ", " << data_obj.pred_path[1][1].y 
              << ", " << data_obj.pred_path[1][1].heading 
              << ", " << data_obj.pred_path[1][1].s  
              << ", " << data_obj.tracked_path_point_num
              << ", " << data_obj.tracked_path[0].x
              << ", " << data_obj.tracked_path[0].y
              << ", " << data_obj.tracked_path[1].x
              << ", " << data_obj.tracked_path[1].y                           
              << "}"
              << std::endl;  
  }
      std::cout<< "  }"<<std::endl;
  }
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleList& data = data_array[i];
    const ad_msg::ObstacleList& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::Obstacle& data_obj = data.obstacles[j];
      const ad_msg::Obstacle& dedata_obj = decoded_data.obstacles[j];
    EXPECT_EQ(dedata_obj.id, data_obj.id);
    EXPECT_EQ(dedata_obj.obb.x, data_obj.obb.x);
    EXPECT_EQ(dedata_obj.obb.y, data_obj.obb.y);
    EXPECT_EQ(dedata_obj.obb.half_length, data_obj.obb.half_length);
    EXPECT_EQ(dedata_obj.obb.half_width, data_obj.obb.half_width);
    EXPECT_EQ(dedata_obj.obb.heading, data_obj.obb.heading);
    EXPECT_EQ(dedata_obj.height, data_obj.height);
    EXPECT_EQ(dedata_obj.height_to_ground, data_obj.height_to_ground);
    EXPECT_EQ(dedata_obj.type, data_obj.type);
    EXPECT_EQ(dedata_obj.dynamic, data_obj.dynamic);
    EXPECT_EQ(dedata_obj.existence, data_obj.existence);
    EXPECT_EQ(dedata_obj.perception_type, data_obj.perception_type);
    EXPECT_EQ(dedata_obj.v_x, data_obj.v_x);
    EXPECT_EQ(dedata_obj.v_y, data_obj.v_y);
    EXPECT_EQ(dedata_obj.v, data_obj.v);
    EXPECT_EQ(dedata_obj.a_x, data_obj.a_x);
    EXPECT_EQ(dedata_obj.a_y, data_obj.a_y);
    EXPECT_EQ(dedata_obj.a, data_obj.a);
    EXPECT_EQ(dedata_obj.pred_path_num, data_obj.pred_path_num); 
    EXPECT_EQ(dedata_obj.pred_path_point_num[0], data_obj.pred_path_point_num[0]);
    EXPECT_EQ(dedata_obj.pred_path_point_num[1], 
             data_obj.pred_path_point_num[1]);
    EXPECT_EQ(dedata_obj.pred_path[0][0].x, data_obj.pred_path[0][0].x);
    EXPECT_EQ(dedata_obj.pred_path[0][0].y, data_obj.pred_path[0][0].y);             
    EXPECT_EQ(dedata_obj.pred_path[0][0].s, data_obj.pred_path[0][0].s);
    EXPECT_EQ(dedata_obj.pred_path[0][0].heading, data_obj.pred_path[0][0].heading);
    EXPECT_EQ(dedata_obj.pred_path[1][0].x, data_obj.pred_path[1][0].x);
    EXPECT_EQ(dedata_obj.pred_path[1][0].y, data_obj.pred_path[1][0].y);             
    EXPECT_EQ(dedata_obj.pred_path[1][0].s, data_obj.pred_path[1][0].s);
    EXPECT_EQ(dedata_obj.pred_path[1][0].heading, data_obj.pred_path[1][0].heading);
    EXPECT_EQ(dedata_obj.pred_path[1][1].x, data_obj.pred_path[1][1].x);
    EXPECT_EQ(dedata_obj.pred_path[1][1].y, data_obj.pred_path[1][1].y);             
    EXPECT_EQ(dedata_obj.pred_path[1][1].s, data_obj.pred_path[1][1].s);
    EXPECT_EQ(dedata_obj.pred_path[1][1].heading, data_obj.pred_path[1][1].heading);     
    EXPECT_EQ(dedata_obj.tracked_path_point_num, data_obj.tracked_path_point_num);
    EXPECT_EQ(dedata_obj.tracked_path[0].x, data_obj.tracked_path[0].x);
    EXPECT_EQ(dedata_obj.tracked_path[0].y, data_obj.tracked_path[0].y);
    EXPECT_EQ(dedata_obj.tracked_path[1].x, data_obj.tracked_path[1].x);
    EXPECT_EQ(dedata_obj.tracked_path[1].y, data_obj.tracked_path[1].y);       
    } 
  }
}

TEST(EncodePlanningResultArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::PlanningResult)*s_data_array_size;
  std::cout << "sizeof(ad_msg::PlanningResult)="
            << sizeof(ad_msg::PlanningResult) << " Bytes" << std::endl;

  ad_msg::PlanningResult data_array[s_data_array_size];
  ad_msg::PlanningResult decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::PlanningResult& data = data_array[i];
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.cur_status = 3; 
    data.cur_status = i+4;
    data.tar_driving_mode = i+5;
    data.tar_gear = i+6; 
    data.tar_turn_lamp = i+7;
    data.tar_brake_lamp = i+8; 
    data.tar_v = i+9;
    data.tar_a = i+10;        
    data.tar_trj.timestamp = i+11;
    data.tar_trj.curr_pos.x=i+12;
    data.tar_trj.curr_pos.y=i+13;
    data.tar_trj.curr_pos.h=i+14;
    data.tar_trj.curr_pos.c=i+15;
    data.tar_trj.curr_pos.s=i+16;
    data.tar_trj.curr_pos.l=i+17;
    data.tar_trj.leading_pos.x=i+18;
    data.tar_trj.leading_pos.y=i+19;
    data.tar_trj.leading_pos.h=i+20;
    data.tar_trj.leading_pos.c=i+21;
    data.tar_trj.leading_pos.s=i+22;
    data.tar_trj.leading_pos.l=i+23;
    data.tar_trj.trj_direction=i+4;
    data.tar_trj.points_num=1;
    data.tar_trj.points[0].x=i+24;
    data.tar_trj.points[0].y=i+25;
    data.tar_trj.points[0].h=i+26;
    data.tar_trj.points[0].c=i+27;
    data.tar_trj.points[0].s=i+28;
  }
  
  Int32_t encoded_size = EncodePlanningResultArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodePlanningResultArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);
  
  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::PlanningResult& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.cur_status
              << ", " << (Int32_t)data.tar_driving_mode
              << ", " << (Int32_t)data.tar_gear              
              << ", " << (Int32_t)data.tar_turn_lamp
              << ", " << (Int32_t)data.tar_brake_lamp
              << ", " << (Int32_t)data.tar_v 
              << ", " << (Int32_t)data.tar_a
              << ", " << data.tar_trj.curr_pos.x
              << ", " << data.tar_trj.curr_pos.y              
              << ", " << data.tar_trj.curr_pos.h
              << ", " << data.tar_trj.curr_pos.c
              << ", " << data.tar_trj.curr_pos.s
              << ", " << data.tar_trj.curr_pos.l
              << ", " << data.tar_trj.leading_pos.x
              << ", " << data.tar_trj.leading_pos.y              
              << ", " << data.tar_trj.leading_pos.h
              << ", " << data.tar_trj.leading_pos.c
              << ", " << data.tar_trj.leading_pos.s
              << ", " << data.tar_trj.leading_pos.l
              << ", " << (Int32_t)data.tar_trj.trj_direction 
              << ", " << (Int32_t)data.tar_trj.points_num  
              << ", " << data.tar_trj.points[0].x
              << ", " << data.tar_trj.points[0].y              
              << ", " << data.tar_trj.points[0].h
              << ", " << data.tar_trj.points[0].c
              << ", " << data.tar_trj.points[0].s                          
              << "} " 
              <<std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::PlanningResult& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.cur_status
              << ", " << (Int32_t)data.tar_driving_mode
              << ", " << (Int32_t)data.tar_gear              
              << ", " << (Int32_t)data.tar_turn_lamp
              << ", " << (Int32_t)data.tar_brake_lamp
              << ", " << (Int32_t)data.tar_v 
              << ", " << (Int32_t)data.tar_a
              << ", " << data.tar_trj.curr_pos.x
              << ", " << data.tar_trj.curr_pos.y              
              << ", " << data.tar_trj.curr_pos.h
              << ", " << data.tar_trj.curr_pos.c
              << ", " << data.tar_trj.curr_pos.s
              << ", " << data.tar_trj.curr_pos.l
              << ", " << data.tar_trj.leading_pos.x
              << ", " << data.tar_trj.leading_pos.y              
              << ", " << data.tar_trj.leading_pos.h
              << ", " << data.tar_trj.leading_pos.c
              << ", " << data.tar_trj.leading_pos.s
              << ", " << data.tar_trj.leading_pos.l
              << ", " << (Int32_t)data.tar_trj.trj_direction 
              << ", " << (Int32_t)data.tar_trj.points_num  
              << ", " << data.tar_trj.points[0].x
              << ", " << data.tar_trj.points[0].y              
              << ", " << data.tar_trj.points[0].h
              << ", " << data.tar_trj.points[0].c
              << ", " << data.tar_trj.points[0].s                          
              << "} " 
              <<std::endl;
  }
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::PlanningResult& data = data_array[i];
    const ad_msg::PlanningResult& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);

    EXPECT_EQ(decoded_data.cur_status, data.cur_status);
    EXPECT_EQ(decoded_data.tar_driving_mode, data.tar_driving_mode);
    EXPECT_EQ(decoded_data.tar_gear, data.tar_gear);
    EXPECT_EQ(decoded_data.tar_turn_lamp, data.tar_turn_lamp); 
    EXPECT_EQ(decoded_data.tar_brake_lamp, data.tar_brake_lamp);
    EXPECT_EQ(decoded_data.tar_v, data.tar_v);
    EXPECT_EQ(decoded_data.tar_a, data.tar_a);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.c, data.tar_trj.curr_pos.c);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.x, data.tar_trj.curr_pos.x);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.y, data.tar_trj.curr_pos.y);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.h, data.tar_trj.curr_pos.h);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.s, data.tar_trj.curr_pos.s);
    EXPECT_EQ(decoded_data.tar_trj.curr_pos.l, data.tar_trj.curr_pos.l);  
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.c, data.tar_trj.leading_pos.c);
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.x, data.tar_trj.leading_pos.x);
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.y, data.tar_trj.leading_pos.y);
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.h, data.tar_trj.leading_pos.h);
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.s, data.tar_trj.leading_pos.s);
    EXPECT_EQ(decoded_data.tar_trj.leading_pos.l, data.tar_trj.leading_pos.l);
    EXPECT_EQ(decoded_data.tar_trj.trj_direction, data.tar_trj.trj_direction);
    EXPECT_EQ(decoded_data.tar_trj.points_num, data.tar_trj.points_num);
    EXPECT_EQ(decoded_data.tar_trj.points[0].c, data.tar_trj.points[0].c);
    EXPECT_EQ(decoded_data.tar_trj.points[0].x, data.tar_trj.points[0].x);
    EXPECT_EQ(decoded_data.tar_trj.points[0].y, data.tar_trj.points[0].y);
    EXPECT_EQ(decoded_data.tar_trj.points[0].h, data.tar_trj.points[0].h);
    EXPECT_EQ(decoded_data.tar_trj.points[0].s, data.tar_trj.points[0].s);
  }  
}
TEST(HmiSettings, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::HmiSettings)*s_data_array_size;

  std::cout << "sizeof(ad_msg::HmiSettings)="
            << sizeof(ad_msg::HmiSettings) << " Bytes" << std::endl;

  ad_msg::HmiSettings data_array[s_data_array_size];
  ad_msg::HmiSettings decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::HmiSettings& data = data_array[i];

    data.adas_start = true;
    data.lks_enable = true;
    data.acc_enable = true;
    data.aeb_enable = true;
    data.target_velocity = i+2;
    data.target_acc = i+3;
  }


  Int32_t encoded_size = EncodeHmiSettingsArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeHmiSettingsArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::HmiSettings& data = data_array[i];
    const ad_msg::HmiSettings& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.adas_start, data.adas_start);
    EXPECT_EQ(decoded_data.lks_enable, data.lks_enable);
    EXPECT_EQ(decoded_data.acc_enable, data.acc_enable);
    EXPECT_EQ(decoded_data.aeb_enable, data.aeb_enable);
    EXPECT_EQ(decoded_data.target_velocity, data.target_velocity);
  }
}

TEST(EncodeTrafficSignalArray,Case_01) {
static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::TrafficSignal)*s_data_array_size;

  std::cout << "sizeof(ad_msg::TrafficSignal)="
            << sizeof(ad_msg::TrafficSignal) << " Bytes" << std::endl;

  ad_msg::TrafficSignal data_array[s_data_array_size];
  ad_msg::TrafficSignal decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::TrafficSignal& data = data_array[i];

    data.traffic_light = i+2;

  }


  Int32_t encoded_size = EncodeTrafficSignalArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeTrafficSignalArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::TrafficSignal& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << (Int32_t)data.traffic_light
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::TrafficSignal& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << (Int32_t)data.traffic_light
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::TrafficSignal& data = data_array[i];
    const ad_msg::TrafficSignal& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.traffic_light, data.traffic_light);
  
  }
}
TEST(EncodeObstacleTrackedInfoArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleTrackedInfo)*s_data_array_size;

  std::cout << "sizeof(ad_msg::ObstacleTrackedInfo)="
            << sizeof(ad_msg::ObstacleTrackedInfo) << " Bytes" << std::endl;
  common::com_memset(obstacleinfolist, 0,
                     sizeof(obstacleinfolist)); 
  ad_msg::ObstacleTrackedInfo data_array[s_data_array_size];
  ad_msg::ObstacleTrackedInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleTrackedInfo& data = data_array[i];

    data.sensor_id = i % 2;
    data.obj_type = i+1;
    data.x = (i+1)*100;
    data.y = i+2;
    data.width = i+3;
    data.length = i+9;
    data.obb.x = i+1.1F;
    data.obb.y = i+2.1F;
    data.obb.heading = i+3.1F;
    data.obb.half_width = i+4.1F;
    data.obb.half_length = i+5.1F;
    data.v_x = i+3;
    data.v_y = i+1;
    data.track_status = i+2;
    data.age = i+4; 
    data.duration = i+5;
    data.confidence = i+6;

  }
  common::com_memcpy(obstacleinfolist,data_array,sizeof(data_array));

  Int32_t encoded_size = EncodeObstacleTrackedInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleTrackedInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;



 
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleTrackedInfo& data = data_array[i];
    const ad_msg::ObstacleTrackedInfo& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.sensor_id, data.sensor_id);
    EXPECT_EQ(decoded_data.obj_type, data.obj_type);
    EXPECT_EQ(decoded_data.x, data.x);
    EXPECT_EQ(decoded_data.y, data.y);
    EXPECT_EQ(decoded_data.width, data.width);

    EXPECT_EQ(decoded_data.length, data.length);
    EXPECT_EQ(decoded_data.v_x, data.v_x);
    EXPECT_EQ(decoded_data.v_y, data.v_y);
    EXPECT_EQ(decoded_data.track_status, data.track_status);
    EXPECT_EQ(decoded_data.age, data.age);
    EXPECT_EQ(decoded_data.duration, data.duration);
    EXPECT_EQ(decoded_data.confidence, data.confidence);
    EXPECT_EQ(decoded_data.obb.x, data.obb.x);
    EXPECT_EQ(decoded_data.obb.y, data.obb.y);
    EXPECT_EQ(decoded_data.obb.half_length, data.obb.half_length);
    EXPECT_EQ(decoded_data.obb.half_width, data.obb.half_width);
    EXPECT_EQ(decoded_data.obb.heading, data.obb.heading);   
  }  
}
TEST(EncodeObstacleTrackedInfolistArray, Case_01) {
  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(ad_msg::ObstacleTrackedInfoList)*s_data_array_size;
  std::cout << "sizeof(ad_msg::ObstacleTrackedInfoList)="
            << sizeof(ad_msg::ObstacleTrackedInfoList) << " Bytes" << std::endl;

  ad_msg::ObstacleTrackedInfoList data_array[s_data_array_size];
  ad_msg::ObstacleTrackedInfoList decoded_data_array[s_data_array_size];
  Uint8_t data_buffer[s_buff_size] = { 0 };
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::ObstacleTrackedInfoList& data = data_array[i];
    data.msg_head.dst_module_id = i+1;
    data.msg_head.src_module_id = i+2;
    data.msg_head.sequence = i+3;
    data.msg_head.timestamp = i+4;
    data.msg_head.valid = i+5;
    data.obstacle_num = 3;                                                  
    common::com_memcpy(data.obstacles,obstacleinfolist,sizeof(obstacle)); 
  }
  Int32_t encoded_size = EncodeObstacleTrackedInfoListArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleTrackedInfoListArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  
  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::ObstacleTrackedInfoList& data = data_array[i];
    const ad_msg::ObstacleTrackedInfoList& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
    for (Int32_t j = 0; j <data.obstacle_num; ++j) {
      const ad_msg::ObstacleTrackedInfo& data_obj = data.obstacles[j];
      const ad_msg::ObstacleTrackedInfo& dedata_obj = decoded_data.obstacles[j];
    
    EXPECT_EQ(dedata_obj.obb.x, data_obj.obb.x);
    EXPECT_EQ(dedata_obj.obb.y, data_obj.obb.y);
    EXPECT_EQ(dedata_obj.obb.half_length, data_obj.obb.half_length);
    EXPECT_EQ(dedata_obj.obb.half_width, data_obj.obb.half_width);
    EXPECT_EQ(dedata_obj.obb.heading, data_obj.obb.heading);
    EXPECT_EQ(dedata_obj.sensor_id, data_obj.sensor_id);
    EXPECT_EQ(dedata_obj.obj_type, data_obj.obj_type);
    EXPECT_EQ(dedata_obj.x, data_obj.x);
    EXPECT_EQ(dedata_obj.y, data_obj.y);
    EXPECT_EQ(dedata_obj.width, data_obj.width);
    EXPECT_EQ(dedata_obj.length, data_obj.length);
    EXPECT_EQ(dedata_obj.v_x, data_obj.v_x);
    EXPECT_EQ(dedata_obj.v_y, data_obj.v_y);
    EXPECT_EQ(dedata_obj.track_status, data_obj.track_status);
    EXPECT_EQ(dedata_obj.age, data_obj.age);
    EXPECT_EQ(dedata_obj.duration, data_obj.duration);
    EXPECT_EQ(dedata_obj.confidence, data_obj.confidence);
    
    } 
  }

}
TEST(EncodeActionPlanningResultArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(planning::ActionPlanningResult)*s_data_array_size;

  std::cout << "sizeof(planning::ActionPlanningResult)="
            << sizeof(planning::ActionPlanningResult) << " Bytes" << std::endl;

  planning::ActionPlanningResult data_array[s_data_array_size];
  planning::ActionPlanningResult decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    planning::ActionPlanningResult& data = data_array[i];

    data.msg_head.valid = i % 2;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;
    data.action_requst = i+4;
  }


  Int32_t encoded_size = EncodeActionPlanningResultArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeActionPlanningResultArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::ActionPlanningResult& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.action_requst
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::ActionPlanningResult& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.action_requst
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::ActionPlanningResult& data = data_array[i];
    const planning::ActionPlanningResult& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.action_requst, data.action_requst);
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
  } 
}

TEST(EncodeTrajectoryPlanningResultArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(planning::TrajectoryPlanningResult)*s_data_array_size;

  std::cout << "sizeof(planning::TrajectoryPlanningResult)="
            << sizeof(planning::TrajectoryPlanningResult) << " Bytes" << std::endl;

  planning::TrajectoryPlanningResult data_array[s_data_array_size];
  planning::TrajectoryPlanningResult decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    planning::TrajectoryPlanningResult& data = data_array[i];

    data.msg_head.valid = i % 2;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;
    data.curr_pos.x = i+4.1;
    data.curr_pos.y = i+5.1;
    data.curr_pos.heading = i+6.1;
    data.curr_pos.curvature = i+7.1;
    data.curr_pos.s = i+8.7;
    data.curr_pos.l = i+9.9;

    data.leading_pos.x = i+10.8;
    data.leading_pos.y = i+11.2;
    data.leading_pos.heading = i+12.2;
    data.leading_pos.curvature = i+13.5;
    data.leading_pos.s = i+14.2;
    data.leading_pos.l = i+14.5;
   
    data.trj_direction = i+15;
    for (int32_t j = 0; j < 4; ++j) {
    common::PathPoint point;
    point.point.set_x(i+16.4 +j);
    point.point.set_y(i+17.4 +j);
    point.heading = i+18.4+j;
    point.curvature = i +19.2+j;
    point.s = i +20.1+j;
    point.l = i +21.2+j;
    data.target_trajectory_sample_points.PushBack(point);
    }
  }


  Int32_t encoded_size = EncodeTrajectoryPlanningResultArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeTrajectoryPlanningResultArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningResult& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.curr_pos.x
              << ", " << data.curr_pos.y
              << ", " << data.curr_pos.heading
              << ", " << data.curr_pos.curvature
              << ", " << data.curr_pos.s
              << ", " << data.curr_pos.l
              << ", " << data.leading_pos.x
              << ", " << data.leading_pos.y
              << ", " << data.leading_pos.heading
              << ", " << data.leading_pos.curvature
              << ", " << data.leading_pos.s
              << ", " << data.leading_pos.l
              << ", " << data.trj_direction
              << ", " << data.target_trajectory_sample_points.Size()
              << ", " << data.target_trajectory_sample_points[0].point.x()
              << ", " << data.target_trajectory_sample_points[0].point.y()
              << ", " << data.target_trajectory_sample_points[0].heading
              << ", " << data.target_trajectory_sample_points[0].curvature
              << ", " << data.target_trajectory_sample_points[0].s
              << ", " << data.target_trajectory_sample_points[0].l
              << ", " << data.target_trajectory_sample_points[1].point.x()
              << ", " << data.target_trajectory_sample_points[1].point.y()
              << ", " << data.target_trajectory_sample_points[1].heading
              << ", " << data.target_trajectory_sample_points[1].curvature
              << ", " << data.target_trajectory_sample_points[1].s
              << ", " << data.target_trajectory_sample_points[1].l
              << ", " << data.target_trajectory_sample_points[2].point.x()
              << ", " << data.target_trajectory_sample_points[2].point.y()
              << ", " << data.target_trajectory_sample_points[2].heading
              << ", " << data.target_trajectory_sample_points[2].curvature
              << ", " << data.target_trajectory_sample_points[2].s
              << ", " << data.target_trajectory_sample_points[2].l
              << ", " << data.target_trajectory_sample_points[3].point.x()
              << ", " << data.target_trajectory_sample_points[3].point.y()
              << ", " << data.target_trajectory_sample_points[3].heading
              << ", " << data.target_trajectory_sample_points[3].curvature
              << ", " << data.target_trajectory_sample_points[3].s
              << ", " << data.target_trajectory_sample_points[3].l

              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningResult& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

      std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.curr_pos.x
              << ", " << data.curr_pos.y
              << ", " << data.curr_pos.heading
              << ", " << data.curr_pos.curvature
              << ", " << data.curr_pos.s
              << ", " << data.curr_pos.l
              << ", " << data.leading_pos.x
              << ", " << data.leading_pos.y
              << ", " << data.leading_pos.heading
              << ", " << data.leading_pos.curvature
              << ", " << data.leading_pos.s
              << ", " << data.leading_pos.l
              << ", " << data.trj_direction
              << ", " << data.target_trajectory_sample_points.Size()
              << ", " << data.target_trajectory_sample_points[0].point.x()
              << ", " << data.target_trajectory_sample_points[0].point.y()
              << ", " << data.target_trajectory_sample_points[0].heading
              << ", " << data.target_trajectory_sample_points[0].curvature
              << ", " << data.target_trajectory_sample_points[0].s
              << ", " << data.target_trajectory_sample_points[0].l
              << ", " << data.target_trajectory_sample_points[1].point.x()
              << ", " << data.target_trajectory_sample_points[1].point.y()
              << ", " << data.target_trajectory_sample_points[1].heading
              << ", " << data.target_trajectory_sample_points[1].curvature
              << ", " << data.target_trajectory_sample_points[1].s
              << ", " << data.target_trajectory_sample_points[1].l
              << ", " << data.target_trajectory_sample_points[2].point.x()
              << ", " << data.target_trajectory_sample_points[2].point.y()
              << ", " << data.target_trajectory_sample_points[2].heading
              << ", " << data.target_trajectory_sample_points[2].curvature
              << ", " << data.target_trajectory_sample_points[2].s
              << ", " << data.target_trajectory_sample_points[2].l
              << ", " << data.target_trajectory_sample_points[3].point.x()
              << ", " << data.target_trajectory_sample_points[3].point.y()
              << ", " << data.target_trajectory_sample_points[3].heading
              << ", " << data.target_trajectory_sample_points[3].curvature
              << ", " << data.target_trajectory_sample_points[3].s
              << ", " << data.target_trajectory_sample_points[3].l

              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::TrajectoryPlanningResult& data = data_array[i];
    const planning::TrajectoryPlanningResult& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.trj_direction, data.trj_direction);
    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
  
  
    EXPECT_EQ(decoded_data.curr_pos.curvature, data.curr_pos.curvature);
    EXPECT_EQ(decoded_data.curr_pos.x, data.curr_pos.x);
    EXPECT_EQ(decoded_data.curr_pos.y, data.curr_pos.y);
    EXPECT_EQ(decoded_data.curr_pos.heading, data.curr_pos.heading);
    EXPECT_EQ(decoded_data.curr_pos.s, data.curr_pos.s);
    EXPECT_EQ(decoded_data.curr_pos.l, data.curr_pos.l);  

    EXPECT_EQ(decoded_data.leading_pos.curvature, data.leading_pos.curvature);
    EXPECT_EQ(decoded_data.leading_pos.x, data.leading_pos.x);
    EXPECT_EQ(decoded_data.leading_pos.y, data.leading_pos.y);
    EXPECT_EQ(decoded_data.leading_pos.heading, data.leading_pos.heading);
    EXPECT_EQ(decoded_data.leading_pos.s, data.leading_pos.s);
    EXPECT_EQ(decoded_data.leading_pos.l, data.leading_pos.l);  
    
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].s, data.target_trajectory_sample_points[0].s);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].l, data.target_trajectory_sample_points[0].l);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].heading, data.target_trajectory_sample_points[0].heading);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].curvature, data.target_trajectory_sample_points[0].curvature);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].point.x(), data.target_trajectory_sample_points[0].point.x());
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[0].point.y(), data.target_trajectory_sample_points[0].point.y());
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].s, data.target_trajectory_sample_points[1].s);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].l, data.target_trajectory_sample_points[1].l);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].heading, data.target_trajectory_sample_points[1].heading);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].curvature, data.target_trajectory_sample_points[1].curvature);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].point.x(), data.target_trajectory_sample_points[1].point.x());
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[1].point.y(), data.target_trajectory_sample_points[1].point.y());


    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].s, data.target_trajectory_sample_points[2].s);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].l, data.target_trajectory_sample_points[2].l);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].heading, data.target_trajectory_sample_points[2].heading);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].curvature, data.target_trajectory_sample_points[2].curvature);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].point.x(), data.target_trajectory_sample_points[2].point.x());
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[2].point.y(), data.target_trajectory_sample_points[2].point.y());

   
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].s, data.target_trajectory_sample_points[3].s);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].l, data.target_trajectory_sample_points[3].l);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].heading, data.target_trajectory_sample_points[3].heading);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].curvature, data.target_trajectory_sample_points[3].curvature);
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].point.x(), data.target_trajectory_sample_points[3].point.x());
    EXPECT_EQ(decoded_data.target_trajectory_sample_points[3].point.y(), data.target_trajectory_sample_points[3].point.y());
  } 
}

TEST(EncodeVelocityPlanningResultArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(planning::VelocityPlanningResult)*s_data_array_size;

  std::cout << "sizeof(planning::VelocityPlanningResult)="
            << sizeof(planning::VelocityPlanningResult) << " Bytes" << std::endl;

  planning::VelocityPlanningResult data_array[s_data_array_size];
  planning::VelocityPlanningResult decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    planning::VelocityPlanningResult& data = data_array[i];

    data.msg_head.valid = i % 2;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;
    data.tar_type = i+4;
    data.tar_v = i+5.2;
    data.tar_a = i+6.3;
    data.tar_pos.x = i+7.7;
    data.tar_pos.y = i+8.8;
    data.tar_pos.heading = i+9.7;
    data.tar_pos.s = i+10.8;
  }


  Int32_t encoded_size = EncodeVelocityPlanningResultArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeVelocityPlanningResultArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::VelocityPlanningResult& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.tar_type
              << ", " << data.tar_v
              << ", " << data.tar_a
              << ", " << data.tar_pos.x
              << ", " << data.tar_pos.y
              << ", " << data.tar_pos.heading
              << ", " << data.tar_pos.s
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::VelocityPlanningResult& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << ", " << data.tar_type
              << ", " << data.tar_v
              << ", " << data.tar_a
              << ", " << data.tar_pos.x
              << ", " << data.tar_pos.y
              << ", " << data.tar_pos.heading
              << ", " << data.tar_pos.s
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const planning::VelocityPlanningResult& data = data_array[i];
    const planning::VelocityPlanningResult& decoded_data = decoded_data_array[i];
    EXPECT_EQ(decoded_data.tar_type, data.tar_type);
    EXPECT_EQ(decoded_data.tar_v, data.tar_v);
    EXPECT_EQ(decoded_data.tar_a, data.tar_a);

   EXPECT_EQ(decoded_data.tar_pos.x, data.tar_pos.x);
   EXPECT_EQ(decoded_data.tar_pos.y, data.tar_pos.y);
   EXPECT_EQ(decoded_data.tar_pos.heading, data.tar_pos.heading);
   EXPECT_EQ(decoded_data.tar_pos.s, data.tar_pos.s);

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);
  } 
}

// struct MsgHead {
//   /// 此报文是否有效
//   bool valid;
//   /// 消息的顺序号，用于检索及同步消息报文
//   Uint32_t sequence;
//   /// 时间戳，用于时间同步
//   Int64_t timestamp;
//   /// 发送模块的ID
//   Uint32_t src_module_id;
//   /// 目标模块的ID
//   Uint32_t dst_module_id;
// };
TEST(EncodeMsgHeadArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(ad_msg::MsgHead)*s_data_array_size;

  std::cout << "sizeof(ad_msg::MsgHead)="
            << sizeof(ad_msg::MsgHead) << " Bytes" << std::endl;

  ad_msg::MsgHead data_array[s_data_array_size];
  ad_msg::MsgHead decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    ad_msg::MsgHead& data = data_array[i];

    data.valid = i % 2;
    data.sequence = i+1;
    data.timestamp = (i+1)*100;
    data.src_module_id = i+2;
    data.dst_module_id = i+3;
  }


  Int32_t encoded_size = EncodeMsgHeadArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeMsgHeadArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::MsgHead& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.valid
              << ", " << data.sequence
              << ", " << data.timestamp
              << ", " << data.src_module_id
              << ", " << data.dst_module_id
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::MsgHead& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.valid
              << ", " << data.sequence
              << ", " << data.timestamp
              << ", " << data.src_module_id
              << ", " << data.dst_module_id
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const ad_msg::MsgHead& data = data_array[i];
    const ad_msg::MsgHead& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.valid, data.valid);
    EXPECT_EQ(decoded_data.sequence, data.sequence);
    EXPECT_EQ(decoded_data.timestamp, data.timestamp);
    EXPECT_EQ(decoded_data.src_module_id, data.src_module_id);
    EXPECT_EQ(decoded_data.dst_module_id, data.dst_module_id);
  }
}



TEST(EncodeVec2dArray, Case_01) {

  common::Vec2d data(1.36F, 56.9F);
  common::Vec2d decoded_data;
  common::Vec2d data_array[3];
  data_array[0].set_x(1.23F);
  data_array[0].set_y(2.23F);
  data_array[1].set_x(3.23F);
  data_array[1].set_y(4.23F);
  data_array[2].set_x(5.23F);
  data_array[2].set_y(6.23F);
  common::Vec2d decoded_data_array[3];

  Uint8_t data_buffer[512] = { 0 };
  Int32_t encoded_size = EncodeVec2dArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeVec2dArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data={(" << data.x()
            << ", " << data.y()
            << ")}, decoded_data={(" << decoded_data.x()
            << ", " << decoded_data.y() << ")}"
            << std::endl;

  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data.x(), data.x());
  EXPECT_EQ(decoded_data.y(), data.y());

  encoded_size = EncodeVec2dArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeVec2dArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={(" << data_array[0].x()
            << ", " << data_array[0].y()
            << ") (" << data_array[1].x()
            << ", " << data_array[1].y()
            << ") (" << data_array[2].x()
            << ", " << data_array[2].y()
            << ")}"
            << ", decoded_data_array={(" << decoded_data_array[0].x()
            << ", " << decoded_data_array[0].y()
            << ") (" << decoded_data_array[1].x()
            << ", " << decoded_data_array[1].y()
            << ") (" << decoded_data_array[2].x()
            << ", " << decoded_data_array[2].y()
            << ")}"
            << std::endl;

  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0].x(), data_array[0].x());
  EXPECT_EQ(decoded_data_array[0].y(), data_array[0].y());
  EXPECT_EQ(decoded_data_array[1].x(), data_array[1].x());
  EXPECT_EQ(decoded_data_array[1].y(), data_array[1].y());
  EXPECT_EQ(decoded_data_array[2].x(), data_array[2].x());
  EXPECT_EQ(decoded_data_array[2].y(), data_array[2].y());
}



TEST(EncodeOBBox2dArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(common::OBBox2d)*s_data_array_size;

  std::cout << "sizeof(common::OBBox2d)="
            << sizeof(common::OBBox2d) << " Bytes" << std::endl;

  common::OBBox2d data_array[s_data_array_size];
  common::OBBox2d decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    common::OBBox2d& data = data_array[i];

    data.set_center(common::Vec2d((i+1)*1.269F, (i+1)*2.269F));
    data.set_unit_direction((i+1)*1.56F, (i+1)*2.56F);
    data.set_extents(common::Vec2d((i+1)*3.89F, (i+1)*3.89F));
  }


  Int32_t encoded_size = EncodeOBBox2dArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeOBBox2dArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const common::OBBox2d& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.center().x()
              << ", " << data.center().y()
              << ", " << data.unit_direction_x().x()
              << ", " << data.unit_direction_x().y()
              << ", " << data.unit_direction_y().x()
              << ", " << data.unit_direction_y().y()
              << ", " << data.extents().x()
              << ", " << data.extents().y()
              << "}"
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const common::OBBox2d& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.center().x()
              << ", " << data.center().y()
              << ", " << data.unit_direction_x().x()
              << ", " << data.unit_direction_x().y()
              << ", " << data.unit_direction_y().x()
              << ", " << data.unit_direction_y().y()
              << ", " << data.extents().x()
              << ", " << data.extents().y()
              << "}"
              << std::endl;
  }

  // Check result
  EXPECT_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const common::OBBox2d& data = data_array[i];
    const common::OBBox2d& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.center().x(), data.center().x());
    EXPECT_EQ(decoded_data.center().y(), data.center().y());
    EXPECT_EQ(decoded_data.unit_direction_x().x(), data.unit_direction_x().x());
    EXPECT_EQ(decoded_data.unit_direction_x().y(), data.unit_direction_x().y());
    EXPECT_EQ(decoded_data.unit_direction_y().x(), data.unit_direction_y().x());
    EXPECT_EQ(decoded_data.unit_direction_y().y(), data.unit_direction_y().y());
    EXPECT_EQ(decoded_data.extents().x(), data.extents().x());
    EXPECT_EQ(decoded_data.extents().y(), data.extents().y());
  }
}



TEST(EncodePathPointArray, Case_01) {

  common::PathPoint data;
  data.point.set_x(1.23F);
  data.point.set_y(2.33F);
  data.heading = 3.36F;
  data.curvature = 4.69F;
  data.s = 5.96F;
  data.l = 6.98F;
  common::PathPoint decoded_data;
  common::PathPoint data_array[3];
  data_array[0].point.set_x(1.23F);
  data_array[0].point.set_y(2.23F);
  data_array[0].heading = 3.36F;
  data_array[0].curvature = 4.69F;
  data_array[0].s = 5.96F;
  data_array[0].l = 6.98F;
  data_array[1].point.set_x(3.23F);
  data_array[1].point.set_y(4.23F);
  data_array[1].heading = 3.361F;
  data_array[1].curvature = 4.691F;
  data_array[1].s = 5.961F;
  data_array[1].l = 6.981F;
  data_array[2].point.set_x(5.23F);
  data_array[2].point.set_y(6.23F);
  data_array[2].heading = 3.3612F;
  data_array[2].curvature = 4.6912F;
  data_array[2].s = 5.9612F;
  data_array[2].l = 6.9812F;
  common::PathPoint decoded_data_array[3];

  Uint8_t data_buffer[512] = { 0 };
  Int32_t encoded_size = EncodePathPointArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodePathPointArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data={(" << data.point.x()
            << ", " << data.point.y()
            << ", " << data.heading
            << ", " << data.curvature
            << ", " << data.s
            << ", " << data.l
            << ")}, decoded_data={(" << decoded_data.point.x()
            << ", " << decoded_data.point.y()
            << ", " << decoded_data.heading
            << ", " << decoded_data.curvature
            << ", " << decoded_data.s
            << ", " << decoded_data.l
            << ")}"
            << std::endl;

  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data.point.x(), data.point.x());
  EXPECT_EQ(decoded_data.point.y(), data.point.y());
  EXPECT_EQ(decoded_data.heading, data.heading);
  EXPECT_EQ(decoded_data.curvature, data.curvature);
  EXPECT_EQ(decoded_data.s, data.s);
  EXPECT_EQ(decoded_data.l, data.l);

  encoded_size = EncodePathPointArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodePathPointArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={(" << data_array[0].point.x()
            << ", " << data_array[0].point.y()
            << ", " << data_array[0].heading
            << ", " << data_array[0].curvature
            << ", " << data_array[0].s
            << ", " << data_array[0].l
            << ") (" << data_array[1].point.x()
            << ", " << data_array[1].point.y()
            << ", " << data_array[1].heading
            << ", " << data_array[1].curvature
            << ", " << data_array[1].s
            << ", " << data_array[1].l
            << ") (" << data_array[2].point.x()
            << ", " << data_array[2].point.y()
            << ", " << data_array[2].heading
            << ", " << data_array[2].curvature
            << ", " << data_array[2].s
            << ", " << data_array[2].l
            << ")}"
            << ", decoded_data_array={(" << decoded_data_array[0].point.x()
            << ", " << decoded_data_array[0].point.y()
            << ", " << decoded_data_array[0].heading
            << ", " << decoded_data_array[0].curvature
            << ", " << decoded_data_array[0].s
            << ", " << decoded_data_array[0].l
            << ") (" << decoded_data_array[1].point.x()
            << ", " << decoded_data_array[1].point.y()
            << ", " << decoded_data_array[1].heading
            << ", " << decoded_data_array[1].curvature
            << ", " << decoded_data_array[1].s
            << ", " << decoded_data_array[1].l
            << ") (" << decoded_data_array[2].point.x()
            << ", " << decoded_data_array[2].point.y()
            << ", " << decoded_data_array[2].heading
            << ", " << decoded_data_array[2].curvature
            << ", " << decoded_data_array[2].s
            << ", " << decoded_data_array[2].l
            << ")}"
            << std::endl;

  EXPECT_EQ(encoded_size, 72);
  EXPECT_EQ(decoded_size, 72);
  EXPECT_EQ(decoded_data_array[0].point.x(), data_array[0].point.x());
  EXPECT_EQ(decoded_data_array[0].point.y(), data_array[0].point.y());
  EXPECT_EQ(decoded_data_array[0].heading, data_array[0].heading);
  EXPECT_EQ(decoded_data_array[0].curvature, data_array[0].curvature);
  EXPECT_EQ(decoded_data_array[0].s, data_array[0].s);
  EXPECT_EQ(decoded_data_array[0].l, data_array[0].l);

  EXPECT_EQ(decoded_data_array[1].point.x(), data_array[1].point.x());
  EXPECT_EQ(decoded_data_array[1].point.y(), data_array[1].point.y());
  EXPECT_EQ(decoded_data_array[1].heading, data_array[1].heading);
  EXPECT_EQ(decoded_data_array[1].curvature, data_array[1].curvature);
  EXPECT_EQ(decoded_data_array[1].s, data_array[1].s);
  EXPECT_EQ(decoded_data_array[1].l, data_array[1].l);

  EXPECT_EQ(decoded_data_array[2].point.x(), data_array[2].point.x());
  EXPECT_EQ(decoded_data_array[2].point.y(), data_array[2].point.y());
  EXPECT_EQ(decoded_data_array[2].heading, data_array[2].heading);
  EXPECT_EQ(decoded_data_array[2].curvature, data_array[2].curvature);
  EXPECT_EQ(decoded_data_array[2].s, data_array[2].s);
  EXPECT_EQ(decoded_data_array[2].l, data_array[2].l);
}


TEST(EncodeTrajectoryPointArray, Case_01) {
  // struct TrajectoryPoint {
  //   /// 点的空间信息，包含位置、航向角、曲率、弧长等。
  //   PathPoint path_point;
  //   /// 速度（单位：m/s）
  //   geo_var_t v;
  //   /// 加速度（单位：m/s^2）
  //   geo_var_t a;
  //   /// 横摆角速度（单位：rad/s）
  //   geo_var_t yaw_rate;
  //   /// 相对于曲线起点已行驶的相对时间（单位：s）
  //   geo_var_t relative_time;
  // };

  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(common::TrajectoryPoint)*s_data_array_size;

  std::cout << "sizeof(common::TrajectoryPoint)="
            << sizeof(common::TrajectoryPoint) << " Bytes" << std::endl;

  common::TrajectoryPoint data_array[s_data_array_size];
  common::TrajectoryPoint decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    common::TrajectoryPoint& point = data_array[i];

    point.path_point.point.set_x((i+1) * 1.2F);
    point.path_point.point.set_y((i+1) * 2.3F);
    point.path_point.heading = (i+1) * 3.4F;
    point.path_point.curvature = (i+1) * 4.5F;
    point.path_point.s = (i+1) * 5.6F;
    point.path_point.l = (i+1) * 6.7F;
    point.v = (i+1) * 7.8F;
    point.a = (i+1) * 8.9F;
    point.yaw_rate = (i+1) * 9.1F;
    point.relative_time = (i+1) * 10.2F;
  }

  Int32_t encoded_size = EncodeTrajectoryPointArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeTrajectoryPointArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    common::TrajectoryPoint& point = data_array[i];

    std::cout << "point [" << i << "]:" << std::endl;

    std::cout << "  {" << point.path_point.point.x()
              << ", " << point.path_point.point.y()
              << ", " << point.path_point.heading
              << ", " << point.path_point.curvature
              << ", " << point.path_point.s
              << ", " << point.path_point.l
              << ", " << point.v
              << ", " << point.a
              << ", " << point.yaw_rate
              << ", " << point.relative_time
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    common::TrajectoryPoint& point = decoded_data_array[i];

    std::cout << "point [" << i << "]:" << std::endl;

    std::cout << "  {" << point.path_point.point.x()
              << ", " << point.path_point.point.y()
              << ", " << point.path_point.heading
              << ", " << point.path_point.curvature
              << ", " << point.path_point.s
              << ", " << point.path_point.l
              << ", " << point.v
              << ", " << point.a
              << ", " << point.yaw_rate
              << ", " << point.relative_time
              << std::endl;
  }

  // Check result
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    common::TrajectoryPoint& point = data_array[i];
    common::TrajectoryPoint& decoded_point = decoded_data_array[i];

    CHECK_EQ(decoded_point.path_point.point.x(), point.path_point.point.x());
    CHECK_EQ(decoded_point.path_point.point.y(), point.path_point.point.y());
    CHECK_EQ(decoded_point.path_point.heading, point.path_point.heading);
    CHECK_EQ(decoded_point.path_point.curvature, point.path_point.curvature);
    CHECK_EQ(decoded_point.path_point.s, point.path_point.s);
    CHECK_EQ(decoded_point.path_point.l, point.path_point.l);
    CHECK_EQ(decoded_point.v, point.v);
    CHECK_EQ(decoded_point.a, point.a);
    CHECK_EQ(decoded_point.yaw_rate, point.yaw_rate);
    CHECK_EQ(decoded_point.relative_time, point.relative_time);
  }

}


TEST(EncodeLaneInfoArray, Case_01) {
  // LaneInfo
  // 车道id
  // ID lane_id;
  // 车道索引
  // Int32_t lane_index;
  // 车道中心线
  // common::StaticVector<common::Vec2d,
  //     common::Path::kMaxPathPointNum> central_curve;
  // 车道左边界
  // Boundary left_boundary;
  // 车道右边界
  // Boundary right_boundary;

  // struct Boundary {
  //   struct BoundaryAssociation {
  //     /// 在参考线上的路径长
  //     map_var_t s;
  //     /// 宽度（车道半宽）
  //     map_var_t width;
  //     /// 边线类型
  //     Int32_t type;
  //     /// 边线型点
  //     common::Vec2d point;
  //   };
  //   /// 车道边线
  //   common::StaticVector<BoundaryAssociation,
  //   common::Path::kMaxPathPointNum> curve;
  // };

  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(driv_map::LaneInfo);

  std::cout << "sizeof(driv_map::LaneInfo)="
            << sizeof(driv_map::LaneInfo) / 1024 << " Kbytes" << std::endl;

  driv_map::LaneInfo data_array[s_data_array_size];
  driv_map::LaneInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::LaneInfo& lane = data_array[i];

    lane.lane_index = i+1;

    Int32_t central_curve_size = 4+i;
    lane.central_curve.Resize(central_curve_size);
    for (Int32_t j = 0; j < central_curve_size; ++j) {
      common::Vec2d& point = lane.central_curve[j];
      point.set_x((j+1) * 1.2F);
      point.set_y((j+1) * 1.3F);
    }

    Int32_t left_boundary_curve_size = 5+i;
    lane.left_boundary.curve.Resize(left_boundary_curve_size);
    for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.left_boundary.curve[j];

      point.s = (j+1) * 2.3F;
      point.width = (j+1) * 3.3F;
      point.type = (j+1);
      point.point.set_x((j+1) * 4.2F);
      point.point.set_y((j+1) * 5.2F);
    }

    Int32_t right_boundary_curve_size = 6+i;
    lane.right_boundary.curve.Resize(right_boundary_curve_size);
    for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.right_boundary.curve[j];

      point.s = (j+1) * 6.3F;
      point.width = (j+1) * 7.3F;
      point.type = (j+1);
      point.point.set_x((j+1) * 8.2F);
      point.point.set_y((j+1) * 9.3F);
    }
  }

  Int32_t encoded_size = EncodeLaneInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeLaneInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::LaneInfo& lane = data_array[i];

    std::cout << "lane [" << i << "]:" << std::endl;

    std::cout << "  lane_index=" << lane.lane_index << std::endl;

    std::cout << "  central_curve=";
    Int32_t central_curve_size = lane.central_curve.Size();
    for (Int32_t j = 0; j < central_curve_size; ++j) {
      common::Vec2d& point = lane.central_curve[j];
      std::cout << "  (" << point.x() << "," << point.y() << ")";
    }
    std::cout << std::endl;

    std::cout << "  left_boundary=";
    Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
    for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.left_boundary.curve[j];

      std::cout << "  (" << point.s << "," << point.width
                << "," << point.type
                << point.point.x() << "," << point.point.y() << ")";
    }
    std::cout << std::endl;

    std::cout << "  right_boundary=";
    Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
    for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.right_boundary.curve[j];

      std::cout << "  (" << point.s << "," << point.width
                << "," << point.type
                << point.point.x() << "," << point.point.y() << ")";
    }
    std::cout << std::endl;
  }

  std::cout << "\ndecoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::LaneInfo& lane = decoded_data_array[i];

    std::cout << "lane [" << i << "]:" << std::endl;

    std::cout << "  lane_index=" << lane.lane_index << std::endl;

    std::cout << "  central_curve=";
    Int32_t central_curve_size = lane.central_curve.Size();
    for (Int32_t j = 0; j < central_curve_size; ++j) {
      common::Vec2d& point = lane.central_curve[j];
      std::cout << "  (" << point.x() << "," << point.y() << ")";
    }
    std::cout << std::endl;

    std::cout << "  left_boundary=";
    Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
    for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.left_boundary.curve[j];

      std::cout << "  (" << point.s << "," << point.width
                << "," << point.type
                << point.point.x() << "," << point.point.y() << ")";
    }
    std::cout << std::endl;

    std::cout << "  right_boundary=";
    Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
    for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.right_boundary.curve[j];

      std::cout << "  (" << point.s << "," << point.width
                << "," << point.type
                << point.point.x() << "," << point.point.y() << ")";
    }
    std::cout << std::endl;
  }

  // Check result
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::LaneInfo& lane = data_array[i];
    const driv_map::LaneInfo& decoded_lane = decoded_data_array[i];

    CHECK_EQ(decoded_lane.lane_index, lane.lane_index);
    CHECK_EQ(decoded_lane.central_curve.Size(), lane.central_curve.Size());

    Int32_t central_curve_size = lane.central_curve.Size();
    for (Int32_t j = 0; j < central_curve_size; ++j) {
      const common::Vec2d& point = lane.central_curve[j];
      const common::Vec2d& decoded_point = decoded_lane.central_curve[j];

      CHECK_EQ(decoded_point.x(), point.x());
      CHECK_EQ(decoded_point.y(), point.y());
    }


    CHECK_EQ(decoded_lane.left_boundary.curve.Size(), lane.left_boundary.curve.Size());
    Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
    for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.left_boundary.curve[j];
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
          decoded_lane.left_boundary.curve[j];

      CHECK_EQ(decoded_point.s, point.s);
      CHECK_EQ(decoded_point.width, point.width);
      CHECK_EQ(decoded_point.type, point.type);
      CHECK_EQ(decoded_point.point.x(), point.point.x());
      CHECK_EQ(decoded_point.point.y(), point.point.y());
    }

    CHECK_EQ(decoded_lane.right_boundary.curve.Size(), lane.right_boundary.curve.Size());
    Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
    for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
          lane.right_boundary.curve[j];
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
          decoded_lane.right_boundary.curve[j];

      CHECK_EQ(decoded_point.s, point.s);
      CHECK_EQ(decoded_point.width, point.width);
      CHECK_EQ(decoded_point.type, point.type);
      CHECK_EQ(decoded_point.point.x(), point.point.x());
      CHECK_EQ(decoded_point.point.y(), point.point.y());
    }
  }
}


TEST(EncodeMapInfoArray, Case_01) {
  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(driv_map::MapInfo);

  std::cout << "sizeof(driv_map::MapInfo)="
            << sizeof(driv_map::MapInfo) / 1024 << " Kbytes" << std::endl;

  driv_map::MapInfo data_array[s_data_array_size];
  driv_map::MapInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t k = 0; k < s_data_array_size; ++k) {
    driv_map::MapInfo& map = data_array[k];

    Int32_t lane_table_size = 1+k;
    map.lane_table.Resize(lane_table_size);

    for (Int32_t i = 0; i < lane_table_size; ++i) {
      driv_map::LaneInfo& lane = map.lane_table[i];

      lane.lane_index = i+1;

      Int32_t central_curve_size = 4+i;
      lane.central_curve.Resize(central_curve_size);
      for (Int32_t j = 0; j < central_curve_size; ++j) {
        common::Vec2d& point = lane.central_curve[j];
        point.set_x((j+1) * 1.2F);
        point.set_y((j+1) * 1.3F);
      }

      Int32_t left_boundary_curve_size = 5+i;
      lane.left_boundary.curve.Resize(left_boundary_curve_size);
      for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
        driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[j];

        point.s = (j+1) * 2.3F;
        point.width = (j+1) * 3.3F;
        point.type = (j+1);
        point.point.set_x((j+1) * 4.2F);
        point.point.set_y((j+1) * 5.2F);
      }

      Int32_t right_boundary_curve_size = 6+i;
      lane.right_boundary.curve.Resize(right_boundary_curve_size);
      for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
        driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[j];

        point.s = (j+1) * 6.3F;
        point.width = (j+1) * 7.3F;
        point.type = (j+1);
        point.point.set_x((j+1) * 8.2F);
        point.point.set_y((j+1) * 9.3F);
      }
    }
  }

  Int32_t encoded_size = EncodeMapInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeMapInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t k = 0; k < s_data_array_size; ++k) {
    const driv_map::MapInfo& map = data_array[k];

    std::cout << "map[" << k << "]:" << std::endl;

    Int32_t lane_table_size = map.lane_table.Size();
    for (Int32_t i = 0; i < lane_table_size; ++i) {
      const driv_map::LaneInfo& lane = map.lane_table[i];

      std::cout << "lane [" << i << "]:" << std::endl;

      std::cout << "  lane_index=" << lane.lane_index << std::endl;

      std::cout << "  central_curve=";
      Int32_t central_curve_size = lane.central_curve.Size();
      for (Int32_t j = 0; j < central_curve_size; ++j) {
        const common::Vec2d& point = lane.central_curve[j];
        std::cout << "  (" << point.x() << "," << point.y() << ")";
      }
      std::cout << std::endl;

      std::cout << "  left_boundary=";
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[j];

        std::cout << "  (" << point.s << "," << point.width
                  << "," << point.type
                  << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout << std::endl;

      std::cout << "  right_boundary=";
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[j];

        std::cout << "  (" << point.s << "," << point.width
                  << "," << point.type
                  << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout << std::endl;
    }
  }

  std::cout << "\ndecoded_data_array=" << std::endl;
  for (Int32_t k = 0; k < s_data_array_size; ++k) {
    const driv_map::MapInfo& map = decoded_data_array[k];

    std::cout << "map[" << k << "]:" << std::endl;

    Int32_t lane_table_size = map.lane_table.Size();
    for (Int32_t i = 0; i < lane_table_size; ++i) {
      const driv_map::LaneInfo& lane = map.lane_table[i];

      std::cout << "lane [" << i << "]:" << std::endl;

      std::cout << "  lane_index=" << lane.lane_index << std::endl;

      std::cout << "  central_curve=";
      Int32_t central_curve_size = lane.central_curve.Size();
      for (Int32_t j = 0; j < central_curve_size; ++j) {
        const common::Vec2d& point = lane.central_curve[j];
        std::cout << "  (" << point.x() << "," << point.y() << ")";
      }
      std::cout << std::endl;

      std::cout << "  left_boundary=";
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[j];

        std::cout << "  (" << point.s << "," << point.width
                  << "," << point.type
                  << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout << std::endl;

      std::cout << "  right_boundary=";
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[j];

        std::cout << "  (" << point.s << "," << point.width
                  << "," << point.type
                  << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout << std::endl;
    }
  }

  // Check result
  for (Int32_t k = 0; k < s_data_array_size; ++k) {
    const driv_map::MapInfo& map = data_array[k];
    const driv_map::MapInfo& decoded_map = decoded_data_array[k];

    CHECK_EQ(decoded_map.lane_table.Size(), map.lane_table.Size());

    Int32_t lane_table_size = map.lane_table.Size();
    for (Int32_t i = 0; i < lane_table_size; ++i) {
      const driv_map::LaneInfo& lane = map.lane_table[i];
      const driv_map::LaneInfo& decoded_lane = decoded_map.lane_table[i];

      CHECK_EQ(decoded_lane.lane_index, lane.lane_index);
      CHECK_EQ(decoded_lane.central_curve.Size(), lane.central_curve.Size());

      Int32_t central_curve_size = lane.central_curve.Size();
      for (Int32_t j = 0; j < central_curve_size; ++j) {
        const common::Vec2d& point = lane.central_curve[j];
        const common::Vec2d& decoded_point = decoded_lane.central_curve[j];

        CHECK_EQ(decoded_point.x(), point.x());
        CHECK_EQ(decoded_point.y(), point.y());
      }


      CHECK_EQ(decoded_lane.left_boundary.curve.Size(), lane.left_boundary.curve.Size());
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[j];
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
            decoded_lane.left_boundary.curve[j];

        CHECK_EQ(decoded_point.s, point.s);
        CHECK_EQ(decoded_point.width, point.width);
        CHECK_EQ(decoded_point.type, point.type);
        CHECK_EQ(decoded_point.point.x(), point.point.x());
        CHECK_EQ(decoded_point.point.y(), point.point.y());
      }

      CHECK_EQ(decoded_lane.right_boundary.curve.Size(), lane.right_boundary.curve.Size());
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[j];
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
            decoded_lane.right_boundary.curve[j];

        CHECK_EQ(decoded_point.s, point.s);
        CHECK_EQ(decoded_point.width, point.width);
        CHECK_EQ(decoded_point.type, point.type);
        CHECK_EQ(decoded_point.point.x(), point.point.x());
        CHECK_EQ(decoded_point.point.y(), point.point.y());
      }
    }
  }
}



TEST(EncodeCollisionRiskTestResultArray, Case_01) {
  // struct CollisionRiskTestResult {
  //   /// 障碍物在列表中的索引
  //   Int32_t obj_list_index;
  //   /// 此障碍物对应的风险值
  //   Int32_t risk_value;
  //   /// 与障碍物之间的距离(动态距离)
  //   map_var_t dynamic_distance;
  //   /// 与障碍物之间的距离(静态距离)
  //   map_var_t static_distance;
  //   /// 障碍物在参考线上的路径长
  //   map_var_t obj_s_ref;
  //   /// 障碍物在与参考线上之间的横向偏差
  //   map_var_t obj_l_ref;
  //   /// 与此障碍物有碰撞风险的目标物体在轨迹上的路径长
  //   map_var_t collision_s;
  //   /// 保存了障碍物采样点相关的信息
  //   /// （意味着障碍物在其这个采样点的位置，存在碰撞风险）
  //   common::TrajectoryPoint obj_traj_point;
  // };

  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(driv_map::CollisionRiskTestResult)*s_data_array_size;

  std::cout << "sizeof(driv_map::CollisionRiskTestResult)="
            << sizeof(driv_map::CollisionRiskTestResult) << " Bytes" << std::endl;

  driv_map::CollisionRiskTestResult data_array[s_data_array_size];
  driv_map::CollisionRiskTestResult decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::CollisionRiskTestResult& data = data_array[i];

    data.obj_list_index = i+1;
    data.risk_value = (i+1) * 2;
    data.dynamic_distance = (i+1) * 10.1F;
    data.static_distance = (i+1) * 11.2F;
    data.obj_s_ref = (i+1) * 12.3F;
    data.obj_l_ref = (i+1) * 13.4F;
    data.collision_s = (i+1) * 14.5F;

    data.obj_traj_point.path_point.point.set_x((i+1) * 1.2F);
    data.obj_traj_point.path_point.point.set_y((i+1) * 2.3F);
    data.obj_traj_point.path_point.heading = (i+1) * 3.4F;
    data.obj_traj_point.path_point.curvature = (i+1) * 4.5F;
    data.obj_traj_point.path_point.s = (i+1) * 5.6F;
    data.obj_traj_point.path_point.l = (i+1) * 6.7F;
    data.obj_traj_point.v = (i+1) * 7.8F;
    data.obj_traj_point.a = (i+1) * 8.9F;
    data.obj_traj_point.yaw_rate = (i+1) * 9.1F;
    data.obj_traj_point.relative_time = (i+1) * 10.2F;
  }

  Int32_t encoded_size = EncodeCollisionRiskTestResultArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeCollisionRiskTestResultArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::CollisionRiskTestResult& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << data.obj_list_index
              << ", " << data.risk_value
              << ", " << data.dynamic_distance
              << ", " << data.static_distance
              << ", " << data.obj_s_ref
              << ", " << data.obj_l_ref
              << ", " << data.collision_s
              << ", " << data.obj_traj_point.path_point.point.x()
              << ", " << data.obj_traj_point.path_point.point.y()
              << ", " << data.obj_traj_point.path_point.heading
              << ", " << data.obj_traj_point.path_point.curvature
              << ", " << data.obj_traj_point.path_point.s
              << ", " << data.obj_traj_point.path_point.l
              << ", " << data.obj_traj_point.v
              << ", " << data.obj_traj_point.a
              << ", " << data.obj_traj_point.yaw_rate
              << ", " << data.obj_traj_point.relative_time
              << std::endl;
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::CollisionRiskTestResult& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << data.obj_list_index
              << ", " << data.risk_value
              << ", " << data.dynamic_distance
              << ", " << data.static_distance
              << ", " << data.obj_s_ref
              << ", " << data.obj_l_ref
              << ", " << data.collision_s
              << ", " << data.obj_traj_point.path_point.point.x()
              << ", " << data.obj_traj_point.path_point.point.y()
              << ", " << data.obj_traj_point.path_point.heading
              << ", " << data.obj_traj_point.path_point.curvature
              << ", " << data.obj_traj_point.path_point.s
              << ", " << data.obj_traj_point.path_point.l
              << ", " << data.obj_traj_point.v
              << ", " << data.obj_traj_point.a
              << ", " << data.obj_traj_point.yaw_rate
              << ", " << data.obj_traj_point.relative_time
              << std::endl;
  }

  // Check result
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::CollisionRiskTestResult& data = data_array[i];
    const driv_map::CollisionRiskTestResult& decoded_data = decoded_data_array[i];

    CHECK_EQ(decoded_data.obj_list_index, data.obj_list_index);
    CHECK_EQ(decoded_data.risk_value, data.risk_value);
    CHECK_EQ(decoded_data.dynamic_distance, data.dynamic_distance);
    CHECK_EQ(decoded_data.static_distance, data.static_distance);
    CHECK_EQ(decoded_data.obj_s_ref, data.obj_s_ref);
    CHECK_EQ(decoded_data.obj_l_ref, data.obj_l_ref);
    CHECK_EQ(decoded_data.collision_s, data.collision_s);

    CHECK_EQ(decoded_data.obj_traj_point.path_point.point.x(), data.obj_traj_point.path_point.point.x());
    CHECK_EQ(decoded_data.obj_traj_point.path_point.point.y(), data.obj_traj_point.path_point.point.y());
    CHECK_EQ(decoded_data.obj_traj_point.path_point.heading, data.obj_traj_point.path_point.heading);
    CHECK_EQ(decoded_data.obj_traj_point.path_point.curvature, data.obj_traj_point.path_point.curvature);
    CHECK_EQ(decoded_data.obj_traj_point.path_point.s, data.obj_traj_point.path_point.s);
    CHECK_EQ(decoded_data.obj_traj_point.path_point.l, data.obj_traj_point.path_point.l);
    CHECK_EQ(decoded_data.obj_traj_point.v, data.obj_traj_point.v);
    CHECK_EQ(decoded_data.obj_traj_point.a, data.obj_traj_point.a);
    CHECK_EQ(decoded_data.obj_traj_point.yaw_rate, data.obj_traj_point.yaw_rate);
    CHECK_EQ(decoded_data.obj_traj_point.relative_time, data.obj_traj_point.relative_time);
  }

}



TEST(EncodeReferenceLineInfoArray, Case_01) {
  // struct ReferenceLineInfo {
  //   /// 参考线形点
  //   common::StaticVector<common::PathPoint,
  //       common::Path::kMaxPathPointNum> curve;
  //   /// 平滑后的参考线形点
  //   common::StaticVector<common::PathPoint,
  //       common::Path::kMaxPathPointNum> smooth_curve;
  // };

  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(driv_map::DrivingMapInfo::ReferenceLineInfo)*s_data_array_size;

  std::cout << "sizeof(driv_map::DrivingMapInfo::ReferenceLineInfo)="
            << sizeof(driv_map::DrivingMapInfo::ReferenceLineInfo) << " Bytes" << std::endl;

  driv_map::DrivingMapInfo::ReferenceLineInfo data_array[s_data_array_size];
  driv_map::DrivingMapInfo::ReferenceLineInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::DrivingMapInfo::ReferenceLineInfo& data = data_array[i];

    Int32_t curve_size = i+3;
    data.curve.Resize(curve_size);
    for (Int32_t j = 0; j < curve_size; ++j) {
      common::PathPoint& point = data.curve[j];
      point.point.set_x((j+1) * 1.2F);
      point.point.set_y((j+1) * 2.3F);
      point.heading = (j+1) * 3.4F;
      point.curvature = (j+1) * 4.5F;
      point.s = (j+1) * 5.6F;
      point.l = (j+1) * 6.7F;
    }

    Int32_t smooth_curve_size = i+4;
    data.smooth_curve.Resize(smooth_curve_size);
    for (Int32_t j = 0; j < smooth_curve_size; ++j) {
      common::PathPoint& point = data.smooth_curve[j];
      point.point.set_x((j+1) * 7.2F);
      point.point.set_y((j+1) * 8.3F);
      point.heading = (j+1) * 9.4F;
      point.curvature = (j+1) * 10.5F;
      point.s = (j+1) * 11.6F;
      point.l = (j+1) * 12.7F;
    }
  }


  Int32_t encoded_size = EncodeReferenceLineInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeReferenceLineInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ReferenceLineInfo& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  curve =" << std::endl;
    Int32_t curve_size = data.curve.Size();
    for (Int32_t j = 0; j < curve_size; ++j) {
      const common::PathPoint& point = data.curve[j];

      std::cout << "  {"
                << ", " << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << "}"
                << std::endl;
    }

    std::cout << "  smooth_curve =" << std::endl;
    Int32_t smooth_curve_size = data.smooth_curve.Size();
    for (Int32_t j = 0; j < smooth_curve_size; ++j) {
      const common::PathPoint& point = data.smooth_curve[j];

      std::cout << "  {"
                << ", " << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << "}"
                << std::endl;
    }
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ReferenceLineInfo& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  curve =" << std::endl;
    Int32_t curve_size = data.curve.Size();
    for (Int32_t j = 0; j < curve_size; ++j) {
      const common::PathPoint& point = data.curve[j];

      std::cout << "  {"
                << ", " << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << "}"
                << std::endl;
    }

    std::cout << "  smooth_curve =" << std::endl;
    Int32_t smooth_curve_size = data.smooth_curve.Size();
    for (Int32_t j = 0; j < smooth_curve_size; ++j) {
      const common::PathPoint& point = data.smooth_curve[j];

      std::cout << "  {"
                << ", " << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << "}"
                << std::endl;
    }
  }

  // Check result
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ReferenceLineInfo& data = data_array[i];
    const driv_map::DrivingMapInfo::ReferenceLineInfo& decoded_data = decoded_data_array[i];

    Int32_t curve_size = data.curve.Size();
    for (Int32_t j = 0; j < curve_size; ++j) {
      const common::PathPoint& point = data.curve[j];
      const common::PathPoint& decoded_point = decoded_data.curve[j];

      CHECK_EQ(decoded_point.point.x(), point.point.x());
      CHECK_EQ(decoded_point.point.y(), point.point.y());
      CHECK_EQ(decoded_point.heading, point.heading);
      CHECK_EQ(decoded_point.curvature, point.curvature);
      CHECK_EQ(decoded_point.s, point.s);
      CHECK_EQ(decoded_point.l, point.l);
    }

    Int32_t smooth_curve_size = data.smooth_curve.Size();
    for (Int32_t j = 0; j < smooth_curve_size; ++j) {
      const common::PathPoint& point = data.smooth_curve[j];
      const common::PathPoint& decoded_point = decoded_data.smooth_curve[j];

      CHECK_EQ(decoded_point.point.x(), point.point.x());
      CHECK_EQ(decoded_point.point.y(), point.point.y());
      CHECK_EQ(decoded_point.heading, point.heading);
      CHECK_EQ(decoded_point.curvature, point.curvature);
      CHECK_EQ(decoded_point.s, point.s);
      CHECK_EQ(decoded_point.l, point.l);
    }
  }
}



TEST(EncodeObstacleInfoArray, Case_01) {
  /// 障碍物信息
  // struct ObstacleInfo {
  //   // 障碍物ID
  //   Int32_t id;
  //   // 障碍物位置及包围盒
  //   common::OBBox2d obb;
  //   // Heading
  //   map_var_t heading;
  //   // 障碍物类型
  //   Int8_t type;
  //   // 是否是动态障碍物
  //   Int8_t dynamic;
  //   // 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
  //   // 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
  //   bool ignore;
  //   // 障碍物速度
  //   map_var_t v;
  //   // 与参考线之间的关联信息
  //   map_var_t s_ref;
  //   map_var_t l_ref;
  //   // 保存对动态障碍物预测的轨迹
  //   common::StaticVector<common::StaticVector<common::TrajectoryPoint,
  //       MAX_OBJ_PRED_TRAJ_POINT_NUM>, MAX_OBJ_PRED_TRAJ_NUM> pred_trajectory;
  // };

  static const Int32_t s_data_array_size = 3;
  static const Int32_t s_buff_size = sizeof(driv_map::DrivingMapInfo::ObstacleInfo)*s_data_array_size;

  std::cout << "sizeof(driv_map::DrivingMapInfo::ObstacleInfo)="
            << sizeof(driv_map::DrivingMapInfo::ObstacleInfo) << " Bytes" << std::endl;

  driv_map::DrivingMapInfo::ObstacleInfo data_array[s_data_array_size];
  driv_map::DrivingMapInfo::ObstacleInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::DrivingMapInfo::ObstacleInfo& data = data_array[i];

    data.id = i + 1;
    data.obb.set_center(common::Vec2d((i+1)*1.269F, (i+1)*2.269F));
    data.obb.set_unit_direction((i+1)*1.56F, (i+1)*2.56F);
    data.obb.set_extents(common::Vec2d((i+1)*3.89F, (i+1)*3.89F));
    data.heading = (i+1) * 3.698F;
    data.type = i + 2;
    data.dynamic = i;
    data.ignore = i;
    data.v = (i+1) * 4.598F;
    data.s_ref = (i+1) * 5.8956F;
    data.l_ref = (i+1) * 6.89742F;

    Int32_t pred_trajectory_size = (i+1) % 3;
    data.pred_trajectory.Resize(pred_trajectory_size);
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      Int32_t points_size = j + 2;
      data.pred_trajectory[j].Resize(points_size);
      for (Int32_t k = 0; k < points_size; ++k) {
        common::TrajectoryPoint& point = data.pred_trajectory[j][k];

        point.path_point.point.set_x((k+1) * 1.2F);
        point.path_point.point.set_y((k+1) * 2.3F);
        point.path_point.heading = (k+1) * 3.4F;
        point.path_point.curvature = (k+1) * 4.5F;
        point.path_point.s = (k+1) * 5.6F;
        point.path_point.l = (k+1) * 6.7F;
        point.v = (k+1) * 7.8F;
        point.a = (k+1) * 8.9F;
        point.yaw_rate = (k+1) * 9.1F;
        point.relative_time = (k+1) * 10.2F;
      }
    }
  }


  Int32_t encoded_size = EncodeObstacleInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeObstacleInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ObstacleInfo& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.obb.center().x()
              << ", " << data.obb.center().y()
              << ", " << data.obb.unit_direction_x().x()
              << ", " << data.obb.unit_direction_x().y()
              << ", " << data.obb.unit_direction_y().x()
              << ", " << data.obb.unit_direction_y().y()
              << ", " << data.obb.extents().x()
              << ", " << data.obb.extents().y()
              << ", " << data.heading
              << ", " << (Int32_t)data.type
              << ", " << (Int32_t)data.dynamic
              << ", " << data.ignore
              << ", " << data.v
              << ", " << data.s_ref
              << ", " << data.l_ref
              << "}"
              << std::endl;

    Int32_t pred_trajectory_size = data.pred_trajectory.Size();
    std::cout << "pred_trajectory_size=" << pred_trajectory_size << std::endl;
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      std::cout << "pred_trajectory[" << j << "]:" << std::endl;
      Int32_t points_size = data.pred_trajectory[j].Size();
      std::cout << "points_size=" << points_size << std::endl;
      for (Int32_t k = 0; k < points_size; ++k) {
        const common::TrajectoryPoint& point = data.pred_trajectory[j][k];

        std::cout << "point [" << k << "]:" << std::endl;

        std::cout << "  {" << point.path_point.point.x()
                  << ", " << point.path_point.point.y()
                  << ", " << point.path_point.heading
                  << ", " << point.path_point.curvature
                  << ", " << point.path_point.s
                  << ", " << point.path_point.l
                  << ", " << point.v
                  << ", " << point.a
                  << ", " << point.yaw_rate
                  << ", " << point.relative_time
                  << std::endl;
      }
    }
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ObstacleInfo& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "  {"
              << ", " << data.id
              << ", " << data.obb.center().x()
              << ", " << data.obb.center().y()
              << ", " << data.obb.unit_direction_x().x()
              << ", " << data.obb.unit_direction_x().y()
              << ", " << data.obb.unit_direction_y().x()
              << ", " << data.obb.unit_direction_y().y()
              << ", " << data.obb.extents().x()
              << ", " << data.obb.extents().y()
              << ", " << data.heading
              << ", " << (Int32_t)data.type
              << ", " << (Int32_t)data.dynamic
              << ", " << data.ignore
              << ", " << data.v
              << ", " << data.s_ref
              << ", " << data.l_ref
              << "}"
              << std::endl;

    Int32_t pred_trajectory_size = data.pred_trajectory.Size();
    std::cout << "pred_trajectory_size=" << pred_trajectory_size << std::endl;
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      std::cout << "pred_trajectory[" << j << "]:" << std::endl;
      Int32_t points_size = data.pred_trajectory[j].Size();
      std::cout << "points_size=" << points_size << std::endl;
      for (Int32_t k = 0; k < points_size; ++k) {
        const common::TrajectoryPoint& point = data.pred_trajectory[j][k];

        std::cout << "point [" << k << "]:" << std::endl;

        std::cout << "  {" << point.path_point.point.x()
                  << ", " << point.path_point.point.y()
                  << ", " << point.path_point.heading
                  << ", " << point.path_point.curvature
                  << ", " << point.path_point.s
                  << ", " << point.path_point.l
                  << ", " << point.v
                  << ", " << point.a
                  << ", " << point.yaw_rate
                  << ", " << point.relative_time
                  << std::endl;
      }
    }
  }

  // Check result
  CHECK_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo::ObstacleInfo& data = data_array[i];
    const driv_map::DrivingMapInfo::ObstacleInfo& decoded_data = decoded_data_array[i];


    EXPECT_EQ(decoded_data.id, data.id);
    EXPECT_EQ(decoded_data.obb.center().x(), data.obb.center().x());
    EXPECT_EQ(decoded_data.obb.center().y(), data.obb.center().y());
    EXPECT_EQ(decoded_data.obb.unit_direction_x().x(), data.obb.unit_direction_x().x());
    EXPECT_EQ(decoded_data.obb.unit_direction_x().y(), data.obb.unit_direction_x().y());
    EXPECT_EQ(decoded_data.obb.unit_direction_y().x(), data.obb.unit_direction_y().x());
    EXPECT_EQ(decoded_data.obb.unit_direction_y().y(), data.obb.unit_direction_y().y());
    EXPECT_EQ(decoded_data.obb.extents().x(), data.obb.extents().x());
    EXPECT_EQ(decoded_data.obb.extents().y(), data.obb.extents().y());
    EXPECT_EQ(decoded_data.heading, data.heading);
    EXPECT_EQ(decoded_data.type, data.type);
    EXPECT_EQ(decoded_data.dynamic, data.dynamic);
    EXPECT_EQ(decoded_data.ignore, data.ignore);
    EXPECT_EQ(decoded_data.v, data.v);
    EXPECT_EQ(decoded_data.s_ref, data.s_ref);
    EXPECT_EQ(decoded_data.l_ref, data.l_ref);

    Int32_t pred_trajectory_size = data.pred_trajectory.Size();
    CHECK_EQ(decoded_data.pred_trajectory.Size(), data.pred_trajectory.Size());
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      Int32_t points_size = data.pred_trajectory[j].Size();
      CHECK_EQ(decoded_data.pred_trajectory[j].Size(), data.pred_trajectory[j].Size());
      for (Int32_t k = 0; k < points_size; ++k) {
        const common::TrajectoryPoint& point = data.pred_trajectory[j][k];
        const common::TrajectoryPoint& decoded_point = decoded_data.pred_trajectory[j][k];


        EXPECT_EQ(decoded_point.path_point.point.x(), point.path_point.point.x());
        EXPECT_EQ(decoded_point.path_point.point.y(), point.path_point.point.y());
        EXPECT_EQ(decoded_point.path_point.heading, point.path_point.heading);
        EXPECT_EQ(decoded_point.path_point.curvature, point.path_point.curvature);
        EXPECT_EQ(decoded_point.path_point.s, point.path_point.s);
        EXPECT_EQ(decoded_point.path_point.l, point.path_point.l);
        EXPECT_EQ(decoded_point.v, point.v);
        EXPECT_EQ(decoded_point.a, point.a);
        EXPECT_EQ(decoded_point.yaw_rate, point.yaw_rate);
        EXPECT_EQ(decoded_point.relative_time, point.relative_time);
      }
    }
  }
}



TEST(EncodeDrivingMapInfoArray, Case_01) {
  // struct DrivingMapInfo {
  //   /// 车道上距离当前车辆位置最近的点
  //   common::PathPoint nearest_point_to_veh_on_lane;
  //   /// 内部地图信息
  //   MapInfo map;
  //   /// 当前参考线的索引
  //   Int32_t current_reference_line_index;
  //   /// 所有参考线的集合
  //   common::StaticVector<ReferenceLineInfo,
  //       MAX_REFERENCE_LINE_NUM> reference_lines;
  //   /// 道路左边界采样点
  //   common::StaticVector<common::PathPoint,
  //   common::Path::kMaxPathPointNum> left_road_boundary;
  //   /// 道路右边界采样点
  //   common::StaticVector<common::PathPoint,
  //   common::Path::kMaxPathPointNum> right_road_boundary;
  //   /// 障碍物列表
  //   common::StaticVector<ObstacleInfo, MAX_OBJECT_NUM_IN_OBJ_SPACE> obstacle_list;
  //   /// 风险区域分析结果(有风险的障碍物信息)
  //   common::StaticVector<CollisionRiskTestResult,
  //       MAX_OBJECT_NUM_IN_OBJ_SPACE> risky_obj_list;
  // };

  static const Int32_t s_data_array_size = 2;
  static const Int32_t s_buff_size = sizeof(driv_map::DrivingMapInfo)*s_data_array_size;

  std::cout << "sizeof(driv_map::DrivingMapInfo)="
            << sizeof(driv_map::DrivingMapInfo) / 1024 << " KBytes" << std::endl;

  driv_map::DrivingMapInfo data_array[s_data_array_size];
  driv_map::DrivingMapInfo decoded_data_array[s_data_array_size];

  Uint8_t data_buffer[s_buff_size] = { 0 };

  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    driv_map::DrivingMapInfo& data = data_array[i];

    data.msg_head.valid = i % 2;
    data.msg_head.sequence = i+1;
    data.msg_head.timestamp = (i+1)*100;
    data.msg_head.src_module_id = i+2;
    data.msg_head.dst_module_id = i+3;

    // 车道上距离当前车辆位置最近的点
    data.nearest_point_to_veh_on_lane.point.set_x((i+1) * 1.2F);
    data.nearest_point_to_veh_on_lane.point.set_y((i+1) * 2.3F);
    data.nearest_point_to_veh_on_lane.heading = (i+1) * 3.4F;
    data.nearest_point_to_veh_on_lane.curvature = (i+1) * 4.5F;
    data.nearest_point_to_veh_on_lane.s = (i+1) * 5.6F;
    data.nearest_point_to_veh_on_lane.l = (i+1) * 6.7F;

    // 内部地图信息
    driv_map::MapInfo& map = data.map;
    Int32_t lane_table_size = i+1;
    map.lane_table.Resize(lane_table_size);
    for (Int32_t j = 0; j < lane_table_size; ++j) {
      driv_map::LaneInfo& lane = map.lane_table[j];
      lane.lane_index = j+1;
      Int32_t central_curve_size = 4+j;
      lane.central_curve.Resize(central_curve_size);
      for (Int32_t k = 0; k < central_curve_size; ++k) {
        common::Vec2d& point = lane.central_curve[k];
        point.set_x((k+1) * 1.2F);
        point.set_y((k+1) * 1.3F);
      }
      Int32_t left_boundary_curve_size = 5+j;
      lane.left_boundary.curve.Resize(left_boundary_curve_size);
      for (Int32_t k = 0; k < left_boundary_curve_size; ++k) {
        driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[k];
        point.s = (k+1) * 2.3F;
        point.width = (k+1) * 3.3F;
        point.type = (k+1);
        point.point.set_x((k+1) * 4.2F);
        point.point.set_y((k+1) * 5.2F);
      }
      Int32_t right_boundary_curve_size = 6+j;
      lane.right_boundary.curve.Resize(right_boundary_curve_size);
      for (Int32_t k = 0; k < right_boundary_curve_size; ++k) {
        driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[k];
        point.s = (k+1) * 6.3F;
        point.width = (k+1) * 7.3F;
        point.type = (k+1);
        point.point.set_x((k+1) * 8.2F);
        point.point.set_y((k+1) * 9.3F);
      }
    }

    // 当前参考线的索引
    data.current_reference_line_index = i+1;

    // 所有参考线的集合
    Int32_t reference_lines_size = i + 2;
    data.reference_lines.Resize(reference_lines_size);
    for (Int32_t j = 0; j < reference_lines_size; ++j) {
      driv_map::DrivingMapInfo::ReferenceLineInfo& ref_line = data.reference_lines[j];

      Int32_t curve_size = j+3;
      ref_line.curve.Resize(curve_size);
      for (Int32_t k = 0; k < curve_size; ++k) {
        common::PathPoint& point = ref_line.curve[k];
        point.point.set_x((k+1) * 1.2F);
        point.point.set_y((k+1) * 2.3F);
        point.heading = (k+1) * 3.4F;
        point.curvature = (k+1) * 4.5F;
        point.s = (k+1) * 5.6F;
        point.l = (k+1) * 6.7F;
      }

      Int32_t smooth_curve_size = j+4;
      ref_line.smooth_curve.Resize(smooth_curve_size);
      for (Int32_t k = 0; k < smooth_curve_size; ++k) {
        common::PathPoint& point = ref_line.smooth_curve[k];
        point.point.set_x((k+1) * 7.2F);
        point.point.set_y((k+1) * 8.3F);
        point.heading = (k+1) * 9.4F;
        point.curvature = (k+1) * 10.5F;
        point.s = (k+1) * 11.6F;
        point.l = (k+1) * 12.7F;
      }
    }

    // 道路左边界采样点
    Int32_t left_road_boundary_size = i + 3;
    data.left_road_boundary.Resize(left_road_boundary_size);
    for (Int32_t j = 0; j < left_road_boundary_size; ++j) {
      common::PathPoint& point = data.left_road_boundary[j];

      point.point.set_x((j+1) * 1.21F);
      point.point.set_y((j+1) * 2.31F);
      point.heading = (j+1) * 3.41F;
      point.curvature = (j+1) * 4.51F;
      point.s = (j+1) * 5.61F;
      point.l = (j+1) * 6.71F;
    }

    // 道路右边界采样点
    Int32_t right_road_boundary_size = i + 4;
    data.right_road_boundary.Resize(right_road_boundary_size);
    for (Int32_t j = 0; j < right_road_boundary_size; ++j) {
      common::PathPoint& point = data.right_road_boundary[j];

      point.point.set_x((j+1) * 1.22F);
      point.point.set_y((j+1) * 2.32F);
      point.heading = (j+1) * 3.42F;
      point.curvature = (j+1) * 4.52F;
      point.s = (j+1) * 5.62F;
      point.l = (j+1) * 6.72F;
    }

    // 障碍物列表
    Int32_t obstacle_list_size = i + 5;
    data.obstacle_list.Resize(obstacle_list_size);
    for (Int32_t j = 0; j < obstacle_list_size; ++j) {
      driv_map::DrivingMapInfo::ObstacleInfo& obj = data.obstacle_list[j];

      obj.id = j + 1;
      obj.obb.set_center(common::Vec2d((j+1)*1.269F, (j+1)*2.269F));
      obj.obb.set_unit_direction((j+1)*1.56F, (j+1)*2.56F);
      obj.obb.set_extents(common::Vec2d((j+1)*3.89F, (j+1)*3.89F));
      obj.heading = (j+1) * 3.698F;
      obj.type = j + 2;
      obj.dynamic = j;
      obj.ignore = j;
      obj.v = (j+1) * 4.598F;
      obj.s_ref = (j+1) * 5.8956F;
      obj.l_ref = (j+1) * 6.89742F;

      Int32_t pred_trajectory_size = (j+1) % 3;
      obj.pred_trajectory.Resize(pred_trajectory_size);
      for (Int32_t j2 = 0; j2 < pred_trajectory_size; ++j2) {
        Int32_t points_size = j2 + 2;
        obj.pred_trajectory[j2].Resize(points_size);
        for (Int32_t k = 0; k < points_size; ++k) {
          common::TrajectoryPoint& point = obj.pred_trajectory[j2][k];

          point.path_point.point.set_x((k+1) * 1.2F);
          point.path_point.point.set_y((k+1) * 2.3F);
          point.path_point.heading = (k+1) * 3.4F;
          point.path_point.curvature = (k+1) * 4.5F;
          point.path_point.s = (k+1) * 5.6F;
          point.path_point.l = (k+1) * 6.7F;
          point.v = (k+1) * 7.8F;
          point.a = (k+1) * 8.9F;
          point.yaw_rate = (k+1) * 9.1F;
          point.relative_time = (k+1) * 10.2F;
        }
      }
    }

    // 风险区域分析结果(有风险的障碍物信息)
    Int32_t risky_obj_list_size = i + 6;
    data.risky_obj_list.Resize(risky_obj_list_size);
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      driv_map::CollisionRiskTestResult& obj = data.risky_obj_list[j];

      obj.obj_list_index = j+1;
      obj.risk_value = (j+1) * 2;
      obj.dynamic_distance = (j+1) * 10.1F;
      obj.static_distance = (j+1) * 11.2F;
      obj.obj_s_ref = (j+1) * 12.3F;
      obj.obj_l_ref = (j+1) * 13.4F;
      obj.collision_s = (j+1) * 14.5F;

      obj.obj_traj_point.path_point.point.set_x((j+1) * 1.2F);
      obj.obj_traj_point.path_point.point.set_y((j+1) * 2.3F);
      obj.obj_traj_point.path_point.heading = (j+1) * 3.4F;
      obj.obj_traj_point.path_point.curvature = (j+1) * 4.5F;
      obj.obj_traj_point.path_point.s = (j+1) * 5.6F;
      obj.obj_traj_point.path_point.l = (j+1) * 6.7F;
      obj.obj_traj_point.v = (j+1) * 7.8F;
      obj.obj_traj_point.a = (j+1) * 8.9F;
      obj.obj_traj_point.yaw_rate = (j+1) * 9.1F;
      obj.obj_traj_point.relative_time = (j+1) * 10.2F;
    }
  }


  Int32_t encoded_size = EncodeDrivingMapInfoArray(data_buffer, 0, s_buff_size, data_array, s_data_array_size);
  Int32_t decoded_size = DecodeDrivingMapInfoArray(data_buffer, 0, s_buff_size, decoded_data_array, s_data_array_size);

  std::cout << "encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size << std::endl;

  std::cout << "data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo& data = data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "msg_head = {"
              << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << "}" << std::endl;

    // 车道上距离当前车辆位置最近的点
    std::cout << "nearest_point_to_veh_on_lane = {"
              << data.nearest_point_to_veh_on_lane.point.x()
              << ", " << data.nearest_point_to_veh_on_lane.point.y()
              << ", " << data.nearest_point_to_veh_on_lane.heading
              << ", " << data.nearest_point_to_veh_on_lane.curvature
              << ", " << data.nearest_point_to_veh_on_lane.s
              << ", " << data.nearest_point_to_veh_on_lane.l
              << "}" << std::endl;

    // 内部地图信息
    std::cout << "map = {" << std::endl;
    const driv_map::MapInfo& map = data.map;
    Int32_t lane_table_size = map.lane_table.Size();
    std::cout << "lane_table_size=" << lane_table_size << std::endl;
    for (Int32_t j = 0; j < lane_table_size; ++j) {
      std::cout << "lane [" << j << "] = {" << std::endl;
      const driv_map::LaneInfo& lane = map.lane_table[j];
      std::cout << "lane_index=" << lane.lane_index << std::endl;
      Int32_t central_curve_size = lane.central_curve.Size();
      std::cout << "central_curve_size=" << central_curve_size << std::endl;
      std::cout << "central_curve=";
      for (Int32_t k = 0; k < central_curve_size; ++k) {
        const common::Vec2d& point = lane.central_curve[k];
        std::cout << "  (" << point.x() << "," << point.y() << ")";
      }
      std::cout << std::endl;
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      std::cout << "left_boundary_curve_size=" << left_boundary_curve_size << std::endl;
      std::cout << "left_boundary.curve=";
      for (Int32_t k = 0; k < left_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[k];
        std::cout << "  (" << point.s << "," << point.width << "," << point.type
                  << "," << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout  << std::endl;
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      std::cout << "right_boundary_curve_size=" << left_boundary_curve_size << std::endl;
      std::cout << "right_boundary.curve=";
      for (Int32_t k = 0; k < right_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[k];
        std::cout << "  (" << point.s << "," << point.width << "," << point.type
                  << "," << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout  << std::endl;
    }

    // 当前参考线的索引
    std::cout << "current_reference_line_index=" << data.current_reference_line_index << std::endl;

    // 所有参考线的集合
    Int32_t reference_lines_size = data.reference_lines.Size();
    std::cout << "reference_lines_size=" << reference_lines_size << std::endl;
    std::cout << "reference_lines=" << std::endl;
    for (Int32_t j = 0; j < reference_lines_size; ++j) {
      const driv_map::DrivingMapInfo::ReferenceLineInfo& ref_line = data.reference_lines[j];

      Int32_t curve_size = ref_line.curve.Size();
      std::cout << "curve_size=" << curve_size << std::endl;
      std::cout << "ref_line.curve=";
      for (Int32_t k = 0; k < curve_size; ++k) {
        const common::PathPoint& point = ref_line.curve[k];
        std::cout << "  ("
                  << point.point.x()
                  << ", " << point.point.y()
                  << ", " << point.heading
                  << ", " << point.curvature
                  << ", " << point.s
                  << ", " << point.l
                  << ")";
      }
      std::cout << std::endl;

      Int32_t smooth_curve_size = ref_line.smooth_curve.Size();
      std::cout << "smooth_curve_size=" << smooth_curve_size << std::endl;
      std::cout << "ref_line.smooth_curve=";
      for (Int32_t k = 0; k < smooth_curve_size; ++k) {
        const common::PathPoint& point = ref_line.smooth_curve[k];
        std::cout << "  ("
                  << point.point.x()
                  << ", " << point.point.y()
                  << ", " << point.heading
                  << ", " << point.curvature
                  << ", " << point.s
                  << ", " << point.l
                  << ")";
      }
      std::cout << std::endl;
    }

    // 道路左边界采样点
    Int32_t left_road_boundary_size = data.left_road_boundary.Size();
    std::cout << "left_road_boundary_size=" << left_road_boundary_size << std::endl;
    std::cout << "left_road_boundary=";
    for (Int32_t j = 0; j < left_road_boundary_size; ++j) {
      const common::PathPoint& point = data.left_road_boundary[j];
      std::cout << "  ("
                << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << ")";
    }
    std::cout << std::endl;
    // 道路右边界采样点
    Int32_t right_road_boundary_size = data.right_road_boundary.Size();
    std::cout << "right_road_boundary_size=" << right_road_boundary_size << std::endl;
    std::cout << "right_road_boundary=";
    for (Int32_t j = 0; j < right_road_boundary_size; ++j) {
      const common::PathPoint& point = data.right_road_boundary[j];
      std::cout << "  ("
                << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << ")";
    }
    std::cout << std::endl;

    // 障碍物列表
    Int32_t obstacle_list_size = data.obstacle_list.Size();
    std::cout << "obstacle_list_size=" << obstacle_list_size << std::endl;
    std::cout << "obstacle_list=" << std::endl;
    for (Int32_t j = 0; j < obstacle_list_size; ++j) {
      const driv_map::DrivingMapInfo::ObstacleInfo& obj = data.obstacle_list[j];

      std::cout << "obj [" << i << "]:" << std::endl;
      std::cout << "  {"
                << obj.id
                << ", " << obj.obb.center().x()
                << ", " << obj.obb.center().y()
                << ", " << obj.obb.unit_direction_x().x()
                << ", " << obj.obb.unit_direction_x().y()
                << ", " << obj.obb.unit_direction_y().x()
                << ", " << obj.obb.unit_direction_y().y()
                << ", " << obj.obb.extents().x()
                << ", " << obj.obb.extents().y()
                << ", " << obj.heading
                << ", " << (Int32_t)obj.type
                << ", " << (Int32_t)obj.dynamic
                << ", " << obj.ignore
                << ", " << obj.v
                << ", " << obj.s_ref
                << ", " << obj.l_ref
                << "}"
                << std::endl;

      Int32_t pred_trajectory_size = obj.pred_trajectory.Size();
      std::cout << "pred_trajectory_size=" << pred_trajectory_size << std::endl;
      for (Int32_t j2 = 0; j2 < pred_trajectory_size; ++j2) {
        std::cout << "pred_trajectory[" << j2 << "]:" << std::endl;
        Int32_t points_size = obj.pred_trajectory[j2].Size();
        std::cout << "points_size=" << points_size << std::endl;
        for (Int32_t k = 0; k < points_size; ++k) {
          const common::TrajectoryPoint& point = obj.pred_trajectory[j2][k];

          std::cout << "point [" << k << "]:" << std::endl;
          std::cout << "  {" << point.path_point.point.x()
                    << ", " << point.path_point.point.y()
                    << ", " << point.path_point.heading
                    << ", " << point.path_point.curvature
                    << ", " << point.path_point.s
                    << ", " << point.path_point.l
                    << ", " << point.v
                    << ", " << point.a
                    << ", " << point.yaw_rate
                    << ", " << point.relative_time
                    << std::endl;
        }
      }
    }

    // 风险区域分析结果(有风险的障碍物信息)
    Int32_t risky_obj_list_size = data.risky_obj_list.Size();
    std::cout << "risky_obj_list_size=" << risky_obj_list_size << std::endl;
    std::cout << "risky_obj_list=" << std::endl;
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      const driv_map::CollisionRiskTestResult& obj = data.risky_obj_list[j];

      std::cout << "obj [" << j << "]:" << std::endl;
      std::cout << "  {"
                << obj.obj_list_index
                << ", " << obj.risk_value
                << ", " << obj.dynamic_distance
                << ", " << obj.static_distance
                << ", " << obj.obj_s_ref
                << ", " << obj.obj_l_ref
                << ", " << obj.collision_s
                << ", " << obj.obj_traj_point.path_point.point.x()
                << ", " << obj.obj_traj_point.path_point.point.y()
                << ", " << obj.obj_traj_point.path_point.heading
                << ", " << obj.obj_traj_point.path_point.curvature
                << ", " << obj.obj_traj_point.path_point.s
                << ", " << obj.obj_traj_point.path_point.l
                << ", " << obj.obj_traj_point.v
                << ", " << obj.obj_traj_point.a
                << ", " << obj.obj_traj_point.yaw_rate
                << ", " << obj.obj_traj_point.relative_time
                << "}"
                << std::endl;
    }
  }

  std::cout << "decoded_data_array=" << std::endl;
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo& data = decoded_data_array[i];

    std::cout << "data [" << i << "]:" << std::endl;

    std::cout << "msg_head = {"
              << data.msg_head.valid
              << ", " << data.msg_head.sequence
              << ", " << data.msg_head.timestamp
              << ", " << data.msg_head.src_module_id
              << ", " << data.msg_head.dst_module_id
              << "}" << std::endl;

    // 车道上距离当前车辆位置最近的点
    std::cout << "nearest_point_to_veh_on_lane = {"
              << data.nearest_point_to_veh_on_lane.point.x()
              << ", " << data.nearest_point_to_veh_on_lane.point.y()
              << ", " << data.nearest_point_to_veh_on_lane.heading
              << ", " << data.nearest_point_to_veh_on_lane.curvature
              << ", " << data.nearest_point_to_veh_on_lane.s
              << ", " << data.nearest_point_to_veh_on_lane.l
              << "}" << std::endl;

    // 内部地图信息
    std::cout << "map = {" << std::endl;
    const driv_map::MapInfo& map = data.map;
    Int32_t lane_table_size = map.lane_table.Size();
    std::cout << "lane_table_size=" << lane_table_size << std::endl;
    for (Int32_t j = 0; j < lane_table_size; ++j) {
      std::cout << "lane [" << j << "] = {" << std::endl;
      const driv_map::LaneInfo& lane = map.lane_table[j];
      std::cout << "lane_index=" << lane.lane_index << std::endl;
      Int32_t central_curve_size = lane.central_curve.Size();
      std::cout << "central_curve_size=" << central_curve_size << std::endl;
      std::cout << "central_curve=";
      for (Int32_t k = 0; k < central_curve_size; ++k) {
        const common::Vec2d& point = lane.central_curve[k];
        std::cout << "  (" << point.x() << "," << point.y() << ")";
      }
      std::cout << std::endl;
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      std::cout << "left_boundary_curve_size=" << left_boundary_curve_size << std::endl;
      std::cout << "left_boundary.curve=";
      for (Int32_t k = 0; k < left_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[k];
        std::cout << "  (" << point.s << "," << point.width << "," << point.type
                  << "," << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout  << std::endl;
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      std::cout << "right_boundary_curve_size=" << left_boundary_curve_size << std::endl;
      std::cout << "right_boundary.curve=";
      for (Int32_t k = 0; k < right_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[k];
        std::cout << "  (" << point.s << "," << point.width << "," << point.type
                  << "," << point.point.x() << "," << point.point.y() << ")";
      }
      std::cout  << std::endl;
    }

    // 当前参考线的索引
    std::cout << "current_reference_line_index=" << data.current_reference_line_index << std::endl;

    // 所有参考线的集合
    Int32_t reference_lines_size = data.reference_lines.Size();
    std::cout << "reference_lines_size=" << reference_lines_size << std::endl;
    std::cout << "reference_lines=" << std::endl;
    for (Int32_t j = 0; j < reference_lines_size; ++j) {
      const driv_map::DrivingMapInfo::ReferenceLineInfo& ref_line = data.reference_lines[j];

      Int32_t curve_size = ref_line.curve.Size();
      std::cout << "curve_size=" << curve_size << std::endl;
      std::cout << "ref_line.curve=";
      for (Int32_t k = 0; k < curve_size; ++k) {
        const common::PathPoint& point = ref_line.curve[k];
        std::cout << "  ("
                  << point.point.x()
                  << ", " << point.point.y()
                  << ", " << point.heading
                  << ", " << point.curvature
                  << ", " << point.s
                  << ", " << point.l
                  << ")";
      }
      std::cout << std::endl;

      Int32_t smooth_curve_size = ref_line.smooth_curve.Size();
      std::cout << "smooth_curve_size=" << smooth_curve_size << std::endl;
      std::cout << "ref_line.smooth_curve=";
      for (Int32_t k = 0; k < smooth_curve_size; ++k) {
        const common::PathPoint& point = ref_line.smooth_curve[k];
        std::cout << "  ("
                  << point.point.x()
                  << ", " << point.point.y()
                  << ", " << point.heading
                  << ", " << point.curvature
                  << ", " << point.s
                  << ", " << point.l
                  << ")";
      }
      std::cout << std::endl;
    }

    // 道路左边界采样点
    Int32_t left_road_boundary_size = data.left_road_boundary.Size();
    std::cout << "left_road_boundary_size=" << left_road_boundary_size << std::endl;
    std::cout << "left_road_boundary=";
    for (Int32_t j = 0; j < left_road_boundary_size; ++j) {
      const common::PathPoint& point = data.left_road_boundary[j];
      std::cout << "  ("
                << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << ")";
    }
    std::cout << std::endl;
    // 道路右边界采样点
    Int32_t right_road_boundary_size = data.right_road_boundary.Size();
    std::cout << "right_road_boundary_size=" << right_road_boundary_size << std::endl;
    std::cout << "right_road_boundary=";
    for (Int32_t j = 0; j < right_road_boundary_size; ++j) {
      const common::PathPoint& point = data.right_road_boundary[j];
      std::cout << "  ("
                << point.point.x()
                << ", " << point.point.y()
                << ", " << point.heading
                << ", " << point.curvature
                << ", " << point.s
                << ", " << point.l
                << ")";
    }
    std::cout << std::endl;

    // 障碍物列表
    Int32_t obstacle_list_size = data.obstacle_list.Size();
    std::cout << "obstacle_list_size=" << obstacle_list_size << std::endl;
    std::cout << "obstacle_list=" << std::endl;
    for (Int32_t j = 0; j < obstacle_list_size; ++j) {
      const driv_map::DrivingMapInfo::ObstacleInfo& obj = data.obstacle_list[j];

      std::cout << "obj [" << i << "]:" << std::endl;
      std::cout << "  {"
                << obj.id
                << ", " << obj.obb.center().x()
                << ", " << obj.obb.center().y()
                << ", " << obj.obb.unit_direction_x().x()
                << ", " << obj.obb.unit_direction_x().y()
                << ", " << obj.obb.unit_direction_y().x()
                << ", " << obj.obb.unit_direction_y().y()
                << ", " << obj.obb.extents().x()
                << ", " << obj.obb.extents().y()
                << ", " << obj.heading
                << ", " << (Int32_t)obj.type
                << ", " << (Int32_t)obj.dynamic
                << ", " << obj.ignore
                << ", " << obj.v
                << ", " << obj.s_ref
                << ", " << obj.l_ref
                << "}"
                << std::endl;

      Int32_t pred_trajectory_size = obj.pred_trajectory.Size();
      std::cout << "pred_trajectory_size=" << pred_trajectory_size << std::endl;
      for (Int32_t j2 = 0; j2 < pred_trajectory_size; ++j2) {
        std::cout << "pred_trajectory[" << j2 << "]:" << std::endl;
        Int32_t points_size = obj.pred_trajectory[j2].Size();
        std::cout << "points_size=" << points_size << std::endl;
        for (Int32_t k = 0; k < points_size; ++k) {
          const common::TrajectoryPoint& point = obj.pred_trajectory[j2][k];

          std::cout << "point [" << k << "]:" << std::endl;
          std::cout << "  {" << point.path_point.point.x()
                    << ", " << point.path_point.point.y()
                    << ", " << point.path_point.heading
                    << ", " << point.path_point.curvature
                    << ", " << point.path_point.s
                    << ", " << point.path_point.l
                    << ", " << point.v
                    << ", " << point.a
                    << ", " << point.yaw_rate
                    << ", " << point.relative_time
                    << std::endl;
        }
      }
    }

    // 风险区域分析结果(有风险的障碍物信息)
    Int32_t risky_obj_list_size = data.risky_obj_list.Size();
    std::cout << "risky_obj_list_size=" << risky_obj_list_size << std::endl;
    std::cout << "risky_obj_list=" << std::endl;
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      const driv_map::CollisionRiskTestResult& obj = data.risky_obj_list[j];

      std::cout << "obj [" << j << "]:" << std::endl;
      std::cout << "  {"
                << obj.obj_list_index
                << ", " << obj.risk_value
                << ", " << obj.dynamic_distance
                << ", " << obj.static_distance
                << ", " << obj.obj_s_ref
                << ", " << obj.obj_l_ref
                << ", " << obj.collision_s
                << ", " << obj.obj_traj_point.path_point.point.x()
                << ", " << obj.obj_traj_point.path_point.point.y()
                << ", " << obj.obj_traj_point.path_point.heading
                << ", " << obj.obj_traj_point.path_point.curvature
                << ", " << obj.obj_traj_point.path_point.s
                << ", " << obj.obj_traj_point.path_point.l
                << ", " << obj.obj_traj_point.v
                << ", " << obj.obj_traj_point.a
                << ", " << obj.obj_traj_point.yaw_rate
                << ", " << obj.obj_traj_point.relative_time
                << "}"
                << std::endl;
    }
  }


  // Check result
  CHECK_EQ(encoded_size, decoded_size);
  for (Int32_t i = 0; i < s_data_array_size; ++i) {
    const driv_map::DrivingMapInfo& data = data_array[i];
    const driv_map::DrivingMapInfo& decoded_data = decoded_data_array[i];

    EXPECT_EQ(decoded_data.msg_head.valid, data.msg_head.valid);
    EXPECT_EQ(decoded_data.msg_head.sequence, data.msg_head.sequence);
    EXPECT_EQ(decoded_data.msg_head.timestamp, data.msg_head.timestamp);
    EXPECT_EQ(decoded_data.msg_head.src_module_id, data.msg_head.src_module_id);
    EXPECT_EQ(decoded_data.msg_head.dst_module_id, data.msg_head.dst_module_id);

    // 车道上距离当前车辆位置最近的点
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.point.x(), data.nearest_point_to_veh_on_lane.point.x());
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.point.y(), data.nearest_point_to_veh_on_lane.point.y());
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.heading, data.nearest_point_to_veh_on_lane.heading);
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.curvature, data.nearest_point_to_veh_on_lane.curvature);
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.s, data.nearest_point_to_veh_on_lane.s);
    EXPECT_EQ(decoded_data.nearest_point_to_veh_on_lane.l, data.nearest_point_to_veh_on_lane.l);

    // 内部地图信息
    const driv_map::MapInfo& map = data.map;
    const driv_map::MapInfo& decoded_map = decoded_data.map;
    Int32_t lane_table_size = map.lane_table.Size();
    CHECK_EQ(decoded_data.map.lane_table.Size(), data.map.lane_table.Size());
    for (Int32_t j = 0; j < lane_table_size; ++j) {
      const driv_map::LaneInfo& lane = map.lane_table[j];
      const driv_map::LaneInfo& decoded_lane = decoded_map.lane_table[j];
      EXPECT_EQ(decoded_lane.lane_index, lane.lane_index);

      Int32_t central_curve_size = lane.central_curve.Size();
      CHECK_EQ(decoded_lane.central_curve.Size(), lane.central_curve.Size());
      for (Int32_t k = 0; k < central_curve_size; ++k) {
        const common::Vec2d& point = lane.central_curve[k];
        const common::Vec2d& decoded_point = decoded_lane.central_curve[k];
        EXPECT_EQ(decoded_point.x(), point.x());
        EXPECT_EQ(decoded_point.y(), point.y());
      }
      Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
      CHECK_EQ(decoded_lane.left_boundary.curve.Size(), lane.left_boundary.curve.Size());
      for (Int32_t k = 0; k < left_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.left_boundary.curve[k];
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
            decoded_lane.left_boundary.curve[k];
        EXPECT_EQ(decoded_point.s, point.s);
        EXPECT_EQ(decoded_point.width, point.width);
        EXPECT_EQ(decoded_point.type, point.type);
        EXPECT_EQ(decoded_point.point.x(), point.point.x());
        EXPECT_EQ(decoded_point.point.y(), point.point.y());
      }
      Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
      CHECK_EQ(decoded_lane.right_boundary.curve.Size(), lane.right_boundary.curve.Size());
      for (Int32_t k = 0; k < right_boundary_curve_size; ++k) {
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& point =
            lane.right_boundary.curve[k];
        const driv_map::LaneInfo::Boundary::BoundaryAssociation& decoded_point =
            decoded_lane.right_boundary.curve[k];
        EXPECT_EQ(decoded_point.s, point.s);
        EXPECT_EQ(decoded_point.width, point.width);
        EXPECT_EQ(decoded_point.type, point.type);
        EXPECT_EQ(decoded_point.point.x(), point.point.x());
        EXPECT_EQ(decoded_point.point.y(), point.point.y());
      }
    }

    // 当前参考线的索引
    EXPECT_EQ(decoded_data.current_reference_line_index, data.current_reference_line_index);

    // 所有参考线的集合
    Int32_t reference_lines_size = data.reference_lines.Size();
    CHECK_EQ(decoded_data.reference_lines.Size(), data.reference_lines.Size());
    for (Int32_t j = 0; j < reference_lines_size; ++j) {
      const driv_map::DrivingMapInfo::ReferenceLineInfo& ref_line = data.reference_lines[j];
      const driv_map::DrivingMapInfo::ReferenceLineInfo& decoded_ref_line = decoded_data.reference_lines[j];

      Int32_t curve_size = ref_line.curve.Size();
      CHECK_EQ(decoded_ref_line.curve.Size(), ref_line.curve.Size());
      for (Int32_t k = 0; k < curve_size; ++k) {
        const common::PathPoint& point = ref_line.curve[k];
        const common::PathPoint& decoded_point = decoded_ref_line.curve[k];
        EXPECT_EQ(decoded_point.point.x(), point.point.x());
        EXPECT_EQ(decoded_point.point.y(), point.point.y());
        EXPECT_EQ(decoded_point.heading, point.heading);
        EXPECT_EQ(decoded_point.curvature, point.curvature);
        EXPECT_EQ(decoded_point.s, point.s);
        EXPECT_EQ(decoded_point.l, point.l);
      }

      Int32_t smooth_curve_size = ref_line.smooth_curve.Size();
      CHECK_EQ(decoded_ref_line.smooth_curve.Size(), ref_line.smooth_curve.Size());
      for (Int32_t k = 0; k < smooth_curve_size; ++k) {
        const common::PathPoint& point = ref_line.smooth_curve[k];
        const common::PathPoint& decoded_point = decoded_ref_line.smooth_curve[k];
        EXPECT_EQ(decoded_point.point.x(), point.point.x());
        EXPECT_EQ(decoded_point.point.y(), point.point.y());
        EXPECT_EQ(decoded_point.heading, point.heading);
        EXPECT_EQ(decoded_point.curvature, point.curvature);
        EXPECT_EQ(decoded_point.s, point.s);
        EXPECT_EQ(decoded_point.l, point.l);
      }
    }

    // 道路左边界采样点
    Int32_t left_road_boundary_size = data.left_road_boundary.Size();
    CHECK_EQ(decoded_data.left_road_boundary.Size(), data.left_road_boundary.Size());
    for (Int32_t j = 0; j < left_road_boundary_size; ++j) {
      const common::PathPoint& point = data.left_road_boundary[j];
      const common::PathPoint& decoded_point = decoded_data.left_road_boundary[j];
      EXPECT_EQ(decoded_point.point.x(), point.point.x());
      EXPECT_EQ(decoded_point.point.y(), point.point.y());
      EXPECT_EQ(decoded_point.heading, point.heading);
      EXPECT_EQ(decoded_point.curvature, point.curvature);
      EXPECT_EQ(decoded_point.s, point.s);
      EXPECT_EQ(decoded_point.l, point.l);
    }

    // 道路右边界采样点
    Int32_t right_road_boundary_size = data.right_road_boundary.Size();
    CHECK_EQ(decoded_data.right_road_boundary.Size(), data.right_road_boundary.Size());
    for (Int32_t j = 0; j < right_road_boundary_size; ++j) {
      const common::PathPoint& point = data.right_road_boundary[j];
      const common::PathPoint& decoded_point = decoded_data.right_road_boundary[j];
      EXPECT_EQ(decoded_point.point.x(), point.point.x());
      EXPECT_EQ(decoded_point.point.y(), point.point.y());
      EXPECT_EQ(decoded_point.heading, point.heading);
      EXPECT_EQ(decoded_point.curvature, point.curvature);
      EXPECT_EQ(decoded_point.s, point.s);
      EXPECT_EQ(decoded_point.l, point.l);
    }

    // 障碍物列表
    Int32_t obstacle_list_size = data.obstacle_list.Size();
    CHECK_EQ(decoded_data.obstacle_list.Size(), data.obstacle_list.Size());
    for (Int32_t j = 0; j < obstacle_list_size; ++j) {
      const driv_map::DrivingMapInfo::ObstacleInfo& obj = data.obstacle_list[j];
      const driv_map::DrivingMapInfo::ObstacleInfo& decoded_obj = decoded_data.obstacle_list[j];

      EXPECT_EQ(decoded_obj.id, obj.id);
      EXPECT_EQ(decoded_obj.obb.center().x(), obj.obb.center().x());
      EXPECT_EQ(decoded_obj.obb.center().y(), obj.obb.center().y());
      EXPECT_EQ(decoded_obj.obb.unit_direction_x().x(), obj.obb.unit_direction_x().x());
      EXPECT_EQ(decoded_obj.obb.unit_direction_x().y(), obj.obb.unit_direction_x().y());
      EXPECT_EQ(decoded_obj.obb.unit_direction_y().x(), obj.obb.unit_direction_y().x());
      EXPECT_EQ(decoded_obj.obb.unit_direction_y().y(), obj.obb.unit_direction_y().y());
      EXPECT_EQ(decoded_obj.obb.extents().x(), obj.obb.extents().x());
      EXPECT_EQ(decoded_obj.obb.extents().y(), obj.obb.extents().y());
      EXPECT_EQ(decoded_obj.heading, obj.heading);
      EXPECT_EQ(decoded_obj.type, obj.type);
      EXPECT_EQ(decoded_obj.dynamic, obj.dynamic);
      EXPECT_EQ(decoded_obj.ignore, obj.ignore);
      EXPECT_EQ(decoded_obj.v, obj.v);
      EXPECT_EQ(decoded_obj.s_ref, obj.s_ref);
      EXPECT_EQ(decoded_obj.l_ref, obj.l_ref);

      Int32_t pred_trajectory_size = obj.pred_trajectory.Size();
      CHECK_EQ(decoded_obj.pred_trajectory.Size(), obj.pred_trajectory.Size());
      for (Int32_t j2 = 0; j2 < pred_trajectory_size; ++j2) {
        Int32_t points_size = obj.pred_trajectory[j2].Size();
        CHECK_EQ(decoded_obj.pred_trajectory[j2].Size(), obj.pred_trajectory[j2].Size());
        for (Int32_t k = 0; k < points_size; ++k) {
          const common::TrajectoryPoint& point = obj.pred_trajectory[j2][k];
          const common::TrajectoryPoint& decoded_point = decoded_obj.pred_trajectory[j2][k];
          EXPECT_EQ(decoded_point.path_point.point.x(), point.path_point.point.x());
          EXPECT_EQ(decoded_point.path_point.point.y(), point.path_point.point.y());
          EXPECT_EQ(decoded_point.path_point.heading, point.path_point.heading);
          EXPECT_EQ(decoded_point.path_point.curvature, point.path_point.curvature);
          EXPECT_EQ(decoded_point.path_point.s, point.path_point.s);
          EXPECT_EQ(decoded_point.path_point.l, point.path_point.l);
          EXPECT_EQ(decoded_point.v, point.v);
          EXPECT_EQ(decoded_point.a, point.a);
          EXPECT_EQ(decoded_point.yaw_rate, point.yaw_rate);
          EXPECT_EQ(decoded_point.relative_time, point.relative_time);
        }
      }
    }

    // 风险区域分析结果(有风险的障碍物信息)
    Int32_t risky_obj_list_size = data.risky_obj_list.Size();
    CHECK_EQ(decoded_data.risky_obj_list.Size(), data.risky_obj_list.Size());
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      const driv_map::CollisionRiskTestResult& obj = data.risky_obj_list[j];
      const driv_map::CollisionRiskTestResult& decoded_obj = decoded_data.risky_obj_list[j];

      EXPECT_EQ(decoded_obj.obj_list_index, obj.obj_list_index);
      EXPECT_EQ(decoded_obj.risk_value, obj.risk_value);
      EXPECT_EQ(decoded_obj.dynamic_distance, obj.dynamic_distance);
      EXPECT_EQ(decoded_obj.static_distance, obj.static_distance);
      EXPECT_EQ(decoded_obj.obj_s_ref, obj.obj_s_ref);
      EXPECT_EQ(decoded_obj.obj_l_ref, obj.obj_l_ref);
      EXPECT_EQ(decoded_obj.collision_s, obj.collision_s);

      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.point.x(), obj.obj_traj_point.path_point.point.x());
      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.point.y(), obj.obj_traj_point.path_point.point.y());
      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.heading, obj.obj_traj_point.path_point.heading);
      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.curvature, obj.obj_traj_point.path_point.curvature);
      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.s, obj.obj_traj_point.path_point.s);
      EXPECT_EQ(decoded_obj.obj_traj_point.path_point.l, obj.obj_traj_point.path_point.l);
      EXPECT_EQ(decoded_obj.obj_traj_point.v, obj.obj_traj_point.v);
      EXPECT_EQ(decoded_obj.obj_traj_point.a, obj.obj_traj_point.a);
      EXPECT_EQ(decoded_obj.obj_traj_point.yaw_rate, obj.obj_traj_point.yaw_rate);
      EXPECT_EQ(decoded_obj.obj_traj_point.relative_time, obj.obj_traj_point.relative_time);
    }
  }
}

}  // namespace data_serial
}  // namespace phoenix


#endif // #if (ENABLE_GTEST)

