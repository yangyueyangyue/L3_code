/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
#define PHOENIX_FRAMEWORK_MSG_RECEIVER_H_


#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <rosgraph_msgs/Clock.h>
#include <time.h>
#include "sensing/CanFrame.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "communication/ros_node.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"

#include "src/dev_driver/can_dev/can_driver.h"
#include "src/framework/communication/shared_data.h"

#if (ENTER_PLAYBACK_MODE)
#include "sensor/task_recv_ars_data_front.h"
#include "sensor/task_recv_maxieye_data.h"
#include "communication/shared_data.h"
#include "math/matrix.h"
#include "vehicle_model_wrapper.h"
#include "sensor/parse_sensor_data_common.h"
#endif

namespace phoenix {
namespace framework {


class MsgReceiver : public Task {
public:
  MsgReceiver(Task* manager);

#if (ENABLE_ROS_NODE)
  void SetRosNode(RosNode* node) {
    ros_node_ = node;
  }
#endif
#if (ENABLE_LCM_NODE)
  void SetLcmNode(LcmNode* node) {
    lcm_node_ = node;
  }
#endif
#if (ENABLE_UDP_NODE)
  void SetUdpNode(UdpNode* node) {
    udp_node_ = node;
  }
#endif

  bool Start();

private:
#if (ENABLE_ROS_NODE)
  void HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg);

  // 接受融合障碍物数据
  void HandleObstacleFusionListMessageRos(const std_msgs::ByteMultiArray& msg);

#if (ENTER_PLAYBACK_MODE)

  #if (ENTER_PLAYBACK_MODE_AD_HMI)
  // 接受AD_HMI的传感器数据
  void HandleADHMICanFrame0MessageRos(const sensing::CanFrame& msg);
  void HandleADHMICanFrame1MessageRos(const sensing::CanFrame& msg);
  void HandleADHMICanFrame2MessageRos(const sensing::CanFrame& msg);
  void HandleADHMICanFrame3MessageRos(const sensing::CanFrame& msg);

  void HandleADHMIObstacleFusionListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleADHMIClockMessageRos(const rosgraph_msgs::Clock& msg);
  #endif  // #if (ENTER_PLAYBACK_MODE_AD_HMI)

  // 接受大陆430毫米波原始can数据
  void HandleArsRadarCanFrameListMessageRos(const std_msgs::ByteMultiArray& msg);
  void ParseArsCanFrame(const can_dev::CanFrame &frame);

  // 接受前视一体机/视觉控制器原始can数据
  void HandleCameraCanFdFrameListMessageRos(const std_msgs::ByteMultiArray& msg);
  void ParseCameraCanFdFrame(const can_dev::CanFdFrame& frame);
  void ParseLaneFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseOneLaneFromData(const Uint8_t* data);
  void ParseLaneCurbFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseOneLaneCurbFromData(const Uint8_t* data);
  void ParseLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame);

  void ParseMaxieyeObjFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseMaxieyeOneObjFromData(const Uint8_t* data);

  void ParseTSRFromCanFrame(const can_dev::CanFdFrame& frame);

  void ParseTrafficConeFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseTwoTrafficConeFromData(const Uint8_t* data);

  // 视觉控制器
  void ParseVisualControlLaneFromCanFrame(const can_dev::CanFdFrame& frame);
  void ParseVisualControlOneLaneFromData(const Uint8_t* data);
  void ParseVisualControlLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame);

  void ParseVisualControlObjFromCanFrame(const can_dev::CanFdFrame& frame, Uint8_t camera_symbol);
  void ParseVisualControlObjFromData(const Uint8_t* data, Uint8_t camera_symbol);

  void NotifyObjList(bool obj_list_ready, Uint8_t camera_symbol);

  // 接受侧向毫米波雷达数据
  void HandleAnngicRadarCanFdFrameListMessageRos(const std_msgs::ByteMultiArray& msg);
  void ParseAnngicRadarCanFdFrame(const can_dev::CanFdFrame &frame);
  void ParseAnngicRadarObjFromCanFrame(const can_dev::CanFdFrame& frame, Uint8_t radar_symbol, Uint16_t *message_count);
  void ParseAnngicRadarOneObjFromData(const Uint8_t* data, Uint8_t radar_symbol, Uint16_t *message_count);
  void NotifyAnngicRadarObjList(bool obj_list_ready, Uint8_t radar_symbol);
  void HandleMpuStateMessageRos(const std_msgs::ByteMultiArray& msg);

#endif  // #if (ENTER_PLAYBACK_MODE)

#endif

#if (ENABLE_LCM_NODE)
#endif

private:
#if (ENABLE_ROS_NODE)
  RosNode* ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  LcmNode* lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  UdpNode* udp_node_;
#endif

#if (ENABLE_ROS_NODE)
  phoenix::can_dev::CanFrameList ros_ars_radar_can_frame_list_;   //大陆430雷达
  phoenix::can_dev::CanFdFrameList ros_camera_canfd_frame_list_;  //前视一体机和视觉控制器
  phoenix::can_dev::CanFdFrameList ros_anngic_radar_canfd_frame_list_;  //侧向毫米波雷达

  ad_msg::Chassis ros_chassis_info_;

  ad_msg::ObstacleList ros_obstacle_fusion_list_;

#if (ENTER_PLAYBACK_MODE)
  // 传感器标定参数
  common::Matrix<Float32_t, 3, 3> maxieye_mat_calibration_;
  common::Matrix<Float32_t, 3, 3> ars_mat_calibration_;
  common::Matrix<Float32_t, 3, 3> visual_control_mat_calibration_;
  common::Matrix<Float32_t, 3, 3> anngic_radar_mat_calibration_;
  ad_msg::Chassis chassis_;
  MpuState::MonitorMpuState mpu_state_;

  // 前视一体机
  bool lane_list_ready_;
  ad_msg::LaneMarkCameraList lane_list_;
  bool lane_curb_list_ready_;
  ad_msg::LaneMarkCameraList lane_curb_list_;

  bool maxieye_obj_list_ready_;
  ad_msg::ObstacleCameraList maxieye_obj_list_;

  bool traffic_signal_ready_;
  ad_msg::TrafficSignalList traffic_signal_list_;

  // 视觉控制器
  // 车道线
  bool visual_control_lane_list_ready_;
  ad_msg::LaneMarkCameraList visual_control_lane_list_;
  
  // 前摄像头
  bool visual_control_front_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_front_obj_list_;

  // 前左摄像头
  bool visual_control_front_left_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_front_left_obj_list_;

  // 前右摄像头
  bool visual_control_front_right_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_front_right_obj_list_;

  // 后左摄像头
  bool visual_control_rear_left_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_rear_left_obj_list_;

  // 后右摄像头
  bool visual_control_rear_right_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_rear_right_obj_list_;

  // 后摄像头
  bool visual_control_rear_obj_list_ready_;
  ad_msg::ObstacleCameraList visual_control_rear_obj_list_;

  // 大陆430雷达
  ad_msg::ObstacleRadarList ars_obj_list_;
  Int32_t single_obj_ready_mask_;
  Int32_t obj_can_frame_cnts_;
  bool asr_data_ready_;

  // 侧向毫米波雷达
  // Message Count
  Uint8_t front_left_can_id_count_;
  Uint8_t front_right_can_id_count_;
  Uint8_t rear_left_can_id_count_;
  Uint8_t rear_right_can_id_count_;

  // 前左雷达
  bool anngic_radar_front_left_obj_list_ready_;
  ad_msg::ObstacleRadarList anngic_radar_front_left_obj_list_;

  // 前右雷达
  bool anngic_radar_front_right_obj_list_ready_;
  ad_msg::ObstacleRadarList anngic_radar_front_right_obj_list_;

  // 后左雷达
  bool anngic_radar_rear_left_obj_list_ready_;
  ad_msg::ObstacleRadarList anngic_radar_rear_left_obj_list_;

  // 后右雷达
  bool anngic_radar_rear_right_obj_list_ready_;
  ad_msg::ObstacleRadarList anngic_radar_rear_right_obj_list_;

  ad_msg::ObstacleList ros_adhmi_obstacle_fusion_list_;
#endif // ENTER_PLAYBACK_MODE

#endif  // ENABLE_ROS_NODE
};

#if 0
static visualization_msgs::MarkerArray MaxieyeDataToRosMessage(const ad_msg::ObstacleCameraList &obstacle_list_) {
    visualization_msgs::MarkerArray marker_array;
    if(obstacle_list_.obstacle_num > 0) {
      for(uint8_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
          visualization_msgs::Marker marker;
          marker.header.frame_id="map";
          marker.header.stamp = ros::Time::now();
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.ns = "camera_obstacles";
          marker.id = obstacle_list_.obstacles[i].id;
          marker.lifetime = ros::Duration(0.034);
          if(obstacle_list_.obstacles[i].x == 0 && obstacle_list_.obstacles[i].y == 0) {
            continue;
          }
          if(obstacle_list_.obstacles[i].length == 0) {
            marker.scale.x = 0.5;
          }else {
            marker.scale.x = obstacle_list_.obstacles[i].length;
          }
          if(obstacle_list_.obstacles[i].width == 0) {
            marker.scale.y = 0.5;
          }else {
            marker.scale.y = obstacle_list_.obstacles[i].width;
          }
          if(obstacle_list_.obstacles[i].height == 0) {
            marker.scale.z = 0.5;
          }else {
            marker.scale.z = obstacle_list_.obstacles[i].height;
          }		    
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
          marker.color.a = 1;
          geometry_msgs::Pose pose;
          pose.position.x = obstacle_list_.obstacles[i].x;
          pose.position.y = obstacle_list_.obstacles[i].y;
          pose.position.z = 0;
          pose.orientation.w = 1;
          pose.orientation.x = 0;
          pose.orientation.y = 0;
          pose.orientation.z = 0;
          marker.pose = pose;
          marker_array.markers.emplace_back(marker);
      }
    }
    return marker_array;
}

static visualization_msgs::MarkerArray LaneMarkToRosMessage(const ad_msg::LaneMarkCameraList & lane_mark_camera_list) {
  visualization_msgs::MarkerArray marker_array;
  const Float32_t sample_step_len = 1.0F;
  Int32_t lane_index = -1;
  for(auto & lane_mark : lane_mark_camera_list.lane_marks) {
    if (lane_mark.quality < 2) {
      continue;
    }
    Float32_t lane_mark_len = lane_mark.view_range_end - lane_mark.view_range_start;
    if (lane_mark_len < sample_step_len) {
      continue;
    }
    Int32_t sample_size = common::com_round(lane_mark_len / sample_step_len);
    common::CubicPolynomialCurve1d<Float64_t> curve;
    curve.SetCoefficient(lane_mark.c0, lane_mark.c1, lane_mark.c2, lane_mark.c3);

    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;
    if ((1 == lane_mark.id) || (-1 == lane_mark.id)) {
      color.r = 0;
      color.g = 1;
      color.b = 1;
      color.a = 0.5;
      marker.type = visualization_msgs::Marker::LINE_LIST;
    } else {
      color.r = 0;
      color.g = 0;
      color.b = 1;
      color.a = 0.5;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
    }

    marker.header.frame_id="map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lane_points";
    marker.id = lane_index;
    marker.lifetime = ros::Duration(0.034);
    marker.color = color;
    marker.scale.x = 0.2;
    marker.scale.y = 0;
    marker.scale.z = 0; 
    marker.pose.orientation.w = 1;

    for (Int32_t j = 0; j < sample_size; ++j) {
      Float32_t x = j * sample_step_len;
      Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));
      geometry_msgs::Point point;
      point.x = x;
      point.y = y;
      point.z = 0;
      marker.points.emplace_back(point);
    }
    marker_array.markers.emplace_back(marker);
    ++lane_index;
  }
  return marker_array;
}

// Sensor Data Visualization (start)
static visualization_msgs::MarkerArray RadarDataToRosMessage(const ad_msg::ObstacleRadarList &obstacle_list_) {
  visualization_msgs::MarkerArray marker_array;
  if(obstacle_list_.obstacle_num > 0) {
    for(uint8_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id="map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.ns = "radar_obstacles";
      marker.id = obstacle_list_.obstacles[i].id;
      marker.lifetime = ros::Duration(0.071);
      if(obstacle_list_.obstacles[i].x == 0 && obstacle_list_.obstacles[i].y == 0) {
        continue;
      }
      if(obstacle_list_.obstacles[i].length == 0) {
        marker.scale.x = 0.1;
      }else {
        marker.scale.x = obstacle_list_.obstacles[i].length;
      }
      if(obstacle_list_.obstacles[i].width == 0) {
        marker.scale.y = 0.1;
      }else {
        marker.scale.y = obstacle_list_.obstacles[i].width;
      }
      marker.scale.z = 0.2;
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;
      geometry_msgs::Pose pose;
      pose.position.x = obstacle_list_.obstacles[i].x;
      pose.position.y = obstacle_list_.obstacles[i].y;
      pose.position.z = 0;
      pose.orientation.w = 1;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      marker.pose = pose;
      marker_array.markers.emplace_back(marker);
    }
  }
  return marker_array;
}
// Sensor Data Visualization (end)

static visualization_msgs::MarkerArray VisualControlToRosMessage(const ad_msg::ObstacleCameraList &obstacle_list_) {
    visualization_msgs::MarkerArray marker_array;
    if(obstacle_list_.obstacle_num > 0) {
      for(uint8_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
          visualization_msgs::Marker marker;
          marker.header.frame_id="map";
          marker.header.stamp = ros::Time::now();
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.ns = "visualcontrol_obstacles";
          marker.id = obstacle_list_.obstacles[i].id;
          marker.lifetime = ros::Duration(0.034);//duration可能要调整的
          if(obstacle_list_.obstacles[i].x == 0 && obstacle_list_.obstacles[i].y == 0) {
            continue;
          }
          if(obstacle_list_.obstacles[i].length == 0) {
            marker.scale.x = 0.5;
          }else {
            marker.scale.x = obstacle_list_.obstacles[i].length;
          }
          if(obstacle_list_.obstacles[i].width == 0) {
            marker.scale.y = 0.5;
          }else {
            marker.scale.y = obstacle_list_.obstacles[i].width;
          }
          if(obstacle_list_.obstacles[i].height == 0) {
            marker.scale.z = 0.5;
          }else {
            marker.scale.z = obstacle_list_.obstacles[i].height;
          }		    
          marker.color.r = 1;//视觉控制器目标物显示为粉色
          marker.color.g = 0.75;
          marker.color.b = 0.79;
          marker.color.a = 1;
          geometry_msgs::Pose pose;
          pose.position.x = obstacle_list_.obstacles[i].x;
          pose.position.y = obstacle_list_.obstacles[i].y;
          pose.position.z = 0;
          pose.orientation.w = 1;
          pose.orientation.x = 0;
          pose.orientation.y = 0;
          pose.orientation.z = 0;
          marker.pose = pose;
          marker_array.markers.emplace_back(marker);
      }
    }
    return marker_array;
}

static visualization_msgs::MarkerArray AnngicRadarToRosMessage(const ad_msg::ObstacleRadarList &obstacle_list_) {
  visualization_msgs::MarkerArray marker_array;
  if(obstacle_list_.obstacle_num > 0) {
    for(uint8_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id="map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.ns = "anngicradar_obstacles";
      marker.id = obstacle_list_.obstacles[i].id;
      marker.lifetime = ros::Duration(0.071);
      if(obstacle_list_.obstacles[i].x == 0 && obstacle_list_.obstacles[i].y == 0) {
        continue;
      }
      if(obstacle_list_.obstacles[i].length == 0) {
        marker.scale.x = 0.1;
      }else {
        marker.scale.x = obstacle_list_.obstacles[i].length;
      }
      if(obstacle_list_.obstacles[i].width == 0) {
        marker.scale.y = 0.1;
      }else {
        marker.scale.y = obstacle_list_.obstacles[i].width;
      }
      marker.scale.z = 0.2;
      marker.color.r = 1;//侧向毫米波雷达目标物显示为黄色
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;
      geometry_msgs::Pose pose;
      pose.position.x = obstacle_list_.obstacles[i].x;
      pose.position.y = obstacle_list_.obstacles[i].y;
      pose.position.z = 0;
      pose.orientation.w = 1;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      marker.pose = pose;
      marker_array.markers.emplace_back(marker);
    }
  }
  return marker_array;
}
#endif

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
