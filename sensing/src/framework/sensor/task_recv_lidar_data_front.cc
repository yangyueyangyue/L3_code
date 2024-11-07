//
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "sensor/task_recv_lidar_data_front.h"
#include "communication/shared_data.h"
#include "parse_sensor_data_common.h"

#define LIDAR_CANFRAME_LOG_FLAG 0

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {


TaskRecvLidarDataFront::TaskRecvLidarDataFront(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_LIDAR_DATA_FRONT, "Recv Lidar Data", manager) {
  running_flag_recv_ = false;

  can_channel_ = Nullptr_t;

#if (ENABLE_CAN_DEV_ZLGCANNET)
  can_channel_ = new can_dev::CanDriverZlgCanNetFd();
#endif

  veh_model::VehicleModelWrapper veh_model;
  const veh_model::SensorOnVehicle& sensor_param = veh_model.GetSensor();
  const veh_model::SensorOnVehicle::Calibration& calibration =
      sensor_param.calibration_lidar[0];
  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_calibration_.SetIdentity();
  common::Rotate_2D<Float32_t>(
        rotate_center, calibration.yaw_offset, &mat_calibration_);
  common::Translate_2D(
        calibration.x_offset, calibration.y_offset, &mat_calibration_);

  obj_list_ready_ = false;
  obj_list_.Clear();
}

TaskRecvLidarDataFront::~TaskRecvLidarDataFront() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvLidarDataFront::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 3;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 8003;

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel " << can_channel_param.channel;
    return false;
  }

  obj_list_ready_ = false;
  obj_list_.Clear();

  LOG_INFO(3) << "Create thread of receiving lidar front data...";
  running_flag_recv_ = true;
  thread_recv_ = boost::thread(boost::bind(&TaskRecvLidarDataFront::ThreadReceiving, this));

  LOG_INFO(3) << "Create thread of receiving lidar front data... [OK]";

  return (true);
}

bool TaskRecvLidarDataFront::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;

    LOG_INFO(3) << "Stop thread of receiving lidar front data ...";
    bool ret = thread_recv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving lidar front data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving lidar front data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving lidar front data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvLidarDataFront::ThreadReceiving() {
  LOG_INFO(3) << "Thread of receiving lidar front data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFdFrame can_frame[40];
  while (running_flag_recv_) {
    // 从共享空间中获取车道线CanFd数据，并发送给激光雷达
    static phoenix::can_dev::CanFdFrame LaneCanFdData;
    static Int64_t current_timestamp;
    static Int64_t last_timestamp;
    phoenix::framework::SharedData* shared_data = phoenix::framework::SharedData::instance();
    shared_data->GetLaneCanFdData(&LaneCanFdData);
    current_timestamp = LaneCanFdData.time_stamp;
    if (current_timestamp != last_timestamp) {
      last_timestamp = current_timestamp;
      LaneCanFdData.id = 0x19D0871C;
      LaneCanFdData.data_len = 64;
      LaneCanFdData.FD = 1;
      LaneCanFdData.RTR = 0;
      LaneCanFdData.BRS = 0;
      can_channel_->SendCanFd(&LaneCanFdData);

#if 0
      for(int i = 0; i< 64; i= i + 8){
        printf("data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
              LaneCanFdData.data[i], LaneCanFdData.data[i+1], LaneCanFdData.data[i+2], LaneCanFdData.data[i+3],
            LaneCanFdData.data[i+4], LaneCanFdData.data[i+5], LaneCanFdData.data[i+6], LaneCanFdData.data[i+7]);
      }
#endif
      memset(&LaneCanFdData, 0, sizeof(phoenix::can_dev::CanFdFrame));
    }


    // 获取障碍物数据并解析
    frame_num = can_channel_->ReadCanFdWait(&(can_frame[0]), 40, 50);

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFdFrame& fm = can_frame[i];

#if (1 == LIDAR_CANFRAME_LOG_FLAG)
        printf("lidar front: id[%08X] data\n", fm.id);
        for(int i = 0; i< 64; i= i + 8){
          printf("data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                 fm.data[i], fm.data[i+1], fm.data[i+2], fm.data[i+3],
              fm.data[i+4], fm.data[i+5], fm.data[i+6], fm.data[i+7]);
        }
#endif

        FullCanFdFrame(fm);
        ParseCanFrame(fm);
      }

      // 发送原始CanFdFrame数据
      framework::MessageCanFdFrameList message(framework::MSG_ID_RECV_LIDAR_CANFD_FRAME, &canfd_frame_list);
      Notify(message);

      canfd_frame_list.Clear();

      if (obj_list_ready_) {
        ad_msg::ObstacleLidarList& obstacle_list = obj_list_.obstacle_list_;

        obstacle_list.lidar_type = ad_msg::ObstacleLidarList::LIDAR_TYPE_UNKNOWN;
        // Convert to vehicle coordinate.
        framework::SharedData::instance()->GetChassis(&chassis_);
        common::Matrix<Float32_t, 2, 1> point_conv;
        for (Int32_t j = 0; j < obstacle_list.obstacle_num; ++j) {
          ad_msg::ObstacleLidar& obj = obstacle_list.obstacles[j];

          point_conv(0) = obj.x;
          point_conv(1) = obj.y;
          common::TransformVert_2D(mat_calibration_, &point_conv);
          obj.x = point_conv(0);
          obj.y = point_conv(1);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify Obstacle Lidar Front List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvLidarObjects message(framework::MSG_ID_RECV_LIDAR_OBJECTS_FRONT, &obj_list_.GetObjList());
        Notify(message);

        obj_list_ready_ = false;
        obj_list_.Clear();
      }
    }
  }

  LOG_INFO(3) << "Thread of receiving mobileye data ... [Stopped]";
}

void TaskRecvLidarDataFront::FullCanFdFrame(const can_dev::CanFdFrame& frame) {
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].time_stamp = frame.time_stamp;

  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].id = frame.id;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].FD = frame.FD;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].RTR = frame.RTR;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].EXT = frame.EXT;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].BRS = frame.BRS;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].ESI = frame.ESI;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].data_len = frame.data_len;
  for(Uint8_t i = 0; i < frame.data_len; i++) {
    canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].data[i] = frame.data[i];
  }

  canfd_frame_list.canfd_frame_num++;
}

void TaskRecvLidarDataFront::ParseCanFrame(const can_dev::CanFdFrame &frame) {
  static Int32_t frame_num = 0;
  if(0x19D0219F <= frame.id && frame.id <= 0x19D0489F) {
    if(frame.id == 0x19D0219F) {
      if (frame_num > 0) {
        obj_list_ready_ = true;
        frame_num = 0;
      }
    }

    ParseObjFromCanFrame(frame);

    frame_num++;
  }
}

void TaskRecvLidarDataFront::ParseObjFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // 包引索号
  Uint32_t PackageNum = parse_sensor_data(frame.data, 0, 0, 32);

  // 目标编号
  Uint8_t ObjectNum = parse_sensor_data(frame.data, 4, 32, 8);

  // 目标横向距离(m)
  Float32_t Obj_Lateral_Dist = parse_sensor_data(frame.data, 5, 40, 16) * 0.01 + (-300);
  if(0 == parse_sensor_data(frame.data, 5, 40, 16)) {
    return;
  }

  // 目标纵向距离(m)
  Float32_t Obj_Longitudinal_Dist = parse_sensor_data(frame.data, 7, 56, 16) * 0.01 + (-300);
  if (0 == parse_sensor_data(frame.data, 7, 56, 16)) {
    return;
  }

  // 包围盒的中心点坐标x(m)
  Float32_t Bounding_X_Coordinate = parse_sensor_data(frame.data, 9, 72, 16) * 0.01 + (-300);
  if (0 == parse_sensor_data(frame.data, 9, 72, 16)) {
    return;
  }

  // 包围盒的中心点坐标y(m)
  Float32_t Bounding_Y_Coordinate = parse_sensor_data(frame.data, 11, 88, 16) * 0.01 + (-300);
  if (0 == parse_sensor_data(frame.data, 11, 88, 16)) {
    return;
  }

  // 包围盒的中心点坐标z(m)
  Float32_t Bounding_Z_Coordinate = parse_sensor_data(frame.data, 13, 104, 16) * 0.01 + (-300);
  if (0 == parse_sensor_data(frame.data, 13, 104, 16)) {
    return;
  }

  // 包围盒维度-长(m)
  Float32_t Bounding_Length = parse_sensor_data(frame.data, 15, 120, 16) * 0.01;

  // 包围盒维度-宽(m)
  Float32_t Bounding_Width = parse_sensor_data(frame.data, 17, 136, 16) * 0.01;

  // 包围盒维度-高(m)
  Float32_t Bounding_Height = parse_sensor_data(frame.data, 19, 152, 16) * 0.01;

  // 包围盒朝向-绕z轴(deg)
  Float32_t Bounding_Orientation_Z = parse_sensor_data(frame.data, 21, 168, 16) * 0.01 + (-180);

  // 目标横向相对速度(m/s)
  Float32_t Obj_Lateral_Rel_Vel = parse_sensor_data(frame.data, 23, 184, 16) * 0.01 + (-100);

  // 目标纵向相对速度(m/s)
  Float32_t Obj_Longitudinal_Rel_Vel = parse_sensor_data(frame.data, 25, 200, 16) * 0.01 + (-100);
  
  // 目标横向加速度(m/s2)
  Float32_t Obj_Lateral_Acc = parse_sensor_data(frame.data, 27, 216, 16) * 0.01 + (-20);

  // 目标纵向加速度(m/s2)
  Float32_t Obj_Longitudinal_Acc = parse_sensor_data(frame.data, 29, 232, 16) * 0.01 + (-20);

  // 目标运动类型(0-静止 1-运动)
  Uint8_t Obj_Motion_Type = parse_sensor_data(frame.data, 31, 248, 8);

  // 置信(%)
  Float32_t Confidence = parse_sensor_data(frame.data, 32, 264, 8) * 0.1;

  // 雷达状态
  Uint8_t Radar_Status = parse_sensor_data(frame.data, 33, 280, 8);

  // 目标类别
  Uint8_t Obj_Type = parse_sensor_data(frame.data, 34, 296, 8);

    printf("--------------------------------------\n");
    printf("Obj_Lateral_Dist:%f\n", Obj_Lateral_Dist);
    printf("Obj_Longitudinal_Dist:%f\n", Obj_Longitudinal_Dist);
    printf("Bounding_X_Coordinate:%f\n", Bounding_X_Coordinate);
    printf("Bounding_Y_Coordinate:%f\n", Bounding_Y_Coordinate);
    printf("Bounding_Z_Coordinate:%f\n", Bounding_Z_Coordinate);
    printf("Bounding_Length:%f\n", Bounding_Length);
    printf("Bounding_Width:%f\n", Bounding_Width);
    printf("Bounding_Height:%f\n", Bounding_Height);
    printf("----------------------------\n");

  // Get back buffer from list
  ad_msg::ObstacleLidar* obj = obj_list_.Back();
  if (Nullptr_t == obj) {
    LOG_ERR << "Obj list is full.";
    return;
  }
  
  // 障碍物ID
  obj->id = ObjectNum;

  // 障碍物的类型
  switch (Obj_Type) {
    case (0):
      obj->type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case (1):
      obj->type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case (2):
      obj->type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case (3):
      obj->type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case (4):
      obj->type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case (5):
      obj->type = ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case (6):
      obj->type = ad_msg::OBJ_TYPE_DISCARD;
      break;
    case (7):
      obj->type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    default:
      obj->type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
  }

  // Number of scans this object has been tracked for.
  obj->age = 0;

  // Number of scans this object has currently been predicted for without measurement update
  obj->prediction_age = 0;

  // 障碍物位置x坐标 (m)
  obj->x = Obj_Longitudinal_Dist; //Bounding_X_Coordinate - (Bounding_Width / 2);

  /// 障碍物位置y坐标 (m)
  obj->y = Obj_Lateral_Dist;  //Bounding_Y_Coordinate - (Bounding_Length / 2);  

  // 障碍物包围盒
  obj->obb.x = Bounding_X_Coordinate;  // 中心坐标x
  obj->obb.y = Bounding_Y_Coordinate;  // 中心坐标y
  obj->obb.heading = Bounding_Orientation_Z; // 航向角
  obj->obb.half_length = Bounding_Length / 2;  // 半长
  obj->obb.half_width = Bounding_Width / 2;  // 半宽

  /// 障碍物的x向速度 (m/sec)
  obj->v_x = Obj_Longitudinal_Rel_Vel;

  /// 障碍物的y向速度 (m/sec)
  obj->v_y = Obj_Lateral_Rel_Vel;

  /// 障碍物的x向加速度 (m/sec^2)
  obj->accel_x = Obj_Longitudinal_Acc;

  /// 障碍物的y向加速度 (m/sec^2)
  obj->accel_y = Obj_Lateral_Acc;

  /// 障碍物的角速度 (rad/sec)
  obj->yaw_rate = 0;

  // Push back to list
  obj_list_.PushBack();
}

} // namespace sensor
} // namespace phoenix

