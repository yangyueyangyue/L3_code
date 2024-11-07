//
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "sensor/task_recv_anngic_radar_data.h"
#include "communication/shared_data.h"
#include "parse_sensor_data_common.h"
#include "sensor/parse_sensor_data_common.h"

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {


TaskRecvAnngicRadarData::TaskRecvAnngicRadarData(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_ANNGIC_DATA, "Recv Anngic Data", manager) {
  running_flag_recv_ = false;

  can_channel_ = Nullptr_t;

#if (ENABLE_CAN_DEV_ZLGCANNET)
  can_channel_ = new can_dev::CanDriverZlgCanNetFd();
#endif

  // 侧向雷达标定参数
  veh_model::VehicleModelWrapper veh_model;
  const veh_model::SensorOnVehicle& sensor_param = veh_model.GetSensor();
  const veh_model::SensorOnVehicle::Calibration& calibration =
      sensor_param.calibration_radar[1];
  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_calibration_.SetIdentity();
  common::Rotate_2D<Float32_t>(
        rotate_center, calibration.yaw_offset, &mat_calibration_);
  common::Translate_2D(
        calibration.x_offset, calibration.y_offset, &mat_calibration_);

  // Message Count
  front_left_can_id_count_ = 0;
  front_right_can_id_count_ = 0;
  rear_left_can_id_count_ = 0;
  rear_right_can_id_count_ = 0;

  front_left_obj_list_ready_ = false;
  front_left_obj_list_.Clear();

  front_right_obj_list_ready_ = false;
  front_right_obj_list_.Clear();

  rear_left_obj_list_ready_ = false;
  rear_left_obj_list_.Clear();

  rear_right_obj_list_ready_ = false;
  rear_right_obj_list_.Clear();
}

TaskRecvAnngicRadarData::~TaskRecvAnngicRadarData() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvAnngicRadarData::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 1;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 8001;

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel " << can_channel_param.channel;
    return false;
  }

  // Message Count
  front_left_can_id_count_ = 0;
  front_right_can_id_count_ = 0;
  rear_left_can_id_count_ = 0;
  rear_right_can_id_count_ = 0;

  front_left_obj_list_ready_ = false;
  front_left_obj_list_.Clear();

  front_right_obj_list_ready_ = false;
  front_right_obj_list_.Clear();

  rear_left_obj_list_ready_ = false;
  rear_left_obj_list_.Clear();

  rear_right_obj_list_ready_ = false;
  rear_right_obj_list_.Clear();

  LOG_INFO(3) << "Create thread of receiving anngic data...";
  running_flag_recv_ = true;
  thread_recv_ = boost::thread(boost::bind(&TaskRecvAnngicRadarData::ThreadReceiving, this));

  LOG_INFO(3) << "Create thread of receiving anngic data... [OK]";

  return (true);
}

bool TaskRecvAnngicRadarData::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;

    LOG_INFO(3) << "Stop thread of receiving anngic data ...";
    bool ret = thread_recv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving anngic data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving anngic data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving anngic data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvAnngicRadarData::ThreadReceiving() {
  LOG_INFO(3) << "Thread of receiving anngic data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFdFrame can_frame[64];
  while (running_flag_recv_) {
    frame_num = can_channel_->ReadCanFdWait(&(can_frame[0]), 64, 50);

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFdFrame& fm = can_frame[i];
#if 0
        printf("Anngic: id[%08X] data\n", fm.id);
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
      framework::MessageCanFdFrameList message(framework::MSG_ID_RECV_ANNGIC_RADAR_CANFD_FRAME, &canfd_frame_list);
      Notify(message);

      canfd_frame_list.Clear();

      NotifyObjList(front_left_obj_list_ready_, 0);
      NotifyObjList(front_right_obj_list_ready_, 1);
      NotifyObjList(rear_left_obj_list_ready_, 4);
      NotifyObjList(rear_right_obj_list_ready_, 5);

      #if 0
      if (front_left_obj_list_ready_) {
        ad_msg::ObstacleRadarList& obstacle_list = front_left_obj_list_.obstacle_list_;

        obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC;
        // Convert to vehicle coordinate.
        framework::SharedData::instance()->GetChassis(&chassis_);
        common::Matrix<Float32_t, 2, 1> point_conv;
        for (Int32_t j = 0; j < obstacle_list.obstacle_num; ++j) {
          ad_msg::ObstacleRadar& obj = obstacle_list.obstacles[j];

          point_conv(0) = obj.x;
          point_conv(1) = obj.y;
          common::TransformVert_2D(mat_calibration_, &point_conv);
          obj.x = point_conv(0);
          obj.y = point_conv(1);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify Obstacle Front Left Radar List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT, &front_left_obj_list_.GetObjList());
        Notify(message);

        front_left_obj_list_ready_ = false;
        front_left_obj_list_.Clear();
      }

      if (front_right_obj_list_ready_) {
        ad_msg::ObstacleRadarList& obstacle_list = front_right_obj_list_.obstacle_list_;

        obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC;
        // Convert to vehicle coordinate.
        framework::SharedData::instance()->GetChassis(&chassis_);
        common::Matrix<Float32_t, 2, 1> point_conv;
        for (Int32_t j = 0; j < obstacle_list.obstacle_num; ++j) {
          ad_msg::ObstacleRadar& obj = obstacle_list.obstacles[j];

          point_conv(0) = obj.x;
          point_conv(1) = obj.y;
          common::TransformVert_2D(mat_calibration_, &point_conv);
          obj.x = point_conv(0);
          obj.y = point_conv(1);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify Obstacle Front Right Radar List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT, &front_right_obj_list_.GetObjList());
        Notify(message);

        front_right_obj_list_ready_ = false;
        front_right_obj_list_.Clear();
      }

      if (rear_left_obj_list_ready_) {
        ad_msg::ObstacleRadarList& obstacle_list = rear_left_obj_list_.obstacle_list_;

        obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC;
        // Convert to vehicle coordinate.
        framework::SharedData::instance()->GetChassis(&chassis_);
        common::Matrix<Float32_t, 2, 1> point_conv;
        for (Int32_t j = 0; j < obstacle_list.obstacle_num; ++j) {
          ad_msg::ObstacleRadar& obj = obstacle_list.obstacles[j];

          point_conv(0) = obj.x;
          point_conv(1) = obj.y;
          common::TransformVert_2D(mat_calibration_, &point_conv);
          obj.x = point_conv(0);
          obj.y = point_conv(1);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify Obstacle Rear Left Radar List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT, &rear_left_obj_list_.GetObjList());
        Notify(message);

        rear_left_obj_list_ready_ = false;
        rear_left_obj_list_.Clear();
      }

      if (rear_right_obj_list_ready_) {
        ad_msg::ObstacleRadarList& obstacle_list = rear_right_obj_list_.obstacle_list_;

        obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC;
        // Convert to vehicle coordinate.
        framework::SharedData::instance()->GetChassis(&chassis_);
        common::Matrix<Float32_t, 2, 1> point_conv;
        for (Int32_t j = 0; j < obstacle_list.obstacle_num; ++j) {
          ad_msg::ObstacleRadar& obj = obstacle_list.obstacles[j];

          point_conv(0) = obj.x;
          point_conv(1) = obj.y;
          common::TransformVert_2D(mat_calibration_, &point_conv);
          obj.x = point_conv(0);
          obj.y = point_conv(1);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify Obstacle Rear Right Radar List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT, &rear_right_obj_list_.GetObjList());
        Notify(message);

        rear_right_obj_list_ready_ = false;
        rear_right_obj_list_.Clear();
      }
      #endif
    }
  }

  LOG_INFO(3) << "Thread of receiving mobileye data ... [Stopped]";
}

void TaskRecvAnngicRadarData::NotifyObjList(bool obj_list_ready, Uint8_t radar_symbol) {
  if (obj_list_ready) {
    ad_msg::ObstacleRadarList* obstacle_list = NULL;
    switch(radar_symbol) {
      case (0):
        obstacle_list = &front_left_obj_list_.obstacle_list_;
        obstacle_list->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC_FRONT_LEFT;
        break;

      case (1):
        obstacle_list = &front_right_obj_list_.obstacle_list_;
        obstacle_list->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC_FRONT_RIGHT;
        break;

      case (4):
        obstacle_list = &rear_left_obj_list_.obstacle_list_;
        obstacle_list->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC_REAR_LEFT;
        break;

      case (5):
        obstacle_list = &rear_right_obj_list_.obstacle_list_;
        obstacle_list->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC_REAR_RIGHT;
        break;

      default:
        break;
    }

    //obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ANNGIC;
    // Convert to vehicle coordinate.
    framework::SharedData::instance()->GetChassis(&chassis_);
    common::Matrix<Float32_t, 2, 1> point_conv;
    for (Int32_t j = 0; j < obstacle_list->obstacle_num; ++j) {
      ad_msg::ObstacleRadar& obj = obstacle_list->obstacles[j];

      point_conv(0) = obj.x;
      point_conv(1) = obj.y;
      common::TransformVert_2D(mat_calibration_, &point_conv);
      obj.x = point_conv(0);
      obj.y = point_conv(1);

      obj.v_x += chassis_.v;
    }

    if (0 == radar_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Front Left Radar List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT, &front_left_obj_list_.GetObjList());
        Notify(message);

        front_left_can_id_count_ = 0;
        front_left_obj_list_ready_ = false;
        front_left_obj_list_.Clear();

    } else if (1 == radar_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Front Right Radar List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT, &front_right_obj_list_.GetObjList());
        Notify(message);

        front_right_can_id_count_ = 0;
        front_right_obj_list_ready_ = false;
        front_right_obj_list_.Clear();

    } else if (4 == radar_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Rear Left Radar List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT, &rear_left_obj_list_.GetObjList());
        Notify(message);

        rear_left_can_id_count_ = 0;
        rear_left_obj_list_ready_ = false;
        rear_left_obj_list_.Clear();

    } else if (5 == radar_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Rear Right Radar List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageRecvRadarData message(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT, &rear_right_obj_list_.GetObjList());
        Notify(message);

        rear_right_can_id_count_ = 0;
        rear_right_obj_list_ready_ = false;
        rear_right_obj_list_.Clear();

    }
  }
}

void TaskRecvAnngicRadarData::FullCanFdFrame(const can_dev::CanFdFrame& frame) {
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

void TaskRecvAnngicRadarData::ParseCanFrame(const can_dev::CanFdFrame &frame) {
  Uint16_t front_left_message_count_ = 0;
  Uint16_t front_right_message_count_ = 0;
  Uint16_t rear_left_message_count_ = 0;
  Uint16_t rear_right_message_count_ = 0;

  static Uint16_t s_front_left_message_count_ = 0;
  static Uint16_t s_front_right_message_count_ = 0;
  static Uint16_t s_rear_left_message_count_ = 0;
  static Uint16_t s_rear_right_message_count_ = 0;

    if(0x19D1D1D0 <= frame.id && frame.id <= 0x19D1E0D0) {   
    // Front Left Obstacles
    if(frame.id == 0x19D1D1D0) {
      front_left_can_id_count_ = 0;
      front_left_obj_list_ready_ = false;
      front_left_obj_list_.Clear();
    }

    ParseObjFromCanFrame(frame, 0, &front_left_message_count_);
    if(frame.id == 0x19D1D1D0) {
      s_front_left_message_count_ = front_left_message_count_;
      front_left_can_id_count_++;
    } else {
      if(s_front_left_message_count_ == front_left_message_count_) {
        front_left_can_id_count_++;
      }
    }

    //printf("s_front_left_message_count_:%d, front_left_message_count_:%d, front_left_can_id_count_:%d\n", s_front_left_message_count_, front_left_message_count_, front_left_can_id_count_);

    if(frame.id == 0x19D1E0D0 && front_left_can_id_count_ == 16) {
      front_left_obj_list_ready_ = true;
    }
  } else if(0x19D1E1D1 <= frame.id && frame.id <= 0x19D1F0D1) {
    // Front Right Obstacles
    if(frame.id == 0x19D1E1D1) {
      front_right_can_id_count_ = 0;
      front_right_obj_list_ready_ = false;
      front_right_obj_list_.Clear();
    }

    ParseObjFromCanFrame(frame, 1, &front_right_message_count_);
    if(frame.id == 0x19D1E1D1) {
      s_front_right_message_count_ = front_right_message_count_;
      front_right_can_id_count_++;
    } else {
      if(s_front_right_message_count_ == front_right_message_count_) {
        front_right_can_id_count_++;
      }
    }

    //printf("s_front_right_message_count_:%d, front_right_message_count_:%d, front_right_can_id_count_:%d\n", s_front_right_message_count_, front_right_message_count_, front_right_can_id_count_);

    if(frame.id == 0x19D1F0D1 && front_right_can_id_count_ == 16) {
      front_right_obj_list_ready_ = true;
    }
  } else if(0x19D211D2 <= frame.id && frame.id <= 0x19D220D2) {
    // Rear Left Obstacles
    if(frame.id == 0x19D211D2) {
      rear_left_can_id_count_ = 0;
      rear_left_obj_list_ready_ = false;
      rear_left_obj_list_.Clear();
    }

    ParseObjFromCanFrame(frame, 4, &rear_left_message_count_);
    if(frame.id == 0x19D211D2) {
      s_rear_left_message_count_ = rear_left_message_count_;
      rear_left_can_id_count_++;
    } else {
      if(s_rear_left_message_count_ == rear_left_message_count_) {
        rear_left_can_id_count_++;
      }
    }

    //printf("s_rear_left_message_count_:%d, rear_left_message_count_:%d, rear_left_can_id_count_:%d\n", s_rear_left_message_count_, rear_left_message_count_, rear_left_can_id_count_);

    if(frame.id == 0x19D220D2 && rear_left_can_id_count_ == 16) {
      rear_left_obj_list_ready_ = true;
    }
  } else if(0x19D221D3 <= frame.id && frame.id <= 0x19D230D3) {
    // Rear Right Obstacles
    if(frame.id == 0x19D221D3) {
      rear_right_can_id_count_ = 0;
      rear_right_obj_list_ready_ = false;
      rear_right_obj_list_.Clear();
    }

    ParseObjFromCanFrame(frame, 5, &rear_right_message_count_);
    if(frame.id == 0x19D221D3) {
      s_rear_right_message_count_ = rear_right_message_count_;
      rear_right_can_id_count_++;
    } else {
      if(s_rear_right_message_count_ == rear_right_message_count_) {
        rear_right_can_id_count_++;
      }
    }

    //printf("s_rear_right_message_count_:%d, rear_right_message_count_:%d, rear_right_can_id_count_:%d\n", s_rear_right_message_count_, rear_right_message_count_, rear_right_can_id_count_);

    if(frame.id == 0x19D230D3 && rear_right_can_id_count_ == 16) {
      rear_right_obj_list_ready_ = true;
    }
  }

#if 0
  static Uint8_t frame_num_0 = 0;
  static Uint8_t frame_num_1 = 0;
  static Uint8_t frame_num_2 = 0;
  static Uint8_t frame_num_3 = 0;
  //printf("Anngic_CAN_ID:0x%x\n", frame.id);
  // 最多32个目标，每条Message放2个目标，需要分配16个ID
  if(0x19D1D1D0 <= frame.id && frame.id <= 0x19D1E0D0) {   
    // Front Left Obstacles
    if(frame.id == 0x19D1D1D0) {
      if (frame_num_0 > 0) {
        front_left_obj_list_ready_ = true;
        frame_num_0 = 0;
      }
    }

    ParseObjFromCanFrame(frame, 0);

    frame_num_0++;

  } else if(0x19D1E1D1 <= frame.id && frame.id <= 0x19D1F0D1) {
    // Front Right Obstacles
    if(frame.id == 0x19D1E1D1) {
      if (frame_num_1 > 0) {
        front_right_obj_list_ready_ = true;
        frame_num_1 = 0;
      }
    }

    ParseObjFromCanFrame(frame, 1);
    frame_num_1++;

  }else if(0x19D211D2 <= frame.id && frame.id <= 0x19D220D2) {
    // Rear Left Obstacles
    if(frame.id == 0x19D211D2) {
      if (frame_num_2 > 0) {
        rear_left_obj_list_ready_ = true;
        frame_num_2 = 0;
      }
    }

    ParseObjFromCanFrame(frame, 4);
    frame_num_2++;

  }else if(0x19D221D3 <= frame.id && frame.id <= 0x19D230D3) {
    // Rear Right Obstacles
    if(frame.id == 0x19D221D3) {
      if (frame_num_3 > 0) {
        rear_right_obj_list_ready_ = true;
        frame_num_3 = 0;
      }
    }

    ParseObjFromCanFrame(frame, 5);
    frame_num_3++;

  }
#endif
}

void TaskRecvAnngicRadarData::ParseObjFromCanFrame(
    const can_dev::CanFdFrame& frame, Uint8_t radar_symbol, Uint16_t *message_count) {
  // Obj 1
  ParseOneObjFromData(&frame.data[0], radar_symbol, message_count);
  // Obj 2
  ParseOneObjFromData(&frame.data[32], radar_symbol, message_count);
}

void TaskRecvAnngicRadarData::ParseOneObjFromData(const Uint8_t* data, Uint8_t radar_symbol, Uint16_t *message_count) {
    // 传感器ID, Factor:1｜Offset: 0∣Range:0~255
  // 0x0: 前向,0x1: 左前角,0x2: 右前角,0x3: 左侧角,0x4: 右侧角,0x5: 左侧向,0x6: 右侧向
  Uint8_t SensorID = parse_sensor_data(data, 0, 0, 4);

  // 计数器, Factor:1｜Offset: 0∣Range:0~65535
  Uint16_t MeasCounter = parse_sensor_data(data, 1, 8, 16);
  *message_count = MeasCounter;

  // 目标ID, Factor:1｜Offset: 0∣Range:0~255
  Uint8_t Object_ID = parse_sensor_data(data, 3, 24, 8);

  // 纵向距离, Factor:0.1｜Offset: -200∣Range:-200~250m
  Float32_t Object_DistLgt = parse_sensor_data(data, 4, 32, 13) * 0.1 + (-200);

  // 横向距离, Factor:0.1｜Offset: -100∣Range:-100~100m
  Float32_t Object_DistLat = parse_sensor_data(data, 6, 48, 11) * 0.1 + (-100);

  // 纵向相对速度, Factor:0.1｜Offset: -100∣Range:-100~100m/s
  Float32_t Object_VrelLgt = parse_sensor_data(data, 8, 64, 11) * 0.1 + (-100);

  // 横向相对速度, Factor:0.1｜Offset: -100∣Range:-100~100m/s
  Float32_t Object_VrelLat = parse_sensor_data(data, 10, 80, 11) * 0.1 + (-100);

  // 目标俯仰角度, Factor:0.1｜Offset: -10∣Range:-10~10deg
  Float32_t Object_VertAngel = parse_sensor_data(data, 12, 96, 8) * 0.1 + (-10);

  // 水平方位角, Factor:0.1｜Offset: -90∣Range:-90~90deg
  Float32_t Object_HoriAngel = parse_sensor_data(data, 13, 104, 11) * 0.1 + (-90);

  // 横向相对加速度, Factor:0.01｜Offset: -10∣Range:-10~10m/s2
  Float32_t Object_ArelLat = parse_sensor_data(data, 15, 120, 11) * 0.01 + (-10);

  // 纵向相对加速度, Factor:0.01｜Offset: -10∣Range:-10~10m/s2
  Float32_t Object_ArelLgt = parse_sensor_data(data, 17, 136, 11) * 0.01 + (-10);

  // RCS值, Factor:1｜Offset: 0∣Range:0~250
  Uint8_t Object_RCS = parse_sensor_data(data, 19, 152, 8);

  // 目标宽度, Factor:0.2｜Offset: 0∣Range:0~51
  Float32_t Object_Width = parse_sensor_data(data, 20, 160, 8) * 0.2;

  // 目标种类, Factor:1｜Offset: 0∣Range:0~7
  // 0x0: point, 0x1: 车, 0x2: 人, 0x3~0x7: 预留 
  Uint8_t Object_Class = parse_sensor_data(data, 21, 168, 3);

  // 目标运动状态, Factor:1｜Offset: 0∣Range:0~3
  // 0x0: 目标静止, 0x1: 目标运动 
  Uint8_t Object_DynProp = parse_sensor_data(data, 21, 171, 2);

  // 横向距离标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_DistLat_rms = parse_sensor_data(data, 22, 176, 5);

  // 纵向速度标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_VrelLong_rms = parse_sensor_data(data, 22, 181, 5);

  // 纵向距离标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_DistLong_rms = parse_sensor_data(data, 23, 186, 5);

  // 横向速度标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_VrelLat_rms = parse_sensor_data(data, 24, 192, 5);

  // 纵向加速度标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_ArelLong_rms = parse_sensor_data(data, 24, 197, 5);

  // 横向加速度标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_ArelLat_rms = parse_sensor_data(data, 25, 202, 5);

  // 水平方位角标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_HoriAngel_rms = parse_sensor_data(data, 26, 208, 5);

  // 俯仰角度标准差, Factor:1｜Offset: 0∣Range:0~31
  Uint8_t Object_VertAngel_rms = parse_sensor_data(data, 27, 216, 5);

  // 目标来历, Factor:1｜Offset: 0∣Range:0~7
  // 0x0: deleted, 0x1: new, 0x2: measured, 0x3: predicted, 0x4: deletedformerge, 0x5: newfrommerge
  Uint8_t Object_MeasState = parse_sensor_data(data, 28, 224, 3);

  // 置信度, Factor:1｜Offset: 0∣Range:0~7
  // 0x0: 不可信, 0x1: ＜25%, 0x2: ＜50%, 0x3: ＜75%, 0x4: ＜90%, 0x5: ＜99%, 0x6: ＜99.9%, 0x7: ≤100%
  Uint8_t Object_ProbOfExist = parse_sensor_data(data, 28, 227, 3);

  if (Object_ID >= 255 || Object_ID == 0) {
    return;
  }

#if ADD_X_Y_LIMITATION_FOR_AROUND_RADAR

  // 根据障碍物横向纵向距离过滤障碍物
  // 过滤掉纵向距离大于150m的障碍物,或小于-70m的障碍物
  if (Object_DistLgt > 150 || Object_DistLgt < -70) {
    return;
  }
  // 过滤掉横向距离大于三个车道的障碍物(1个车道按4m计算)
  if (abs(Object_DistLat) > 10) {
    return;
  }

#endif

#if 0
  //TODO:车上临时调试代码，原为4，现为2
  if (Object_ProbOfExist < 2) {
    return;
  }
#endif

#if 0
  for(int i = 0; i < 32; i++) {
    printf("%x ", data[i]);
  }
  printf("\n");

  printf("SensorID:%d\n", SensorID);
  printf("MeasCounter:%d\n", MeasCounter);
  printf("Object_ID:%d\n", Object_ID);
  printf("Object_DistLgt:%f\n", Object_DistLgt);
  printf("Object_DistLat:%f\n", Object_DistLat);
  printf("Object_VrelLgt:%f\n", Object_VrelLgt);
  printf("Object_VrelLat:%f\n", Object_VrelLat);
  printf("Object_VertAngel:%f\n", Object_VertAngel);
  printf("Object_HoriAngel:%f\n", Object_HoriAngel);
  printf("Object_ArelLat:%f\n", Object_ArelLat);
  printf("Object_ArelLgt:%f\n", Object_ArelLgt);
  printf("Object_RCS:%d\n", Object_RCS);
  printf("Object_Width:%f\n", Object_Width);
  printf("Object_Class:%d\n", Object_Class);
  printf("Object_DynProp:%d\n", Object_DynProp);
  printf("Object_DistLat_rms:%d\n", Object_DistLat_rms);
  printf("Object_VrelLong_rms:%d\n", Object_VrelLong_rms);
  printf("Object_DistLong_rms:%d\n", Object_DistLong_rms);
  printf("Object_VrelLat_rms:%d\n", Object_VrelLat_rms);
  printf("Object_ArelLong_rms:%d\n", Object_ArelLong_rms);
  printf("Object_ArelLat_rms:%d\n", Object_ArelLat_rms);
  printf("Object_HoriAngel_rms:%d\n", Object_HoriAngel_rms);
  printf("Object_VertAngel_rms:%d\n", Object_VertAngel_rms);
  printf("Object_MeasState:%d\n", Object_MeasState);
  printf("Object_ProbOfExist:%d\n", Object_ProbOfExist);
#endif

  // Get back buffer from list
  ad_msg::ObstacleRadar* obj = Nullptr_t;
  switch (radar_symbol) {
    case (0):
      obj = front_left_obj_list_.Back();
      break;
    case (1):
      obj = front_right_obj_list_.Back();
      break;
    case (4):
      obj = rear_left_obj_list_.Back();
      break;
    case (5):
      obj = rear_right_obj_list_.Back();
      break;
    default:
      break;
  }

  if (Nullptr_t == obj) {
    LOG_ERR << "Obj list is full.";
    return;
  }

  /// 障碍物ID
  obj->id = Object_ID;

  /// 障碍物的类型
  switch (Object_Class) {
    case (1):
      obj->type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case (2):
      obj->type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    default:
      obj->type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
  }

  /// 障碍物长度 (m)
  obj->length = 1.0F;
  /// 障碍宽度 (m)
  //obj->width = Object_Width;
  obj->width = 1.0F;
  /// 障碍物位置x坐标 (m)
  obj->x = Object_DistLgt;
  /// 障碍物位置y坐标 (m)
  obj->y = Object_DistLat;
  /// 障碍物的x向速度 (m/sec)
  obj->v_x = Object_VrelLgt;
  /// 障碍物的y向速度 (m/sec)
  obj->v_y = Object_VrelLat;
  /// 障碍物的x向加速度 (m/sec^2)
  obj->accel_x = Object_ArelLgt;
  /// 障碍物的y向加速度 (m/sec^2)
  obj->accel_y = Object_ArelLat;

#if  DEBUG_REL_VX
  //printf("-------------------DEBUG_REL_VX0-------------------\n\n\n");
  obj->range_acceleration = Object_VrelLgt;
#endif

  // Push back to list
  switch (radar_symbol) {
    case (0):
      front_left_obj_list_.PushBack();
      break;
    case (1):
      front_right_obj_list_.PushBack();
      break;
    case (4):
      rear_left_obj_list_.PushBack();
      break;
    case (5):
      rear_right_obj_list_.PushBack();
      break;
    default:
      break;
  }
}

} // namespace sensor
} // namespace phoenix

