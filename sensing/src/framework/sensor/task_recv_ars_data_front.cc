//
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "sensor/task_recv_ars_data_front.h"
#include "communication/shared_data.h"

#define ARS_RADAR_LOG_FLAG 0
#define ARS_RADAR_CANFRAME_LOG_FLAG 0

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {


TaskRecvArsDataFront::TaskRecvArsDataFront(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_ARS_DATA_FRONT, "Recv ARS Front Data", manager) {
  running_flag_recv_ = false;

  can_channel_ = Nullptr_t;

#if (ENABLE_CAN_DEV_ZLGCANNET)
  can_channel_ = new can_dev::CanDriverZlgCanNetFd();
#endif

  veh_model::VehicleModelWrapper veh_model;
  const veh_model::SensorOnVehicle& sensor_param = veh_model.GetSensor();
  const veh_model::SensorOnVehicle::Calibration& calibration =
      sensor_param.calibration_radar[0];
  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_calibration_.SetIdentity();
  common::Rotate_2D<Float32_t>(
        rotate_center, calibration.yaw_offset, &mat_calibration_);
  common::Translate_2D(
        calibration.x_offset, calibration.y_offset, &mat_calibration_);

  obj_list_.Clear();
  single_obj_ready_mask_ = 0;
  obj_can_frame_cnts_ = 0;
  asr_data_ready_ = false;
}

TaskRecvArsDataFront::~TaskRecvArsDataFront() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvArsDataFront::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 2;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 8002;

#if 0
  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 0;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.177");
  can_channel_param.can_net.port = 4001;
  can_channel_param.can_net.notify_port = 6001;
#endif

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel " << can_channel_param.channel;
    return false;
  }

  obj_list_.Clear();
  single_obj_ready_mask_ = 0;
  obj_can_frame_cnts_ = 0;
  asr_data_ready_ = false;

  LOG_INFO(3) << "Create thread of receiving ARS front data...";
  running_flag_recv_ = true;
  thread_recv_ = boost::thread(boost::bind(&TaskRecvArsDataFront::ThreadReceiving, this));

  LOG_INFO(3) << "Create thread of receiving ARS front data... [OK]";

  return (true);
}

bool TaskRecvArsDataFront::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;

    LOG_INFO(3) << "Stop thread of receiving ARS front data ...";
    bool ret = thread_recv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving ARS front data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving ARS front data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving ARS front data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvArsDataFront::ThreadReceiving() {
  LOG_INFO(3) << "Thread of receiving ARS front data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame[18];
  while (running_flag_recv_) {
    frame_num = can_channel_->ReadWait(&(can_frame[0]), 18, 50);
    // std::cout << "Received " << frame_num << " data from ARS front." << std::endl;

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFrame& fm = can_frame[i];
#if (ARS_RADAR_CANFRAME_LOG_FLAG == 1)
        printf("ARS: id[%08X] data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
               fm.id,
               fm.data[0], fm.data[1], fm.data[2], fm.data[3],
            fm.data[4], fm.data[5], fm.data[6], fm.data[7]);
#endif

        FullCanFrame(fm);

        ParseCanFrame(fm);
      }

      // 发送原始CanFdFrame数据
      framework::MessageCanFrameList message(framework::MSG_ID_RECV_ARS_RADAR_CAN_FRAME, &can_frame_list);
      Notify(message);

      can_frame_list.Clear();

      if((obj_can_frame_cnts_ >= 40) /*组成40帧后*/) {
        asr_data_ready_ = true;
        ad_msg::ObstacleRadarList& obstacle_list = obj_list_.obstacle_list_;

        obstacle_list.radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ARS430;
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

          obj.range = common::com_sqrt(
                common::Square(obj.x) + common::Square(obj.y));
          obj.angle = common::com_atan2(obj.y, obj.x);

          obj.v_x += chassis_.v;
        }

        LOG_INFO(3) << "### Notify 430 Radar List (num="
                    << obstacle_list.obstacle_num << ").";
        framework::MessageRecvRadarData message(
              framework::MSG_ID_RECV_ESR_DATA_FRONT, &obj_list_.GetObjList());
        Notify(message);

        obj_list_.Clear();
        obj_can_frame_cnts_ = 0;
        asr_data_ready_ = false;
      }
    }
  }

  LOG_INFO(3) << "Thread of receiving ARS front data ... [Stopped]";
}

void TaskRecvArsDataFront::FullCanFrame(const can_dev::CanFrame& frame) {
  can_frame_list.can_frame[can_frame_list.can_frame_num].time_stamp = frame.time_stamp;

  can_frame_list.can_frame[can_frame_list.can_frame_num].id = frame.id;
  can_frame_list.can_frame[can_frame_list.can_frame_num].RTR = frame.RTR;
  can_frame_list.can_frame[can_frame_list.can_frame_num].EXT = frame.EXT;
  can_frame_list.can_frame[can_frame_list.can_frame_num].data_len = frame.data_len;
  for(Uint8_t i = 0; i < frame.data_len; i++) {
    can_frame_list.can_frame[can_frame_list.can_frame_num].data[i] = frame.data[i];
  }

  can_frame_list.can_frame_num++;
}

static Uint32_t GetCRCMapValue(const Uint32_t index) {
  const static Uint32_t CRC_data_table[256] = {
    0x00	,	0x9B	,	0xAD	,	0x36	,	0xC1	,	0x5A	,	0x6C	,	0xF7	,
    0x19	,	0x82	,	0xB4	,	0x2F	,	0xD8	,	0x43	,	0x75	,	0xEE	,
    0x32	,	0xA9	,	0x9F	,	0x04	,	0xF3	,	0x68	,	0x5E	,	0xC5	,
    0x2B	,	0xB0	,	0x86	,	0x1D	,	0xEA	,	0x71	,	0x47	,	0xDC	,
    0x64	,	0xFF	,	0xC9	,	0x52	,	0xA5	,	0x3E	,	0x08	,	0x93	,
    0x7D	,	0xE6	,	0xD0	,	0x4B	,	0xBC	,	0x27	,	0x11	,	0x8A	,
    0x56	,	0xCD	,	0xFB	,	0x60	,	0x97	,	0x0C	,	0x3A	,	0xA1	,
    0x4F	,	0xD4	,	0xE2	,	0x79	,	0x8E	,	0x15	,	0x23	,	0xB8	,
    0xC8	,	0x53	,	0x65	,	0xFE	,	0x09	,	0x92	,	0xA4	,	0x3F	,
    0xD1	,	0x4A	,	0x7C	,	0xE7	,	0x10	,	0x8B	,	0xBD	,	0x26	,
    0xFA	,	0x61	,	0x57	,	0xCC	,	0x3B	,	0xA0	,	0x96	,	0x0D	,
    0xE3	,	0x78	,	0x4E	,	0xD5	,	0x22	,	0xB9	,	0x8F	,	0x14	,
    0xAC	,	0x37	,	0x01	,	0x9A	,	0x6D	,	0xF6	,	0xC0	,	0x5B	,
    0xB5	,	0x2E	,	0x18	,	0x83	,	0x74	,	0xEF	,	0xD9	,	0x42	,
    0x9E	,	0x05	,	0x33	,	0xA8	,	0x5F	,	0xC4	,	0xF2	,	0x69	,
    0x87	,	0x1C	,	0x2A	,	0xB1	,	0x46	,	0xDD	,	0xEB	,	0x70	,
    0x0B	,	0x90	,	0xA6	,	0x3D	,	0xCA	,	0x51	,	0x67	,	0xFC	,
    0x12	,	0x89	,	0xBF	,	0x24	,	0xD3	,	0x48	,	0x7E	,	0xE5	,
    0x39	,	0xA2	,	0x94	,	0x0F	,	0xF8	,	0x63	,	0x55	,	0xCE	,
    0x20	,	0xBB	,	0x8D	,	0x16	,	0xE1	,	0x7A	,	0x4C	,	0xD7	,
    0x6F	,	0xF4	,	0xC2	,	0x59	,	0xAE	,	0x35	,	0x03	,	0x98	,
    0x76	,	0xED	,	0xDB	,	0x40	,	0xB7	,	0x2C	,	0x1A	,	0x81	,
    0x5D	,	0xC6	,	0xF0	,	0x6B	,	0x9C	,	0x07	,	0x31	,	0xAA	,
    0x44	,	0xDF	,	0xE9	,	0x72	,	0x85	,	0x1E	,	0x28	,	0xB3	,
    0xC3	,	0x58	,	0x6E	,	0xF5	,	0x02	,	0x99	,	0xAF	,	0x34	,
    0xDA	,	0x41	,	0x77	,	0xEC	,	0x1B	,	0x80	,	0xB6	,	0x2D	,
    0xF1	,	0x6A	,	0x5C	,	0xC7	,	0x30	,	0xAB	,	0x9D	,	0x06	,
    0xE8	,	0x73	,	0x45	,	0xDE	,	0x29	,	0xB2	,	0x84	,	0x1F	,
    0xA7	,	0x3C	,	0x0A	,	0x91	,	0x66	,	0xFD	,	0xCB	,	0x50	,
    0xBE	,	0x25	,	0x13	,	0x88	,	0x7F	,	0xE4	,	0xD2	,	0x49	,
    0x95	,	0x0E	,	0x38	,	0xA3	,	0x54	,	0xCF	,	0xF9	,	0x62	,
    0x8C	,	0x17	,	0x21	,	0xBA	,	0x4D	,	0xD6	,	0xE0	,	0x7B
  };

  if((index < 0) || (index > 256)){
    LOG_ERR << "The input of index is illegal.";
    return (0);
  }

  return (CRC_data_table[index]);
}


void TaskRecvArsDataFront::ParseCanFrame(const can_dev::CanFrame &frame) {
  /// TODO：增加CRC判断

  // 用作障碍物数据帧起始标记
  // 雷达传感器会每个工作周期一次通过 CAN 总线传输雷达状态。
  // CAN Message: RDF_State, CAN ID: 0x18FF812A
  if(0x18FF812A == frame.id){
    obj_list_.Clear();
    obj_can_frame_cnts_ = 0;
    asr_data_ready_ = false;
  }

  ad_msg::ObstacleRadar* obj = obj_list_.Back();
  if (Nullptr_t == obj) {
    LOG_ERR << "Obj list is full.";
    return;
  }

  // 雷达传感器会通过 CAN 总线传输带属性的目标物。在每个测量周期内,雷达总是
  // 会向 CAN 发送 40 个目标物 (约72ms每次)。如果没有检测到目标物或者目标无效,
  // 雷达会为所有属性发送无效数据。

  // 一个目标物的所有的属性会被分割成4帧 (RDF_C01, RDF_C02, RDF_C03,
  // RDF_C04),每帧应当包括 CRC 校验和生存计数器, 信号的位置分配如下列所示。
  // 所有信号的字节顺序是英特尔格式, 所有信号的值类型是无符号。
  switch (frame.id) {
  case (0xCFF002A): {
    // RDF_C01

    // Clear mask
    single_obj_ready_mask_ = 0;

    /// TODO: 填写合适的类型值/状态值，switch ... case ...

    // 目标物识别号的定义域是 0~99.在特定目标物的生命周期内
    // 保持不变。当目标物消失时该识别号会被释放。越早的目标
    // 物取得越小的可用目标物识别号。
    Int32_t obj_id_1 = frame.data[2];
    // 目标物雷达反射截面积(RCS)的现值 (-40 ~ 85)(dB)
    Float32_t rcs_obj_cval = frame.data[3] * 0.5F - 40.0F;
    // 目标物的(运动)类型
    // 0x0  Not Detected 未检测出
    // 0x1   Moving 运动
    // 0x2   Moving to Stop 停止
    // 0x3   Stationary 静止
    // 0x4  Crossing 横穿
    // 0x5  Oncoming 驶来
    // 0x6  Error 错误
    // 0x7  SNA 信号不可用
    Int8_t type_obj_stat_rdf = (frame.data[4] & 0x07);
    // 此信号指示了目标物的分类,为 ACC 功能优化
    // 0x0    Point 点目标
    // 0x1    Car 小汽车
    // 0x2    Truck 卡车
    // 0x3    Pedestrian 行人
    // 0x4    Motorcycle 摩托车
    // 0x5    Bicycle 自行车
    // 0x6    Wide 宽目标
    // 0x7    Unclassified 未分类目标
    Int8_t obj_e_classification = (frame.data[4] >> 3) & 0x07;
    // 隧道观测状态现值
    // 0x0  Not Detected 未检测出
    // 0x1  Detected 检测出
    // 0x2  Error 错误
    // 0x3  SNA 信号不可用
    Int8_t tnl_obsrv_stat_rdf = (frame.data[4] >> 6) & 0x03;
    // 估测的动态属性置信度 (0 ~ 100)
    Uint8_t ui_dyn_confidence = frame.data[5];
    // 目标物为真实目标的置信度 (0 ~ 100)
    Uint8_t ui_prob_of_existence = frame.data[6];
    // 目标物消失原因
    /* 0x0   No Reason 原因未知
       0x1  Target Lost 目标消失
       0x2  Object Distance Far Away 目标物距离过远
       0x3  Object Distance Left 目标物距离向左过远
       0x4  Object Distance Right 目标物距离向右过远
       0x5  Lane Change Left 目标物向左变道
       0x6  Lane Change Right 目标物向右变道
       0x7  Object Loss Left Curve 目标物向左转弯消失
       0x8  Object Loss Right Curve 目标物向右转弯消失
       ...  Reserved 保留位
       0xE  Error 错误
       0xF  SNA 信号不可用
    */
    Int8_t loss_obj_stat_rdf = frame.data[7] & 0x0F;
    // 指示目标物的维护状态:猜测/测量的,创建原因,删除原因
    /*
       0x0   Deleted 删除
       0x1   New 新目标
       0x2   Measured 检测到的目标
       0x3   Predicted 估测的目标
       0x4   Inactive 不活跃的目标
       0x5   MAX_DIFF_TYPES
       0x6   Error 错误
       0x7   SNA 信号不可用
    */
    Int8_t e_maintenance_state = (frame.data[7] >> 4) & 0x07;

    #if (ARS_RADAR_LOG_FLAG == 1)
    printf("------C01---------------\n");
    printf("obj_id_1:%d\n", obj_id_1);
    printf("rcs_obj_cval:%f\n", rcs_obj_cval);
    printf("ui_prob_of_existence:%d\n", ui_prob_of_existence);
    printf("------------------------\n");
    #endif

    if ((0 <= obj_id_1) && (obj_id_1 <= 99) &&
        ((-10.0F <= rcs_obj_cval) && (rcs_obj_cval <= 100.0F)) &&
        (60 < ui_prob_of_existence) && (ui_prob_of_existence <= 100)) {
      obj->id = obj_id_1;

      single_obj_ready_mask_ |= 0x01;
    }
  }
    break;

  case (0xCFF012A): {
    // RDF_C02

    // 相关目标物的横向距离(y)的现值 (-51 ~ 51)(m)
    Float32_t dist_y_obj_cval_rdf =
        ((Uint32_t)(frame.data[2]) |
        ((((Uint32_t)(frame.data[3])) & 0x07) << 8)) * 0.05F - 51.0F;
    // 相关目标物的纵向距离(x)的现值 (-50 ~ 350)(m)
    Float32_t dist_x_obj_cval_rdf =
        ((((Uint32_t)(frame.data[4])) << 5) |
        (Uint32_t)((frame.data[3] >> 3)&0x1F)) * 0.05F - 50.0F;
    // 相关目标物的宽度的现值 (-12.5 ~ 12.5)(m)
    Float32_t width_obj_cval_rdf = frame.data[5] * 0.1F - 12.5F;
    // 目标物识别号的定义域是 0~99
    Int32_t obj_id_2 = frame.data[6];
    // 相关目标物的纵向相对加速度(ax)的现值 (-12.5 ~ 12.5)(m/s^2)
    Float32_t accel_x_obj_cval_rdf = frame.data[7] * 0.1F - 12.5F;

    #if (ARS_RADAR_LOG_FLAG == 1)
    printf("------C02---------------\n");
    printf("obj_id_2:%d\n", obj_id_2);
    printf("dist_y_obj_cval_rdf:%f\n", dist_y_obj_cval_rdf);
    printf("dist_x_obj_cval_rdf:%f\n", dist_x_obj_cval_rdf);
    printf("------------------------\n");
    #endif

    if ((0 <= obj_id_2) && (obj_id_2 <= 99) &&
        /*((-52.0F < dist_y_obj_cval_rdf) && (dist_y_obj_cval_rdf < 52.0F)) &&
        ((-51.0F < dist_x_obj_cval_rdf) && (dist_x_obj_cval_rdf < 200.0F))*/
        ((-10.0F < dist_y_obj_cval_rdf) && (dist_y_obj_cval_rdf < 10.0F)) &&  //过滤掉横向距离大于三个车道的障碍物(1个车道按4m计算)
        ((-51.0F < dist_x_obj_cval_rdf) && (dist_x_obj_cval_rdf < 150.0F))  //过滤掉纵向距离大于150m的障碍物,或小于-70m的障碍物
        ) {
      obj->x = dist_x_obj_cval_rdf;
      obj->y = dist_y_obj_cval_rdf;
      obj->width = width_obj_cval_rdf;
      obj->accel_x = accel_x_obj_cval_rdf;
      obj->accel_y = 0.0F;  // dummy

      single_obj_ready_mask_ |= 0x02;
    }
  }
    break;

  case (0xCFF022A): {
    // RDF_C03

    // 相关目标物的纵向相对速度(vx)的现值 (-400 ~ 400)(km/h)
    Float32_t spd_x_obj_cval_rdf =
        ((Uint32_t)frame.data[2] |
        ((((Uint32_t)frame.data[3]) & 0x1F) << 8)) * 0.1F - 400.0F;
    // 目标物识别号的定义域是 0~99
    Int32_t obj_id_3 = frame.data[4];
    // 高处目标物的置信度 (0 ~ 100)(%)
    Uint8_t obj_uc_obj_prob = frame.data[5];
    // 从目标物创建开始的周期计数
    Uint32_t obj_ui_life_cycles =
        (Uint32_t)frame.data[6] | (((Uint32_t)frame.data[7]) << 8);

    #if (ARS_RADAR_LOG_FLAG == 1)
    printf("------C03---------------\n");
    printf("obj_id_3:%d\n", obj_id_3);
    printf("spd_x_obj_cval_rdf:%f\n", spd_x_obj_cval_rdf);
    printf("------------------------\n");
    #endif


#if  DEBUG_REL_VX
    //printf("---------------------------DEBUG_REL_VX1-----------------------------\n\n\n");
    obj->range_acceleration = spd_x_obj_cval_rdf / 3.6F;

#endif

    if ((0 <= obj_id_3) && (obj_id_3 <= 99) &&
        ((-400.0F < spd_x_obj_cval_rdf) && (spd_x_obj_cval_rdf < 400.0F)) && 
        (obj_uc_obj_prob > 65)
        ) {
      obj->v_x = spd_x_obj_cval_rdf / 3.6F;

      single_obj_ready_mask_ |= 0x04;
    }
  }
    break;

  case (0xCFF032A): {
    // RDF_C04

    // 相关目标物横向距离的标准差现值 (0 ~ 31)
    Int32_t stdrng_y_obj_cval_rdf = frame.data[2] & 0x1F;
    // 相关目标物纵向距离的标准差现值 (0 ~ 31)
    Int32_t stdrng_x_obj_cval_rdf =
        ((frame.data[2] >> 5) & 0x07) | ((frame.data[3] & 0x03) << 3);
    // 相关目标物的相对横向速度的标准差现值 (0 ~ 31)
    Int32_t stdrngspd_y_obj_cval_rdf = (frame.data[3] >> 2) & 0x1F;
    // 相关目标物的相对纵向速度的标准差现值 (0 ~ 31)
    Int32_t stdrngspd_x_obj_cval_rdf =
        ((frame.data[4] & 0x0F) << 1 | ((frame.data[3] >> 7) & 0x01));
    // 相关目标物的横向相对速度(vy)的现值 (-400 ~ 400)(km/h)
    Float32_t spd_y_obj_cval_rdf =
        ((Uint32_t)((frame.data[4] >> 4) & 0x0F) |
        (((Uint32_t)frame.data[5] & 0xFF) << 4) |
        (((Uint32_t)frame.data[6] & 0x01) << 12)) * 0.1F - 400.0F;
    // 目标物识别号的定义域是 0~99
    Int32_t obj_id_4 = frame.data[7];

    #if (ARS_RADAR_LOG_FLAG == 1)
    printf("------C04---------------\n");
    printf("obj_id_4:%d\n", obj_id_4);
    printf("spd_y_obj_cval_rdf:%f\n", spd_y_obj_cval_rdf);
    printf("------------------------\n");
    #endif

    if ((0 <= obj_id_4) && (obj_id_4 <= 99) &&
        ((-400.0F < spd_y_obj_cval_rdf) && (spd_y_obj_cval_rdf < 400.0F))
        ) {
      obj->v_y = spd_y_obj_cval_rdf / 3.6F;
      single_obj_ready_mask_ |= 0x08;

      if (0x0F == single_obj_ready_mask_) {
        obj_list_.PushBack();
      }
    }

    obj_can_frame_cnts_++;
  }
    break;

  /// TODO: 车辆的状态待赋值
  default:
    break;
  }
}


} // namespace sensor
} // namespace phoenix

