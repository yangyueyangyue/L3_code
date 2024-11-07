#include "task_recv_adasisv2_msg.h"

#define CAN_ACCESS_USING_KVASER (1)
#define CAN_ACCESS_USING_ZLG_CANNET (0)

#if CAN_ACCESS_USING_KVASER
#include "can_dev/kvaser/can_driver_kvaser.h"
#endif

#if CAN_ACCESS_USING_ZLG_CANNET
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
#endif

#define ADASISV2_CANFRAME_LOG 1

namespace phoenix {
namespace adasisv2 {

TaskRecvADASISv2Msg::TaskRecvADASISv2Msg(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_MAP_ADASISV2, "Recv ADASISv2 Data", manager) {
  running_flag_recv_ = false;

#if CAN_ACCESS_USING_KVASER
  can_channel_ = new can_dev::CanDriverKvaser();
#endif

#if CAN_ACCESS_USING_ZLG_CANNET
  can_channel_ = new can_dev::CanDriverZlgCanNet();
#endif
}

TaskRecvADASISv2Msg::~TaskRecvADASISv2Msg() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvADASISv2Msg::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "[ADASIS] Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_) {
    return (true);
  }

  LOG_INFO(3) << "[ADASIS] Open CAN Device...";
#if CAN_ACCESS_USING_KVASER
  if (!can_dev::CanDriverKvaser::OpenDevice()) {
    LOG_ERR << "[ADASIS] Open CAN device failed.";
    return (false);
  }
#endif

#if CAN_ACCESS_USING_ZLG_CANNET
  if (!can_dev::CanDriverZlgCanNet::OpenDevice()) {
    LOG_ERR << "[ADASIS] Open CAN device failed.";
    return (false);
  }
#endif

  // Get ADASIS message from B-CAN
  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 1;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.177");
  can_channel_param.can_net.port = 4002;
  can_channel_param.can_net.notify_port = 6002;

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "[ADASIS] Falied to open CAN channel " << can_channel_param.channel;
    return false;
  }

  LOG_INFO(3) << "[ADASIS] Create thread of receiving ADASISv2 data...";
  running_flag_recv_ = true;
  thread_recv_ = boost::thread(boost::bind(&TaskRecvADASISv2Msg::ThreadReceiving, this));

  LOG_INFO(3) << "[ADASIS] Create thread of receiving ADASISv2 data... [OK]";

  return (true);
}

bool TaskRecvADASISv2Msg::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;

    LOG_INFO(3) << "[ADASIS] Stop thread of receiving ADASISv2 data ...";
    bool ret = thread_recv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "[ADASIS] Failed to wait thread of receiving ADASISv2 data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "[ADASIS] Stop thread of receiving ADASISv2 data ... [OK]";
    } else {
      LOG_INFO(3) << "[ADASIS] Stop thread of receiving ADASISv2 data ... [NG]";
    }

    LOG_INFO(3) << "[ADASIS] Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }

#if CAN_ACCESS_USING_KVASER
    can_dev::CanDriverKvaser::CloseDevice();
#endif

#if CAN_ACCESS_USING_ZLG_CANNET
    can_dev::CanDriverZlgCanNet::CloseDevice();
#endif
  }

  return (true);
}

void TaskRecvADASISv2Msg::ThreadReceiving() {
  LOG_INFO(3) << "[ADASIS] Thread of receiving ADASISv2 data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame;
  while (running_flag_recv_) {
    frame_num = can_channel_->ReadWait(&can_frame_, 1, 100);
    if (frame_num == 1) {
      // 发送原始ADASISv2数据
      // TODO: 当前只处理POSITION, PROFILE SHORT
      if (ADASIS_V2_ADAS_POSN_FRAME_ID == can_frame_.id || 
         ADASIS_V2_ADAS_PROFSHORT_FRAME_ID == can_frame_.id) {
#ifdef ADASISV2_CANFRAME_LOG  
      printf("[ADASIS] RAW : timestamp[%ld], id[0x%08X], data[%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_frame_.time_stamp, 
               can_frame_.id, can_frame_.data[0], can_frame_.data[1], can_frame_.data[2], can_frame_.data[3],
            can_frame_.data[4], can_frame_.data[5], can_frame_.data[6], can_frame_.data[7]);
#endif          
          framework::MessageCanFrame message(&can_frame_);
          Notify(message);
          
          ParseCanFrame(can_frame_);
      }
    }
  }

  LOG_INFO(3) << "[ADASIS] Thread of receiving ADASISv2 data ... [Stopped]";
}


void TaskRecvADASISv2Msg::ParseCanFrame(const can_dev::CanFrame &frame) {
  MessageType msg_type = parser_.ParseCanFrame(frame);

  if (ADASIS_V2_MESSAGE_TYPE_POSITION == msg_type) {
    PositionMessage position = parser_.getPosition();
    LOG_INFO(3) << "[ADASIS] POSITION : " << "path = " << position.path_index << ", offset = " << position.offset << ", slope = " << position.slope;
    framework::MessageRecvADASISV2Position message(&position);
    Notify(message);
  } else if (ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT == msg_type) {
    ProfileShortMessage profile_short = parser_.getProfileShort();
    LOG_INFO(3) << "[ADASIS] PROFILE SHORT : " << "type = " << profile_short.profile_type << ", path = " << profile_short.path_index << ", offset = " << profile_short.offset << ", value0 = " << profile_short.value0
                << ", distance1 = " << profile_short.distance1 << ", value1 = " << profile_short.value1;
    framework::MessageRecvADASISV2ProfileShort message(&profile_short);
    Notify(message);
  }
}

} // namespace sensor
} // namespace phoenix

