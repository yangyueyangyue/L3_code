//
#include <string.h>
#include <stdio.h>
#include "can_dev/mdc_can/can_driver_mdc.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_MDC)


#if (MDC_CAN_DEV == MDC_CAN_DEV_16)

bool CanDriverMdc::device_is_open_ = false;

CanDriverMdc::CanDriverMdc() {
  channel_is_open_ = false;

  channel_id_ = -1;
  instance_id_ = -1;
}

CanDriverMdc::~CanDriverMdc() {
  CloseChannel();
}

bool CanDriverMdc::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open MDC CAN device -------->";

  LOG_INFO(3) << "<-------- Open MDC CAN device ---------";

  return (true);
}

bool CanDriverMdc::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
  }

  return (true);
}

bool CanDriverMdc::OpenChannel(const CanChannelParam& param) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (channel_is_open_) {
    LOG_WARN << "This channel has been opened.";
    return (true);
  }
  if ((param.channel < 0) || (param.channel >= MAX_CHANNEL_NUM)) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }

  //初始化
  channel_id_ = param.channel;
  instance_id_ = param.channel + 1;

  can_tx_skeleton_ = std::make_unique<CanTxSkeleton>(
        ara::com::InstanceIdentifier(instance_id_),
        ara::com::MethodCallProcessingMode::kPoll);
  can_tx_skeleton_->OfferService();

  CanRxProxy::StartFindService(
        [this](ara::com::ServiceHandleContainer<CanRxProxy::HandleType> handles,
               ara::com::FindServiceHandle handler) {
    CanDriverMdc::ServiceAvailabilityCallback(std::move(handles), handler);
  }, instance_id_);

  // Channel ready
  channel_is_open_ = true;

  return (true);
}

bool CanDriverMdc::CloseChannel() {
  if (channel_is_open_) {
    // 退出CAN接收状态
    receiving_can_frames_cond_.notify_one();

    channel_is_open_ = false;
  }

  return (true);
}

Int32_t CanDriverMdc::Send(const CanFrame* frame, Int32_t frame_num) {
  if (!channel_is_open_) {
    LOG_ERR << "This channel is close";
    return (-1);
  }
  if (Nullptr_t == can_tx_skeleton_) {
    LOG_ERR << "Invalid can tx skeleton.";
    return (-1);
  }

  //填充数据
  CanBusDataParam can_data_parm;
  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    Element can_raw_data;
    can_raw_data.canId = frame->id;
    can_raw_data.validLen = frame->data_len;
    for (Int32_t j = 0; j < frame->data_len; j++) {
      can_raw_data.data.push_back(frame->data[j]);
    }
    can_data_parm.elementList.push_back(can_raw_data);

    send_count++;
  }

  // Event发送
  // Lock
  sending_can_frames_lock_.lock();
  // send
  auto can_msg = can_tx_skeleton_->CanDataTxEvent.Allocate();
  can_msg->elementList = (can_data_parm.elementList);
  can_msg->seq = (can_data_parm.seq);
  can_tx_skeleton_->CanDataTxEvent.Send(std::move(can_msg));
  can_data_parm.elementList.clear();
  // Unlock
  sending_can_frames_lock_.unlock();

  return (send_count);
}

Int32_t CanDriverMdc::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  Int32_t frame_num = 0;
  //填充数据
  // lock
  receiving_can_frames_lock_.lock();
  // fill data
  frame_num = 0;
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFrame* data = received_can_frames_.PopFront();
      if (Nullptr_t != data) {
        frame[frame_num] = *data;
        frame_num++;
      } else {
        break;
      }
    } else {
      break;
    }
  }
  // unlock
  receiving_can_frames_lock_.unlock();

  if (channel_is_open_ && (frame_num < 1)) {
    // Waiting
    boost::unique_lock<boost::mutex> lock(receiving_lock_);
#if 0
    receiving_can_frames_cond_.wait(lock);
#else
    receiving_can_frames_cond_.wait_until(
          lock, boost::chrono::system_clock::now() +
                boost::chrono::milliseconds(timeout_ms));
#endif
  }

  return (frame_num);
}

void CanDriverMdc::ServiceAvailabilityCallback(
    ara::com::ServiceHandleContainer<CanRxProxy::HandleType> handles,
    ara::com::FindServiceHandle handler) {
  for (Int32_t i = 0; i < handles.size(); i++) {
    Int32_t instance = static_cast<Uint16_t>(handles[i].GetInstanceId());
    if (instance != instance_id_) {
      continue;
    }
    if (Nullptr_t == can_rx_proxy_) {
      can_rx_proxy_ = std::make_unique<CanRxProxy>(handles[i]);
      can_rx_proxy_->CanDataRxEvent.Subscribe(
            ara::com::EventCacheUpdatePolicy::kNewestN, 100);
      Int32_t channel = channel_id_;
      can_rx_proxy_->CanDataRxEvent.SetReceiveHandler(
            [this, channel]() {
        CanDriverMdc::CanDataEventCallback(channel_id_);
      });
      break;
    }
  }
}

void CanDriverMdc::CanDataEventCallback(Uint8_t channel) {
  if ((channel < 0) || (channel >= MAX_CHANNEL_NUM)) {
    return;
  }
  if (channel != channel_id_) {
    return;
  }
  if (Nullptr_t == can_rx_proxy_) {
    LOG_ERR << "Invalid CAN RX proxy.";
    return;
  }

  // Lock
  receiving_can_frames_lock_.lock();
  // Read CAN frames
  can_rx_proxy_->CanDataRxEvent.Update();
  const auto& can_samples = can_rx_proxy_->CanDataRxEvent.GetCachedSamples();
  for (const auto &sample : can_samples) {
    //填充数据
    Int32_t can_frames_num = sample->elementList.size();
    for(Int32_t i = 0; i < can_frames_num; i++) {
      CanFrame* frame = received_can_frames_.AllocateOverride();
      if (Nullptr_t != frame) {
        frame->data_len = sample->elementList[i].validLen;
        frame->id = sample->elementList[i].canId;
        for(int j = 0; j < sample->elementList[i].validLen; j++) {
          frame->data[j] = sample->elementList[i].data[j];
        }
      } else {
        LOG_ERR << "Can't add can frames.";
      }
    }
  }
  can_rx_proxy_->CanDataRxEvent.Cleanup();
  // Unlock
  receiving_can_frames_lock_.unlock();

  // Notify
  receiving_can_frames_cond_.notify_one();
}

#elif (MDC_CAN_DEV == MDC_CAN_DEV_18)

using namespace ara::log;
using namespace ara::core;
using namespace mdc::mdccan;

bool CanDriverMdc::device_is_open_ = false;

bool CanDriverMdc::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open MDC18  CAN device -------->";

  LOG_INFO(3) << "<-------- Open MDC18 CAN device ---------";

  return (true);
}

bool CanDriverMdc::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
  }

  return (true);
}


CanDriverMdc::CanDriverMdc() {
  channel_is_open_ = false;

  channel_id_ = -1;
  instance_id_ = -1;
}

CanDriverMdc::~CanDriverMdc() {
  CloseChannel();
}

bool CanDriverMdc::OpenChannel(const CanChannelParam& param) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (channel_is_open_) {
    LOG_WARN << "This channel has been opened.";
    return (true);
  }
  if ((param.channel < 0) || (param.channel >= MAX_CHANNEL_NUM)) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }

  //初始化
  channel_id_ = param.channel;
  instance_id_ = param.channel + 1;
  std::string instance_id_str = std::to_string(instance_id_);

  // 注册服务发现的回调函数，当发现服务的时候，会回调该函数
  CanFdRxProxy::StartFindService(
        [this](ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles,
        ara::com::FindServiceHandle handler) {
    CanDriverMdc::ServiceAvailabilityCallback(std::move(handles), handler);
  }, ara::com::InstanceIdentifier(StringView(instance_id_str.c_str())));

  // Channel ready
  channel_is_open_ = true;

  return (true);
}

bool CanDriverMdc::CloseChannel() {
  if (channel_is_open_) {
    // 退出CAN接收状态
    receiving_can_frames_cond_.notify_one();

    channel_is_open_ = false;
  }

  return (true);
}

Int32_t CanDriverMdc::Send(const CanFrame* frame, Int32_t frame_num) {
  Int32_t send_count = 0;

  if (!channel_is_open_) {
    LOG_ERR << "This channel is close";
    return (-1);
  }
  if (Nullptr_t == can_rx_proxy_) {
    LOG_ERR<<"CanFdDataMethodSend can_rx_proxy_ == nullptr";
    return (-1);
  }

  // canfd 下发数据包
  // Lock
  sending_can_frames_lock_.lock();

  for (Int32_t i = 0; i < frame_num; i++) {
    mdc::mdccan::CanFdBusDataParam canRawdata;
    canRawdata.seq++;
    canRawdata.canId = frame->id;
    canRawdata.sendType = 0;
    canRawdata.canIdType = 1;
    canRawdata.timeStamp.second = 0xFFFFFFFF;
    canRawdata.timeStamp.nsecond = 0xFFFFFFFF;
    canRawdata.validLen = frame->data_len;
    for (Int32_t j = 0; j < frame->data_len; j++) {
      canRawdata.data.push_back(frame->data[j]);
    }
    // Method发送
    auto handle = can_rx_proxy_->CanFdDataSetMethod(canRawdata);

    send_count++;
  }

  // Unlock
  sending_can_frames_lock_.unlock();

  return (send_count);
}

Int32_t CanDriverMdc::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  Int32_t frame_num = 0;
  //填充数据
  // lock
  receiving_can_frames_lock_.lock();
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFrame* data = received_can_frames_.PopFront();
      if (Nullptr_t != data) {
        frame[frame_num] = *data;
        frame_num++;
      } else {
        break;
      }
    } else {
      break;
    }
  }
  // unlock
  receiving_can_frames_lock_.unlock();

  if (channel_is_open_ && (frame_num < 1)) {
    // Waiting
    boost::unique_lock<boost::mutex> lock(receiving_lock_);
#if 0
    receiving_can_frames_cond_.wait(lock);
#else
    receiving_can_frames_cond_.wait_until(
          lock, boost::chrono::system_clock::now() +
                boost::chrono::milliseconds(timeout_ms));
#endif
  }

  return (frame_num);
}

void CanDriverMdc::ServiceAvailabilityCallback(
    ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles,
    ara::com::FindServiceHandle handler) {
  if (handles.size() > 0) {
    for (Int32_t i = 0; i < handles.size(); i++) {
      std::string instance_id_str = std::to_string(instance_id_);
      if (handles[i].GetInstanceId().ToString() !=
          StringView(instance_id_str.c_str())) {
        continue;
      }
      if (Nullptr_t == can_rx_proxy_) {
    	Int32_t channel = channel_id_;
        // 注册接收MCU上传CANFD帧的回调函数
        can_rx_proxy_ = std::make_unique<CanFdRxProxy>(handles[i]);
        // canFd 短包订阅与回调
        can_rx_proxy_->CanFdDataSRxEvent.Subscribe(100);
        can_rx_proxy_->CanFdDataSRxEvent.SetReceiveHandler(
              [this, channel]() {
          CanDriverMdc::CanFdDataSEventCallback(channel);
        });
        // canFd 长包订阅与回调
        can_rx_proxy_->CanFdDataLRxEvent.Subscribe(100);
        can_rx_proxy_->CanFdDataLRxEvent.SetReceiveHandler(
              [this, channel]() {
          CanDriverMdc::CanFdDataLEventCallback(channel);
        });
      }
    }
  }
}

void CanDriverMdc::CanFdDataSEventCallback (Uint8_t channel) {
  if (channel < 0 || channel >= MAX_CHANNEL_NUM) {
    return;
  }
  if (channel != channel_id_) {
    return;
  }
  if (Nullptr_t == can_rx_proxy_) {
    return;
  }

  // Lock
  receiving_can_frames_lock_.lock();

  // 接收CAN帧
  can_rx_proxy_->CanFdDataSRxEvent.GetNewSamples(
        [this](ara::com::SamplePtr<CanFdBusDataParamS const> ptr) {
    // sample为收到数据
    auto sample = ptr.Get();

    //填充数据
    Int32_t can_frames_num = sample->elementList.size();
    for(Int32_t i = 0; i < can_frames_num; i++) {
      CanFrame* frame = received_can_frames_.AllocateOverride();
      if (Nullptr_t != frame) {
        frame->data_len = sample->elementList[i].validLen;
        frame->id = sample->elementList[i].canId;
        for(int j = 0; j < sample->elementList[i].validLen; j++) {
          frame->data[j] = sample->elementList[i].data[j];
        }
      } else {
        LOG_ERR << "Can't add can frames.";
      }
    }
  });

  // Unlock
  receiving_can_frames_lock_.unlock();

  // Notify
  receiving_can_frames_cond_.notify_one();
}

void CanDriverMdc::CanFdDataLEventCallback(uint8_t channelId) {
  // TODO: Add LE CAN Frames
}

#else
  ERROR: Invalid MDC CAN device.
#endif


#endif  // #if (ENABLE_CAN_DEV_MDC)


}  // namespace can_dev
}  // namespace phoenix
