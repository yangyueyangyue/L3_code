#ifndef PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_FD_H_
#define PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_FD_H_


#include "utils/macros.h"
#include "container/ring_buffer.h"
#include "can_dev/can_driver.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_ZLGCANNET)

class CanDriverZlgCanNetFd : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverZlgCanNetFd();
  ~CanDriverZlgCanNetFd();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();

  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

  Int32_t SendCanFd(const CanFdFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadCanFdWait(
      CanFdFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  struct CanMsgHeader {
    Uint8_t magic;
    Uint8_t package_type;
    Uint8_t package_param;
    Uint16_t data_len;
    Uint16_t total_msg_size;
  };

private:
  void SpinCanData(const Uint8_t* msg_buf, Int32_t msg_size);
  void ParseCanFrame(
      const CanMsgHeader& header, const Uint8_t* msg_buf, Int32_t msg_size);

private:
  static bool device_is_open_;

  bool channel_is_open_;
  Int32_t sockfd_working_;

  Int32_t channel_id_;

  enum { CAN_MSG_HEADER_SIZE = 6 };
  enum { CAN_FRAMES_BUF_SIZE = (6/*包头*/+80/*数据区*/+1/*校验码*/+1/*Reserved*/)*18/*最多18帧*/ };
  enum { CAN_FRAME_HEADER_SIZE = 8/*时间戳*/+4/*报文 ID*/+2/*报文信息*/+1/*报文通道*/+1/*数据长度*/ };
  Uint8_t can_frames_buff_[CAN_FRAMES_BUF_SIZE];
  Int32_t can_frames_buff_idx_;
  CanMsgHeader can_msg_header_;

  common::RingBuffer<CanFdFrame, 50> received_can_frames_;
};

#endif  // #if (ENABLE_CAN_DEV_ZLGCANNET)


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_FD_H_
