#ifndef PHOENIX_CAN_DEV_CAN_DRIVER_H_
#define PHOENIX_CAN_DEV_CAN_DRIVER_H_


#include "utils/macros.h"
#include "utils/com_utils.h"

namespace phoenix {
namespace can_dev {

enum {
  CAN_BIT_RATE_INVALID = 0,
  CAN_BIT_RATE_250K,
  CAN_BIT_RATE_500K
};

struct CanChannelParam {
  Int32_t channel;
  Int32_t bit_rate;

  struct {
    Char_t ip_addr[32];
    Uint16_t port;
    Uint16_t notify_port;
  } can_net;

  void Clear() {
    channel = -1;
    bit_rate = CAN_BIT_RATE_INVALID;
    common::com_memset(can_net.ip_addr, 0, sizeof(can_net.ip_addr));
    can_net.port = 0;
    can_net.notify_port = 0;
  }

  CanChannelParam() {
    Clear();
  }
};

struct CanFrame {
  /// 时间戳
  Int64_t time_stamp;

  /// 报文 ID,标准帧为 11 位,扩展帧为 29 位;
  Uint32_t id;
  /// 1-远程帧, 0-数据帧
  Uint8_t RTR;
  /// 1-扩展帧, 0-标准帧
  Uint8_t EXT;
  /// 报文数据长度, 取值如下:
  /// CAN 报文: 0~8;
  Int32_t data_len;
  /// 报文数据
  /// CAN: 报文数据长度为 8 字节
  Uint8_t data[8];

  void Clear() {
    time_stamp = 0;

    id = 0;
    RTR = false;
    EXT = false;
    data_len = 0;
    common::com_memset(data, 0, sizeof(data));
  }

  CanFrame() {
    Clear();
  }
};

struct CanFrameList {
  /// CAN帧的最大数量
  enum { MAX_CAN_FRAME_NUM = 64 };
  /// CAN帧数量
  Uint8_t can_frame_num;
  /// CAN帧列表
  CanFrame can_frame[MAX_CAN_FRAME_NUM];

  void Clear() {
    can_frame_num = 0;
  }

  CanFrameList() {
    Clear();
  }
};

struct CanFdFrame {
  /// 时间戳
  Int64_t time_stamp;

  /// 报文 ID,标准帧为 11 位,扩展帧为 29 位;
  Uint32_t id;
  /// 1-CANFD, 0-CAN
  Uint8_t FD;
  /// 1-远程帧, 0-数据帧, RTR位在FD位为 1 时,不允许设置为 1;
  Uint8_t RTR;
  /// 1-扩展帧, 0-标准帧
  Uint8_t EXT;
  /// 1-CANFD 加速, 0-不加速, BRS 位在 FD 为 1 时有效
  Uint8_t BRS;
  /// 1-被动错误, 0-主动错误, ESI 仅 CANFD 接收有效
  Uint8_t ESI;
  /// 报文数据长度, 取值如下:
  /// CAN 报文: 0~8; CANFD 报文: 0~8,12,16,20,24,32,48,64
  Int32_t data_len;
  /// 报文数据
  /// CAN: 报文数据长度为 8 字节; CAN FD : 报文数据长度 64 字节;
  Uint8_t data[64];

  void Clear() {
    time_stamp = 0;

    id = 0;
    FD = 0;
    RTR = 0;
    EXT = 0;
    BRS = 0;
    ESI = 0;
    data_len = 0;
    common::com_memset(data, 0, sizeof(data));
  }

  CanFdFrame() {
    Clear();
  }
};

struct CanFdFrameList {
  /// CANFD帧的最大数量
  enum { MAX_CANFD_FRAME_NUM = 64 };
  /// CANFD帧数量
  Uint8_t canfd_frame_num;
  /// CANFD帧列表
  CanFdFrame canfd_frame[MAX_CANFD_FRAME_NUM];

  void Clear() {
    canfd_frame_num = 0;
  }

  CanFdFrameList() {
    Clear();
  }
};

class CanDriver {
public:
  CanDriver();
  virtual ~CanDriver();

  virtual bool OpenChannel(const CanChannelParam& param);
  virtual bool CloseChannel();

  virtual Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  virtual Int32_t SendCanFd(const CanFdFrame* frame, Int32_t frame_num = 1);

  virtual Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);
  virtual Int32_t ReadCanFdWait(
      CanFdFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);
};


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_CAN_DRIVER_H_
