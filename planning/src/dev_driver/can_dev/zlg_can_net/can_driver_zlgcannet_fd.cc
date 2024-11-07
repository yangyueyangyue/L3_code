//
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include "utils/macros.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"


#define ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE (0)


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_ZLGCANNET)

static inline void DecodeInt16ValueBigEndian(
    const void* buf, Int32_t offset, Int16_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0]) << 8) |
      (((Uint32_t) buffer[offset + 1]));
}

static inline void DecodeUint16ValueBigEndian(
    const void* buf, Int32_t offset, Uint16_t* p) {
  DecodeInt16ValueBigEndian(buf, offset, (Int16_t*)p);
}

static inline void DecodeInt32ValueBigEndian(
    const void* buf, Int32_t offset, Int32_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0]) << 24) |
      (((Uint32_t) buffer[offset + 1]) << 16) |
      (((Uint32_t) buffer[offset + 2]) << 8) |
      (((Uint32_t) buffer[offset + 3]));
}

static inline void DecodeUint32ValueBigEndian(
    const void* buf, Int32_t offset, Uint32_t* p) {
  DecodeInt32ValueBigEndian(buf, offset, (Int32_t*)p);
}

static inline void DecodeInt64ValueBigEndian(
    const void* buf, Int32_t offset, Int64_t* p) {
  const Uint8_t* buffer = (const Uint8_t *) buf;
  Int32_t pos = offset;

  Uint64_t a =
      (((Uint32_t) buffer[pos + 0]) << 24) |
      (((Uint32_t) buffer[pos + 1]) << 16) |
      (((Uint32_t) buffer[pos + 2]) << 8) |
      (((Uint32_t) buffer[pos + 3]));
  pos += 4;
  Uint64_t b =
      (((Uint32_t) buffer[pos + 0]) << 24) |
      (((Uint32_t) buffer[pos + 1]) << 16) |
      (((Uint32_t) buffer[pos + 2]) << 8) |
      (((Uint32_t) buffer[pos + 3]));
  pos += 4;
  p[0] = (a << 32) + (b & 0xffffffff);
}

static inline void DecodeUint64ValueBigEndian(
    const void* buf, Int32_t offset, Uint64_t* p) {
  DecodeInt64ValueBigEndian(buf, offset, (Int64_t*)p);
}

static inline void DecodeFloat32ValueBigEndian(
    const void* buf, Int32_t offset, Float32_t* p) {
  DecodeInt32ValueBigEndian(buf, offset, (Int32_t*)p);
}

static inline void DecodeFloat64ValueBigEndian(
    const void* buf, Int32_t offset, Float64_t* p) {
  DecodeInt64ValueBigEndian(buf, offset, (Int64_t*)p);
}


bool CanDriverZlgCanNetFd::device_is_open_ = false;

CanDriverZlgCanNetFd::CanDriverZlgCanNetFd() {
  channel_is_open_ = false;
  sockfd_working_ = -1;
  channel_id_ = 0;

  can_frames_buff_idx_ = 0;
}

CanDriverZlgCanNetFd::~CanDriverZlgCanNetFd() {
  CloseChannel();
}

bool CanDriverZlgCanNetFd::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open ZLG CAN NET device -------->";

  LOG_INFO(3) << "<-------- Open ZLG CAN NET device ---------";

  return (true);
}

bool CanDriverZlgCanNetFd::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
  }

  return (true);
}

bool CanDriverZlgCanNetFd::OpenChannel(const CanChannelParam& param) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (channel_is_open_) {
    LOG_WARN << "This channel has been opened.";
    return (true);
  }
  if ((param.channel < 0) || (param.channel > 3)) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }

  fd_set fd_set_read;
  fd_set fd_set_write;
  struct timeval timeout;
  Int32_t err = 0;
  Uint32_t errlen = sizeof(err);

  // int socket(int domain, int type, int protocol);
  // domain:   该参数一般被设置为AF_INET，表示使用的是IPv4地址。
  //           还有更多选项可以利用man查看该函数
  // type:     该参数也有很多选项，例如SOCK_STREAM表示面向流的传输协议，
  //           SOCK_DGRAM表示数据报，我们这里实现的是TCP，因此选用SOCK_STREAM，
  //           如果实现UDP可选SOCK_DGRAM
  // protocol: 协议类型，一般使用默认，设置为0
  sockfd_working_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_working_ < 0) {
    LOG_ERR << "Can't create sock for working channel.";
    return (false);
  }

  struct sockaddr_in serveraddr;
  common::com_memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = inet_addr(param.can_net.ip_addr); //ip address
  Int32_t socket_flag = 0;
  // connect to working channel
  serveraddr.sin_port = htons(param.can_net.port);
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  unsigned long ul2=1;
  //设置成非阻塞模式
  int retval2 = ioctlsocket(sockfd_working_, FIONBIO, (unsigned long *)&ul2);
  if (retval2 == SOCKET_ERROR) {
      LOG_ERR << "ioctlsocket called failed";
      return false;
  }
#else
  socket_flag = fcntl(sockfd_working_, F_GETFL, 0);
  fcntl(sockfd_working_, F_SETFL, socket_flag | O_NONBLOCK);
#endif
  Int32_t connect_ret = connect(sockfd_working_,
                                (struct sockaddr*)&serveraddr,
                                sizeof(struct sockaddr));
  if (0 != connect_ret) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    if (WSAGetLastError() == WSAEWOULDBLOCK) {
#else
    if (EINPROGRESS == errno) {
#endif
      LOG_INFO(3) << "Doing connection.";
      // 正在处理连接
      FD_ZERO(&fd_set_read);
      FD_ZERO(&fd_set_write);
      FD_SET(sockfd_working_, &fd_set_read);
      FD_SET(sockfd_working_, &fd_set_write);
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      Int32_t rc = select(sockfd_working_+ 1,
                  &fd_set_read, &fd_set_write, NULL, &timeout);
      // select调用失败
      if (rc < 0) {
        LOG_ERR << "Failed to connect to working channel, error="
                << strerror(errno);
        close(sockfd_working_);
        sockfd_working_ = -1;
        return false;
      }
      // 连接超时
      if (0 == rc) {
        LOG_ERR << "Failed to connect to working channel, timeout.";
        close(sockfd_working_);
        sockfd_working_ = -1;
        return false;
      }
      // 当连接建立遇到错误时，描述符变为即可读，也可写，rc=2
      // 遇到这种情况，可调用getsockopt函数
      if (2 == rc) {
        int ret = -1;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        ret = getsockopt(sockfd_working_, SOL_SOCKET, SO_ERROR,(char*) &err, (int*)&errlen);
#else
        ret = getsockopt(sockfd_working_, SOL_SOCKET, SO_ERROR, &err, &errlen);
#endif
        if (-1 == ret) {
          LOG_ERR << "Failed to getsockopt(SO_ERROR), error="
                  << strerror(errno);
          close(sockfd_working_);
          sockfd_working_ = -1;
          return false;
        }
        if (err) {
          LOG_ERR << "Failed to connect to working channel, error="
                  << strerror(err);
          close(sockfd_working_);
          sockfd_working_ = -1;
          return false;
        }
      }
      // 当连接成功建立时，描述符变成可写,rc=1
      if ((1 == rc) && FD_ISSET(sockfd_working_, &fd_set_write)) {
        connect_ret = rc;
        //printf("Connect success\n");
        LOG_INFO(3) << "Succeeded to connecte to working channel.";
      }
    }
  }
  if (connect_ret < 0) {
    LOG_ERR << "Can't connect to server for working channel.";
    close(sockfd_working_);
    sockfd_working_ = -1;
    return (false);
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  unsigned long ul3=1;
  ioctlsocket(sockfd_working_, FIONBIO, (unsigned long *)&ul3);
#else
  socket_flag = fcntl(sockfd_working_, F_GETFL, 0);
  fcntl(sockfd_working_, F_SETFL, socket_flag & (~O_NONBLOCK));
#endif
  channel_is_open_ = true;

  channel_id_ = param.channel;

  can_frames_buff_idx_ = 0;

  return (true);
}

bool CanDriverZlgCanNetFd::CloseChannel() {
  if (channel_is_open_) {
    Int32_t ret = close(sockfd_working_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for working channel.";
    }
    sockfd_working_ = -1;

    channel_is_open_ = false;
  }

  return (true);
}

Int32_t CanDriverZlgCanNetFd::Send(const CanFrame* frame, Int32_t frame_num) {
  if (!channel_is_open_) {
    LOG_ERR << "This channel is close";
    return (-1);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    const CanFrame& fr = frame[i];
    Uint8_t data_buff[32] = { 0 };

    /// (包格式中若无特殊说明,均采用大端格式传输)

    /// 包头
    // 起始标识, 固定为 0x55
    data_buff[0] = 0x55;
    // 包类型:
    // 0x00: CAN 数据包, 数据中最大报文个数分上传和下发,
    //       上传由配置设定,下发每次最多 50 个 CAN 报文
    // 0x01: CAN FD 数据包,数据中最大报文个数分上传和下发,
    //       上传由配置设定,下发每次最多 18 个 CANFD 报文
    // others: See documents
    data_buff[1] = 0x00;
    // 类型参数
    // 当包类型为CAN数据包时, Bit0:是否压缩, 1 为压缩, 采用 zlib 压缩,
    // 仅压缩数据区, 数据长度为 n * CAN 报文长度(n 为报文个数)
    data_buff[2] = 0x00;
    // 保留
    data_buff[3] = 0x00;
    // 指示数据区长度
    data_buff[4] = 0x00;
    data_buff[5] = 8/*时间戳*/ + 4/*报文ID*/ + 2/*报文信息*/ +
        1/*报文通道*/ + 1/*数据长度*/ + 8/*数据*/;

    /// 数据区
    // 时间戳, 当前报文接收/发送时间,单位 us;
    // 当发送报文时,报文信息中 SndDelay=1, 则该参数最低 4 字节表示,
    // 发送后延时时间,单位 100us.
    Uint64_t timestamp = 0;
    data_buff[6] = (timestamp >> 56) & 0xFF;
    data_buff[7] = (timestamp >> 48) & 0xFF;
    data_buff[8] = (timestamp >> 40) & 0xFF;
    data_buff[9] = (timestamp >> 32) & 0xFF;
    data_buff[10] = (timestamp >> 24) & 0xFF;
    data_buff[11] = (timestamp >> 16) & 0xFF;
    data_buff[12] = (timestamp >> 8) & 0xFF;
    data_buff[13] = (timestamp & 0xFF);

    // 报文 ID, Bit0~28:报文 ID,标准帧为 11 位,扩展帧为 29 位; Bit29~31:保留,为 0;
    data_buff[14] = fr.id >> 24;
    data_buff[15] = fr.id >> 16;
    data_buff[16] = fr.id >> 8;
    data_buff[17] = fr.id & 0xFF;

    // 报文信息
    // [bit8] : BRS, 1-CANFD 加速, 0-不加速(CANFD 有效)
    /// data_buff[18] |= (0x00 & 0x01) << 0;
    // [bit9] : ESI ,1-被动错误,0-主动错误; ESI 仅 CANFD 接收有效
    /// data_buff[18] |= (0x00 & 0x01) << 1;
    // [bit10] : SndDelay ,1-发送后延时, 0-普通发送
    /// data_buff[18] |= (0x00 & 0x01) << 2;
    // [bit15:11]:保留,为 0;
    /// data_buff[18] |= (0x00 & 0x1F) << 3;
    // [bit1:0] : 发送类型(仅发送有效,接收为 0), 0: 正常发送; 1: 单次发送; 2: 自发自收
    /// data_buff[19] |= (0x00 & 0x03) << 0;
    // [bit2] : EchoFlag, 发送时该位为 0;
    //          接收时该位表示回显报, 文标识, 1-发送回显报文, 0-普通报文;
    /// data_buff[19] |= (0x00 & 0x01) << 2;
    // [bit3] : ECHO, 1-发送回显, 0-发送不回显
    /// data_buff[19] |= (0x00 & 0x01) << 3;
    // [bit4] : FD, 1-CANFD, 0-CAN;
    /// data_buff[19] |= (0x00 & 0x01) << 4;
    // [bit5] : RTR, 1-远程帧, 0-数据帧;
    if (fr.RTR) {
      data_buff[19] |= (0x01 & 0x01) << 5;
    }
    // [bit6] : EXT, 1-扩展帧, 0-标准帧;
    if (fr.EXT) {
      data_buff[19] |= (0x01 & 0x01) << 6;
    }
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    /// data_buff[19] |= (0x00 & 0x01) << 7;

    // 报文通道, CAN(FD)通道, 取值:0~设备通道数
    data_buff[20] = channel_id_ & 0xFF;
    // 报文数据长度; 取值如下: CAN 报文: 0~8; CANFD 报文: 0~8,12,16,20,24,32,48,64
    data_buff[21] = (fr.data_len & 0xFF);

    // 报文数据; CAN: 报文数据长度为 8 字节; CAN FD : 报文数据长度 64 字节;
    common::com_memcpy(&data_buff[22], fr.data, 8);

    /// 校验码
    /// 采用 BCC(异或校验法), 校验范围从起始标识开始, 直到校验码前一字节为止。
    for (Int32_t i = 0; i < 30; i++ ) {
      data_buff[30] ^= data_buff[i];
    }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    Int32_t bytes =send(sockfd_working_, (const char*)data_buff, 31, 0);
#else
    Int32_t bytes = send(sockfd_working_, data_buff, 31, 0);
#endif
    if (bytes < 31) {
      LOG_ERR << "Failed to write can message.";
      continue;
    }

    send_count++;
  }

  return (send_count);
}

Int32_t CanDriverZlgCanNetFd::SendCanFd(
    const CanFdFrame* frame, Int32_t frame_num) {
  if (!channel_is_open_) {
    LOG_ERR << "This channel is close";
    return (-1);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    const CanFdFrame& fr = frame[i];
    Uint8_t data_buff[96] = { 0 };

    /// (包格式中若无特殊说明,均采用大端格式传输)

    /// 包头
    // 起始标识, 固定为 0x55
    data_buff[0] = 0x55;
    // 包类型:
    // 0x00: CAN 数据包, 数据中最大报文个数分上传和下发,
    //       上传由配置设定,下发每次最多 50 个 CAN 报文
    // 0x01: CAN FD 数据包,数据中最大报文个数分上传和下发,
    //       上传由配置设定,下发每次最多 18 个 CANFD 报文
    // others: See documents
    if (fr.FD) {
      data_buff[1] = 0x01;
    } else {
      data_buff[1] = 0x00;
    }
    // 类型参数
    // 包类型参数定义与 CAN 数据包一致。
    // 当包类型为CAN数据包时, Bit0:是否压缩, 1 为压缩, 采用 zlib 压缩,
    // 仅压缩数据区, 数据长度为 n * CAN FD 报文长度(n 为报文个数)
    data_buff[2] = 0x00;
    // 保留
    data_buff[3] = 0x00;
    // 指示数据区长度
    data_buff[4] = 0x00;
    if (fr.FD) {
      data_buff[5] = 8/*时间戳*/ + 4/*报文ID*/ + 2/*报文信息*/ +
          1/*报文通道*/ + 1/*数据长度*/ + 64/*数据*/;
    } else {
      data_buff[5] = 8/*时间戳*/ + 4/*报文ID*/ + 2/*报文信息*/ +
          1/*报文通道*/ + 1/*数据长度*/ + 8/*数据*/;
    }

    /// 数据区
    // 时间戳, 当前报文接收/发送时间,单位 us;
    // 当发送报文时,报文信息中 SndDelay=1, 则该参数最低 4 字节表示,
    // 发送后延时时间,单位 100us.
    Uint64_t timestamp = 0;
    data_buff[6] = (timestamp >> 56) & 0xFF;
    data_buff[7] = (timestamp >> 48) & 0xFF;
    data_buff[8] = (timestamp >> 40) & 0xFF;
    data_buff[9] = (timestamp >> 32) & 0xFF;
    data_buff[10] = (timestamp >> 24) & 0xFF;
    data_buff[11] = (timestamp >> 16) & 0xFF;
    data_buff[12] = (timestamp >> 8) & 0xFF;
    data_buff[13] = (timestamp & 0xFF);

    // 报文 ID, Bit0~28:报文 ID,标准帧为 11 位,扩展帧为 29 位; Bit29~31:保留,为 0;
    data_buff[14] = fr.id >> 24;
    data_buff[15] = fr.id >> 16;
    data_buff[16] = fr.id >> 8;
    data_buff[17] = fr.id & 0xFF;

    // 报文信息
    // [bit8] : BRS, 1-CANFD 加速, 0-不加速(CANFD 有效)
    if (fr.FD && fr.BRS) {
      data_buff[18] |= (0x01 & 0x01) << 0;
    }
    // [bit9] : ESI ,1-被动错误,0-主动错误; ESI 仅 CANFD 接收有效
    /// data_buff[18] |= (0x00 & 0x01) << 1;
    // [bit10] : SndDelay ,1-发送后延时, 0-普通发送
    /// data_buff[18] |= (0x00 & 0x01) << 2;
    // [bit15:11]:保留,为 0;
    /// data_buff[18] |= (0x00 & 0x1F) << 3;
    // [bit1:0] : 发送类型(仅发送有效,接收为 0), 0: 正常发送; 1: 单次发送; 2: 自发自收
    /// data_buff[19] |= (0x00 & 0x03) << 0;
    // [bit2] : EchoFlag, 发送时该位为 0;
    //          接收时该位表示回显报, 文标识, 1-发送回显报文, 0-普通报文;
    /// data_buff[19] |= (0x00 & 0x01) << 2;
    // [bit3] : ECHO, 1-发送回显, 0-发送不回显
    /// data_buff[19] |= (0x00 & 0x01) << 3;
    // [bit4] : FD, 1-CANFD, 0-CAN;
    if (fr.FD) {
      data_buff[19] |= (0x01 & 0x01) << 4;
    }
    // [bit5] : RTR, 1-远程帧, 0-数据帧;
    if (fr.RTR) {
      data_buff[19] |= (0x01 & 0x01) << 5;
    }
    // [bit6] : EXT, 1-扩展帧, 0-标准帧;
    if (fr.EXT) {
      data_buff[19] |= (0x01 & 0x01) << 6;
    }
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    /// data_buff[19] |= (0x00 & 0x01) << 7;

    // 报文通道, CAN(FD)通道, 取值:0~设备通道数
    data_buff[20] = channel_id_ & 0xFF;
    // 报文数据长度; 取值如下: CAN 报文: 0~8; CANFD 报文: 0~8,12,16,20,24,32,48,64
    data_buff[21] = (fr.data_len & 0xFF);

    // 报文数据; CAN: 报文数据长度为 8 字节; CAN FD : 报文数据长度 64 字节;
    if (fr.FD) {
      common::com_memcpy(&data_buff[22], fr.data, 64);
    } else {
      common::com_memcpy(&data_buff[22], fr.data, 8);
    }

    /// 校验码
    /// 采用 BCC(异或校验法), 校验范围从起始标识开始, 直到校验码前一字节为止。
    if (fr.FD) {
      for (Int32_t i = 0; i < 86; i++ ) {
        data_buff[86] ^= data_buff[i];
      }
    } else {
      for (Int32_t i = 0; i < 30; i++ ) {
        data_buff[30] ^= data_buff[i];
      }
    }

    Int32_t data_size = 31;
    if (fr.FD) {
      data_size = 87;
    }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    Int32_t bytes =send(sockfd_working_, (const char*)data_buff, data_size, 0);
#else
    Int32_t bytes = send(sockfd_working_, data_buff, data_size, 0);
#endif
    if (bytes < data_size) {
      LOG_ERR << "Failed to write can message.";
      continue;
    }

    send_count++;
  }

  return (send_count);
}


Int32_t CanDriverZlgCanNetFd::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  // 先从缓存读取数据
  Int32_t frame_num = 0;
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFdFrame* data = received_can_frames_.PopFront();
      if (Nullptr_t != data) {
        if (0 == data->FD) {
          frame[frame_num].time_stamp = data->time_stamp;
          frame[frame_num].id = data->id;
          frame[frame_num].RTR = data->RTR;
          frame[frame_num].EXT = data->EXT;
          frame[frame_num].data_len = data->data_len;
          common::com_memcpy(frame[frame_num].data, data->data, 8);

          frame_num++;
        }
      } else {
        break;
      }
    } else {
      break;
    }
  }
  if (frame_num > 0) {
    // 若缓存有数据，先返回已经读取的数据
    return (frame_num);
  }

  // 若缓存无数据，从CANNET读取
  fd_set read_fds;  //读文件操作符
  FD_ZERO(&read_fds);
  // 每次调用select之前都要重新在read_fds中设置文件描述符，
  // 因为事件发生以后，文件描述符集合将被内核修改
  FD_SET(sockfd_working_, &read_fds);
  // int select(int maxfdp,
  //            fd_set *readset,
  //            fd_set *writeset,
  //            fd_set *exceptset,
  //            struct timeval *timeout);
  // maxfdp：被监听的文件描述符的总数，
  // 它比所有文件描述符集合中的文件描述符的最大值大1，因为文件描述符是从0开始计数的；
  // readfds、writefds、exceptset：分别指向可读、可写和异常等事件对应的描述符集合。
  // timeout:用于设置select函数的超时时间，即告诉内核select等待多长时间之后就放弃等待。
  // timeout == NULL 表示等待无限长的时间
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  Int32_t ret = select(sockfd_working_+1,
                       &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    LOG_ERR << "Failed to select fds.";
    return -1;
  }

  if(!FD_ISSET(sockfd_working_, &read_fds)) {
    return -1;
  }

  Uint8_t data_buff[CAN_FRAMES_BUF_SIZE] = { 0 };
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  Int32_t bytes = recv(sockfd_working_, (char*)data_buff, sizeof(data_buff), 0);
#else
  Int32_t bytes = recv(sockfd_working_, data_buff, sizeof(data_buff), 0);
#endif

#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
  printf("\n### Receiving %d bytes from can net\n", bytes);
  for (Int32_t i = 0; i < bytes; ++i) {
    printf("%02x ", data_buff[i]);
    if (0 == ((i+1) % 8)) printf(" ");
    if (0 == ((i+1) % 16)) printf("\n");
  }
  printf("\n");
#endif

  // 解析CAN报文
  SpinCanData(data_buff, bytes);

  // 从缓存读取数据
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFdFrame* data = received_can_frames_.PopFront();
      if (Nullptr_t != data) {
        if (0 == data->FD) {
          frame[frame_num].time_stamp = data->time_stamp;
          frame[frame_num].id = data->id;
          frame[frame_num].RTR = data->RTR;
          frame[frame_num].EXT = data->EXT;
          frame[frame_num].data_len = data->data_len;
          common::com_memcpy(frame[frame_num].data, data->data, 8);

          frame_num++;
        }
      } else {
        break;
      }
    } else {
      break;
    }
  }

  return (frame_num);
}

Int32_t CanDriverZlgCanNetFd::ReadCanFdWait(
    CanFdFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  // 先从缓存读取数据
  Int32_t frame_num = 0;
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFdFrame* data = received_can_frames_.PopFront();
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
  if (frame_num > 0) {
    // 若缓存有数据，先返回已经读取的数据
    return (frame_num);
  }

  // 若缓存无数据，从CANNET读取
  fd_set read_fds;  //读文件操作符
  FD_ZERO(&read_fds);
  // 每次调用select之前都要重新在read_fds中设置文件描述符，
  // 因为事件发生以后，文件描述符集合将被内核修改
  FD_SET(sockfd_working_, &read_fds);
  // int select(int maxfdp,
  //            fd_set *readset,
  //            fd_set *writeset,
  //            fd_set *exceptset,
  //            struct timeval *timeout);
  // maxfdp：被监听的文件描述符的总数，
  // 它比所有文件描述符集合中的文件描述符的最大值大1，因为文件描述符是从0开始计数的；
  // readfds、writefds、exceptset：分别指向可读、可写和异常等事件对应的描述符集合。
  // timeout:用于设置select函数的超时时间，即告诉内核select等待多长时间之后就放弃等待。
  // timeout == NULL 表示等待无限长的时间
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  Int32_t ret = select(sockfd_working_+1,
                       &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    LOG_ERR << "Failed to select fds.";
    return -1;
  }

  if(!FD_ISSET(sockfd_working_, &read_fds)) {
    return -1;
  }

  Uint8_t data_buff[CAN_FRAMES_BUF_SIZE] = { 0 };
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  Int32_t bytes = recv(sockfd_working_, (char*)data_buff, sizeof(data_buff), 0);
#else
  Int32_t bytes = recv(sockfd_working_, data_buff, sizeof(data_buff), 0);
#endif

#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
  printf("\n### Receiving %d bytes from can net fd\n", bytes);
  for (Int32_t i = 0; i < bytes; ++i) {
    printf("%02x ", data_buff[i]);
    if (0 == ((i+1) % 8)) printf(" ");
    if (0 == ((i+1) % 16)) printf("\n");
  }
  printf("\n");
#endif

  // 解析CAN报文
  SpinCanData(data_buff, bytes);

  // 从缓存读取数据
  while (!received_can_frames_.Empty()) {
    if (frame_num < max_frame_num) {
      const CanFdFrame* data = received_can_frames_.PopFront();
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

  return (frame_num);
}

static const Uint8_t s_can_msg_magic_word[] = {
  0x55
};
void CanDriverZlgCanNetFd::SpinCanData(
    const Uint8_t* msg_buf, Int32_t msg_size) {
  Uint8_t* data_buff = can_frames_buff_;
  Int32_t* data_idx = &can_frames_buff_idx_;
  CanMsgHeader* can_msg_header = &can_msg_header_;

#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
  std::cout << "### Receiving " << msg_size
            << " bytes from CANFDNET. The message header is:" << std::endl;
  for (Int32_t i = 0; i < std::min(msg_size, 6); ++i) {
    printf("%02X ", msg_buf[i]);
  }
  printf("\n");
#endif

  Int32_t msg_magic_size = sizeof(s_can_msg_magic_word);
  for (Int32_t i = 0; i < msg_size; ++i) {
    if ((*data_idx) < msg_magic_size) {
      if (s_can_msg_magic_word[(*data_idx)] == msg_buf[i]) {
        data_buff[(*data_idx)] = msg_buf[i];
        (*data_idx)++;
      } else {
        (*data_idx) = 0;
      }
    } else if ((*data_idx) < CAN_MSG_HEADER_SIZE) {
      data_buff[(*data_idx)] = msg_buf[i];
      (*data_idx)++;

      if (CAN_MSG_HEADER_SIZE == (*data_idx)) {
        // 起始标识, 固定为 0x55
        can_msg_header->magic = data_buff[0];
        // 指示该包类型, 0x00 ~ CAN 数据包, 0x01 ~ CAN FD 数据包, others ~ Sed documents
        can_msg_header->package_type = data_buff[1];
        // 类型参数
        can_msg_header->package_param = data_buff[2];
        // 数据长度
        can_msg_header->data_len = (data_buff[4] << 8) | data_buff[5];

        can_msg_header->total_msg_size =
            CAN_MSG_HEADER_SIZE + can_msg_header->data_len + 1/*校验码*/;
        if (can_msg_header->total_msg_size > CAN_FRAMES_BUF_SIZE) {
          // 丢弃此数据
          (*data_idx) = 0;
          LOG_ERR << "The total message size(" << can_msg_header->total_msg_size
                  << ") exceed the maximum buffer size.";
        }

        if (0x00 == can_msg_header->package_type) {
          // CAN 数据包
          if (0 != (can_msg_header->data_len % (CAN_FRAME_HEADER_SIZE+8))) {
            // 丢弃此数据
            (*data_idx) = 0;
            LOG_ERR << "Invalid data length("
                    << can_msg_header->data_len << ") of CAN Frames.";
          }
        } else if (0x01 == can_msg_header->package_type) {
          // CAN FD 数据包
          if (0 != (can_msg_header->data_len % (CAN_FRAME_HEADER_SIZE+64))) {
            // 丢弃此数据
            (*data_idx) = 0;
            LOG_ERR << "Invalid data length("
                    << can_msg_header->data_len << ") of CAN FD Frames.";
          }
        } else if (0x02 == can_msg_header->package_type) {
          // 定时发送数据包
        } else if (0x03 == can_msg_header->package_type) {
          // 总线利用率指示包
        } else if (0x04 == can_msg_header->package_type) {
          // 设备状态包
        } else if (0x05 == can_msg_header->package_type) {
          // GPS 数据包
        } else if (0x06 == can_msg_header->package_type) {
          // LIN 数据包
        } else if (0x50 == can_msg_header->package_type) {
          // 请求应答包
        } else if (0x51 == can_msg_header->package_type) {
          // 配置数据包
        } else {
          // 无效数据包类型
          // 丢弃此数据
          (*data_idx) = 0;
          LOG_ERR << "Invalid package type("
                  << can_msg_header->package_type << ").";
        }

#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
        printf("Magic=0x%02X"
               ", package_type=0x%02X, package_param=0x%02X"
               ", data_len=%d"
               "\n",
               can_msg_header->magic,
               can_msg_header->package_type, can_msg_header->package_param,
               can_msg_header->data_len);
#endif
      }
    } else {
      if ((*data_idx) < (can_msg_header->total_msg_size-1)) {
        data_buff[(*data_idx)] = msg_buf[i];
        (*data_idx)++;
      } else if ((*data_idx) == (can_msg_header->total_msg_size-1)) {
        data_buff[(*data_idx)] = msg_buf[i];
        (*data_idx)++;

        // complete message frame

        // 校验码
        Uint8_t bcc = 0;
        for (Int32_t j = 0; j < (*data_idx-1); ++j) {
          bcc ^= data_buff[j];
        }
        if (bcc != data_buff[*data_idx-1]) {
          LOG_ERR << "Failed to check BCC, required=" << data_buff[*data_idx-1]
                  << ", actual=" << bcc;
        } else {
          // printf("Success to check bcc[0x%02X]\n", bcc);
          if (0x00 == can_msg_header->package_type) {
            // CAN 数据包
            ParseCanFrame(*can_msg_header, data_buff, *data_idx);
          } else if (0x01 == can_msg_header->package_type) {
            // CAN FD 数据包
            ParseCanFrame(*can_msg_header, data_buff, *data_idx);
          } else {
            /// TODO: 其它类型数据包(暂时忽略)
          }
        }

        (*data_idx) = 0;
      } else {
        (*data_idx) = 0;
        LOG_ERR << "Unexpected case occurs.";
      }
    }
  }
}

void CanDriverZlgCanNetFd::ParseCanFrame(
    const CanMsgHeader& header, const Uint8_t* msg_buf, Int32_t msg_size) {
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
  printf(">>>> Parse can frame >>>>>> \n");
#endif

  Int32_t data_frame_size = CAN_FRAME_HEADER_SIZE+8;
  if (0x01 == header.package_type) {
    data_frame_size = CAN_FRAME_HEADER_SIZE+64;
  }

  Int32_t frame_num = header.data_len / data_frame_size;
  for (Int32_t i = 0; i < frame_num; ++i) {
    /// 解析数据
    const Uint8_t* data = &msg_buf[CAN_MSG_HEADER_SIZE + i*data_frame_size];
    Int32_t data_idx = 0;

    // 时间戳, 当前报文接收/发送时间,单位 us
    Uint64_t timestamp = 0;
    DecodeUint64ValueBigEndian(data, data_idx, &timestamp);
    data_idx += 8;

    // 报文 ID, Bit0~28:报文ID, 标准帧为11位, 扩展帧为29位, Bit29~31:保留为0
    Uint32_t id = 0;
    DecodeUint32ValueBigEndian(data, data_idx, &id);
    data_idx += 4;

    // 报文信息
    Uint16_t frame_info = 0;
    DecodeUint16ValueBigEndian(data, data_idx, &frame_info);
    data_idx += 2;

    // CAN(FD)通道,取值:0~设备通道数
    Uint8_t channel = data[data_idx];
    data_idx += 1;

    // 报文数据长度;取值如下
    Uint8_t data_len = data[data_idx];
    data_idx += 1;


    /// 判断异常
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    if ((frame_info >> 7) & 0x01) {
      LOG_ERR << "The " << i << "th CAN frame is ERR frame.";
      continue;
    }

    if (channel != channel_id_) {
      LOG_ERR << "Receiving CAN frame from other channel " << channel
              << ", the current channel is " << channel_id_ << ".";
      continue;
    }


    /// 获取CAN帧的存储空间
    CanFdFrame* fr = received_can_frames_.AllocateOverride();
    if (Nullptr_t == fr) {
      LOG_ERR << "Can't add can frames, buffer is full.";
      continue;
    }


    /// 分析报文信息
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
    printf("Frame info:");
#endif
    // [bit1:0] : 发送类型(仅发送有效,接收为 0), 0: 正常发送; 1: 单次发送; 2: 自发自收
    // [bit2] : EchoFlag, 发送时该位为 0;
    //          接收时该位表示回显报, 文标识, 1-发送回显报文, 0-普通报文;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
    if ((frame_info >> 2) & 0x01) {
      printf(" [Is Echo]");
    } else {
      printf(" [Not Echo]");
    }
#endif
    // [bit3] : ECHO, 1-发送回显, 0-发送不回显
    // [bit4] : FD, 1-CANFD, 0-CAN;
    if ((frame_info >> 4) & 0x01) {
      fr->FD = 1;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Is FD]");
#endif
    } else {
      fr->FD = 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Not FD]");
#endif
    }
    // [bit5] : RTR, 1-远程帧, 0-数据帧;
    if ((frame_info >> 5) & 0x01) {
      fr->RTR = 1;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Is RTR]");
#endif
    } else {
      fr->RTR = 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Not RTR]");
#endif
    }
    // [bit6] : EXT, 1-扩展帧, 0-标准帧;
    if ((frame_info >> 6) & 0x01) {
      fr->EXT = 1;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Is EXT]");
#endif
    } else {
      fr->EXT = 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Not EXT]");
#endif
    }
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    // [bit8] : BRS, 1-CANFD 加速, 0-不加速(CANFD 有效)
    if ((frame_info >> 8) & 0x01) {
      fr->BRS = 1;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Is BRS]");
#endif
    } else {
      fr->BRS = 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Not BRS]");
#endif
    }
    // [bit9] : ESI ,1-被动错误,0-主动错误; ESI 仅 CANFD 接收有效
    if ((frame_info >> 9) & 0x01) {
      fr->ESI = 1;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Is ESI]");
#endif
    } else {
      fr->ESI = 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
      printf(" [Not ESI]");
#endif
    }
    // [bit10] : SndDelay ,1-发送后延时, 0-普通发送
    // [bit15:11]:保留,为 0;
#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
    printf("\n");
#endif

#if ENABLE_CAN_DRIVER_ZLGCANNET_FD_TRACE
    printf("data_len=%d\n", data_len);
#endif


    /// 保存CAN数据
    fr->time_stamp = common::GetClockNowMs();
    fr->id = id;
    fr->data_len = data_len;
    if (0x01 == header.package_type) {
      common::com_memcpy(fr->data, &data[data_idx], 64);
    } else {
      common::com_memcpy(fr->data, &data[data_idx], 8);
    }
  }
}

#endif  // #if (ENABLE_CAN_DEV_ZLGCANNET)


}  // namespace can_dev
}  // namespace phoenix
