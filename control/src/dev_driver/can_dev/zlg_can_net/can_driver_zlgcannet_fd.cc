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


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_ZLGCANNET)

bool CanDriverZlgCanNetFd::device_is_open_ = false;

CanDriverZlgCanNetFd::CanDriverZlgCanNetFd() {
  channel_is_open_ = false;
  sockfd_working_ = -1;
  channel_id_ = 0;
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

  if (max_frame_num > 18) {
    max_frame_num = 18;
  }
  Uint8_t data_buff[6/*包头*/+80*18/*数据区*/+1/*校验码*/+1/*Reserved*/] = { 0 };

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
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  Int32_t bytes = recv(sockfd_working_, (char*)data_buff, sizeof(data_buff), 0);
#else
  Int32_t bytes = recv(sockfd_working_, data_buff, sizeof(data_buff), 0);
#endif

  if (bytes < (6/*包头*/+24/*数据区*/+1/*校验码*/)) {
    // LOG_ERR << "Failed to receive can message.";
    return (-1);
  }
  // 起始标识, 固定为 0x55
  if (0x55 != data_buff[0]) {
    return (-1);
  }
  // 指示该包类型, 0x00 ~ CAN 数据包, 0x01 ~ CAN FD 数据包, others ~ Sed documents
  if (0x00 != data_buff[1]) {
    return (-1);
  }
  // 类型参数
  /// data_buff[2];
  // 保留
  /// data_buff[3];
  // 数据长度
  Int32_t data_area_size = (data_buff[4] << 8) | data_buff[5];
  if (0 != (data_area_size % 24)) {
    LOG_ERR << "Detected invalid data area size " << data_area_size
            << ", is not the multiples of 24.";
    return (-1);
  }
  if (data_area_size != (bytes-7)) {
    LOG_ERR << "Detected invalid data area size " << data_area_size
            << ", not equal total bag size ("<< bytes <<") minus 7.";
    return (-1);
  }
  // 校验码
  Uint8_t bcc = 0;
  for (Int32_t i = 0; i < (bytes-1); ++i) {
    bcc ^= data_buff[i];
  }
  if (bcc != data_buff[bytes-1]) {
    LOG_ERR << "Failed to check BCC, required=" << data_buff[bytes-1]
            << ", actual=" << bcc;
    return (-1);
  }

  Int32_t frame_num = (bytes-7) / 24;
  Int32_t frm_idx = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    Uint8_t* buff = &data_buff[6 + i*24];

    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    if ((data_buff[13] >> 7) & 0x01) {
      LOG_ERR << "The " << i << "th CAN frame is ERR frame.";
      continue;
    }

    CanFrame& fr = frame[frm_idx];
    frm_idx++;

    // 时间戳, 当前报文接收/发送时间,单位 us
    Uint64_t timestamp = 0;
    Uint64_t a =
        (((Uint32_t) buff[0]) << 24) +
        (((Uint32_t) buff[1]) << 16) +
        (((Uint32_t) buff[2]) << 8) +
        ((Uint32_t)  buff[3]);
    Uint64_t b =
        (((Uint32_t) buff[4]) << 24) +
        (((Uint32_t) buff[5]) << 16) +
        (((Uint32_t) buff[6]) << 8) +
        ((Uint32_t)  buff[7]);
    timestamp = (a << 32) + (b & 0xFFFFFFFF);
    fr.time_stamp = common::GetClockNowMs();

    // 报文 ID, Bit0~28:报文ID, 标准帧为11位, 扩展帧为29位, Bit29~31:保留为0
    fr.id = (buff[8] << 24) | (buff[9] << 16) | (buff[10] << 8) | buff[11];

    // 报文信息
    // [bit8] : BRS, 1-CANFD 加速, 0-不加速(CANFD 有效)
    /// if (data_buff[12] & 0x01) {
    ///   fr.BRS = 1;
    /// } else {
    ///   fr.BRS = 0;
    /// }
    // [bit9] : ESI ,1-被动错误,0-主动错误; ESI 仅 CANFD 接收有效
    /// if ((data_buff[12] >> 1) & 0x01) {
    ///   fr.ESI = 1;
    /// } else {
    ///   fr.ESI = 0;
    /// }
    // [bit10] : SndDelay ,1-发送后延时, 0-普通发送
    // [bit15:11]:保留,为 0;
    // [bit1:0] : 发送类型(仅发送有效,接收为 0), 0: 正常发送; 1: 单次发送; 2: 自发自收
    // [bit2] : EchoFlag, 发送时该位为 0;
    //          接收时该位表示回显报, 文标识, 1-发送回显报文, 0-普通报文;
    // [bit3] : ECHO, 1-发送回显, 0-发送不回显
    // [bit4] : FD, 1-CANFD, 0-CAN;
    /// if ((data_buff[13] >> 4) & 0x01) {
    ///   fr.FD = 1;
    /// } else {
    ///   fr.FD = 0;
    /// }
    // [bit5] : RTR, 1-远程帧, 0-数据帧;
    if ((data_buff[13] >> 5) & 0x01) {
      fr.RTR = 1;
    } else {
      fr.RTR = 0;
    }
    // [bit6] : EXT, 1-扩展帧, 0-标准帧;
    if ((data_buff[13] >> 6) & 0x01) {
      fr.EXT = 1;
    } else {
      fr.EXT = 0;
    }
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);

    // CAN(FD)通道,取值:0~设备通道数
    if (data_buff[14] != channel_id_) {
      LOG_ERR << "Receiving CAN frame from other channel " << data_buff[14];
    }
    // 报文数据长度;取值如下
    fr.data_len = buff[15] & 0xFF;

    common::com_memcpy(fr.data, &buff[16], 8);
  }

  return (frm_idx);
}

Int32_t CanDriverZlgCanNetFd::ReadCanFdWait(
    CanFdFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  if (max_frame_num > 18) {
    max_frame_num = 18;
  }
  Uint8_t data_buff[6/*包头*/+80*18/*数据区*/+1/*校验码*/+1/*Reserved*/] = { 0 };

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
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  Int32_t bytes = recv(sockfd_working_, (char*)data_buff, sizeof(data_buff), 0);
#else
  Int32_t bytes = recv(sockfd_working_, data_buff, sizeof(data_buff), 0);
#endif

  if (bytes < (6/*包头*/+24/*min数据区*/+1/*校验码*/)) {
    // LOG_ERR << "Failed to receive can message.";
    return (-1);
  }
  // 起始标识, 固定为 0x55
  if (0x55 != data_buff[0]) {
    return (-1);
  }
  // 指示该包类型, 0x00 ~ CAN 数据包, 0x01 ~ CAN FD 数据包, others ~ Sed documents
  if ((0x00 != data_buff[1]) && (0x01 != data_buff[1])) {
    return (-1);
  }
  bool is_canfd = false;
  Int32_t data_frame_size = 24;
  if (0x01 == data_buff[1]) {
    is_canfd = true;
    data_frame_size = 80;
  }
  // 类型参数
  /// data_buff[2];
  // 保留
  /// data_buff[3];
  // 数据长度
  Int32_t data_area_size = (data_buff[4] << 8) | data_buff[5];
  if (0 != (data_area_size % data_frame_size)) {
    LOG_ERR << "Detected invalid data area size " << data_area_size
            << ", is not the multiples of (" << data_frame_size << ").";
    return (-1);
  }
  if (data_area_size != (bytes-7)) {
    LOG_ERR << "Detected invalid data area size " << data_area_size
            << ", not equal total bag size ("<< bytes <<") minus 7.";
    return (-1);
  }
  // 校验码
  Uint8_t bcc = 0;
  for (Int32_t i = 0; i < (bytes-1); ++i) {
    bcc ^= data_buff[i];
  }
  if (bcc != data_buff[bytes-1]) {
    LOG_ERR << "Failed to check BCC, required=" << data_buff[bytes-1]
            << ", actual=" << bcc;
    return (-1);
  }

  Int32_t frame_num = (bytes-7) / data_frame_size;
  Int32_t frm_idx = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    Uint8_t* buff = &data_buff[6 + i*data_frame_size];

    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);
    if ((data_buff[13] >> 7) & 0x01) {
      LOG_ERR << "The " << i << "th CAN frame is ERR frame.";
      continue;
    }

    CanFdFrame& fr = frame[frm_idx];
    frm_idx++;

    // 时间戳, 当前报文接收/发送时间,单位 us
    Uint64_t timestamp = 0;
    Uint64_t a =
        (((Uint32_t) buff[0]) << 24) +
        (((Uint32_t) buff[1]) << 16) +
        (((Uint32_t) buff[2]) << 8) +
        ((Uint32_t)  buff[3]);
    Uint64_t b =
        (((Uint32_t) buff[4]) << 24) +
        (((Uint32_t) buff[5]) << 16) +
        (((Uint32_t) buff[6]) << 8) +
        ((Uint32_t)  buff[7]);
    timestamp = (a << 32) + (b & 0xFFFFFFFF);
    fr.time_stamp = common::GetClockNowMs();

    // 报文 ID, Bit0~28:报文ID, 标准帧为11位, 扩展帧为29位, Bit29~31:保留为0
    fr.id = (buff[8] << 24) | (buff[9] << 16) | (buff[10] << 8) | buff[11];

    // 报文信息
    // [bit8] : BRS, 1-CANFD 加速, 0-不加速(CANFD 有效)
    if (data_buff[12] & 0x01) {
      fr.BRS = 1;
    } else {
      fr.BRS = 0;
    }
    // [bit9] : ESI ,1-被动错误,0-主动错误; ESI 仅 CANFD 接收有效
    if ((data_buff[12] >> 1) & 0x01) {
      fr.ESI = 1;
    } else {
      fr.ESI = 0;
    }
    // [bit10] : SndDelay ,1-发送后延时, 0-普通发送
    // [bit15:11]:保留,为 0;
    // [bit1:0] : 发送类型(仅发送有效,接收为 0), 0: 正常发送; 1: 单次发送; 2: 自发自收
    // [bit2] : EchoFlag, 发送时该位为 0;
    //          接收时该位表示回显报, 文标识, 1-发送回显报文, 0-普通报文;
    // [bit3] : ECHO, 1-发送回显, 0-发送不回显
    // [bit4] : FD, 1-CANFD, 0-CAN;
    if ((data_buff[13] >> 4) & 0x01) {
      fr.FD = 1;
    } else {
      fr.FD = 0;
    }
    // [bit5] : RTR, 1-远程帧, 0-数据帧;
    if ((data_buff[13] >> 5) & 0x01) {
      fr.RTR = 1;
    } else {
      fr.RTR = 0;
    }
    // [bit6] : EXT, 1-扩展帧, 0-标准帧;
    if ((data_buff[13] >> 6) & 0x01) {
      fr.EXT = 1;
    } else {
      fr.EXT = 0;
    }
    // [bit7] : ERR, 1-错误报文, 0-正常报文, (接收有效);

    // CAN(FD)通道,取值:0~设备通道数
    if (data_buff[14] != channel_id_) {
      LOG_ERR << "Receiving CAN frame from other channel " << data_buff[14];
    }
    // 报文数据长度;取值如下
    fr.data_len = buff[15] & 0xFF;

    if (is_canfd) {
      common::com_memcpy(fr.data, &buff[16], 64);
    } else {
      common::com_memcpy(fr.data, &buff[16], 8);
    }
  }

  return (frm_idx);
}

#endif  // #if (ENABLE_CAN_DEV_ZLGCANNET)


}  // namespace can_dev
}  // namespace phoenix
