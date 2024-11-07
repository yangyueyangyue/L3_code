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
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_ZLGCANNET)

bool CanDriverZlgCanNet::device_is_open_ = false;

CanDriverZlgCanNet::CanDriverZlgCanNet() {
  channel_is_open_ = false;
  sockfd_notifying_ = -1;
  sockfd_working_ = -1;
}

CanDriverZlgCanNet::~CanDriverZlgCanNet() {
  CloseChannel();
}

bool CanDriverZlgCanNet::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open ZLG CAN NET device -------->";

  LOG_INFO(3) << "<-------- Open ZLG CAN NET device ---------";

  return (true);
}

bool CanDriverZlgCanNet::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
  }

  return (true);
}

bool CanDriverZlgCanNet::OpenChannel(const CanChannelParam& param) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (channel_is_open_) {
    LOG_WARN << "This channel has been opened.";
    return (true);
  }
  if ((param.channel < 0) || (param.channel > 7)) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }

  fd_set fd_set_read;
  fd_set fd_set_write;
  struct timeval timeout;
  Int32_t err = 0;
  Uint32_t errlen = sizeof(err);

  // connect to notify channel
  // int socket(int domain, int type, int protocol);
  // domain:   该参数一般被设置为AF_INET，表示使用的是IPv4地址。
  //           还有更多选项可以利用man查看该函数
  // type:     该参数也有很多选项，例如SOCK_STREAM表示面向流的传输协议，
  //           SOCK_DGRAM表示数据报，我们这里实现的是TCP，因此选用SOCK_STREAM，
  //           如果实现UDP可选SOCK_DGRAM
  // protocol: 协议类型，一般使用默认，设置为0
  sockfd_notifying_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_notifying_ < 0) {
    LOG_ERR << "Can't create sock for notifying channel.";
    return (false);
  }

  struct sockaddr_in serveraddr;
  common::com_memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = inet_addr(param.can_net.ip_addr); //ip address
  // port for notifying channel
  serveraddr.sin_port = htons(param.can_net.notify_port);
  Int32_t socket_flag = 0;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  unsigned long ul=1;
  // 设置成非阻塞模式
  int retval = ioctlsocket(sockfd_notifying_, FIONBIO, (unsigned long *)&ul);
  if(retval == SOCKET_ERROR) {
      LOG_ERR << "ioctlsocket called failed";
      return false;
  }
#else
  socket_flag = fcntl(sockfd_notifying_, F_GETFL, 0);
  fcntl(sockfd_notifying_, F_SETFL, socket_flag | O_NONBLOCK);
#endif
  Int32_t connect_ret = connect(sockfd_notifying_,
                        (struct sockaddr*)&serveraddr,
                        sizeof(struct sockaddr));
  if (0 != connect_ret) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
     if( WSAGetLastError() == WSAEWOULDBLOCK) {
#else
    if (EINPROGRESS == errno) {
#endif
      LOG_INFO(3) << "Doing connection to notifying channel.";
      // 正在处理连接
      FD_ZERO(&fd_set_read);
      FD_ZERO(&fd_set_write);
      FD_SET(sockfd_notifying_, &fd_set_read);
      FD_SET(sockfd_notifying_, &fd_set_write);
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      Int32_t rc = select(sockfd_notifying_+ 1,
                  &fd_set_read, &fd_set_write, NULL, &timeout);
      // select调用失败
      if (rc < 0) {
        LOG_ERR << "Failed to connect to notifying channel, error="
                << strerror(errno);
        close(sockfd_notifying_);
        sockfd_notifying_ = -1;
        return false;
      }
      // 连接超时
      if (0 == rc) {
        LOG_ERR << "Failed to connect to notifying channel, timeout.";
        close(sockfd_notifying_);
        sockfd_notifying_ = -1;
        return false;
      }
      // 当连接建立遇到错误时，描述符变为即可读，也可写，rc=2
      // 遇到这种情况，可调用getsockopt函数
      if (2 == rc) {
        int ret = -1;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        ret = getsockopt(sockfd_notifying_, SOL_SOCKET, SO_ERROR,(char*) &err, (int*)&errlen);
#else
        ret = getsockopt(sockfd_notifying_, SOL_SOCKET, SO_ERROR, &err, &errlen);
#endif
        if (-1 == ret) {
          LOG_ERR << "Failed to getsockopt(SO_ERROR), error="
                  << strerror(errno);
          close(sockfd_notifying_);
          sockfd_notifying_ = -1;
          return false;
        }
        if (err) {
          LOG_ERR << "Failed to connect to notifying channel, error="
                  << strerror(err);
          close(sockfd_notifying_);
          sockfd_notifying_ = -1;
          return false;
        }
      }
      // 当连接成功建立时，描述符变成可写,rc=1
      if ((1 == rc) && FD_ISSET(sockfd_notifying_, &fd_set_write)) {
        connect_ret = rc;
        LOG_INFO(3) << "Succeeded to connecte to notifying channel="
                    << param.channel << "  "
                    << param.can_net.ip_addr << ":"
                    << param.can_net.port;
      }
    }
  }
  if (connect_ret < 0) {
    LOG_ERR << "Can't connect to server for notifying channel.";
    Int32_t ret = close(sockfd_notifying_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for notifying channel.";
    }
    sockfd_notifying_ = -1;
    return (false);
  }

  sockfd_working_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_working_ < 0) {
    LOG_ERR << "Can't create sock for working channel.";
    Int32_t ret = close(sockfd_notifying_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for notifying channel.";
    }
    sockfd_notifying_ = -1;
    return (false);
  }

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
  connect_ret = connect(sockfd_working_,
                (struct sockaddr*)&serveraddr,
                sizeof(struct sockaddr));
  if (0 != connect_ret) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    if (WSAGetLastError() == WSAEWOULDBLOCK) {
#else
    if (EINPROGRESS == errno) {
#endif
      LOG_INFO(3) << "Doing connection to working channel.";
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
        close(sockfd_notifying_);
        sockfd_notifying_ = -1;
        close(sockfd_working_);
        sockfd_working_ = -1;
        return false;
      }
      // 连接超时
      if (0 == rc) {
        LOG_ERR << "Failed to connect to working channel, timeout.";
        close(sockfd_notifying_);
        sockfd_notifying_ = -1;
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
          close(sockfd_notifying_);
          sockfd_notifying_ = -1;
          close(sockfd_working_);
          sockfd_working_ = -1;
          return false;
        }
        if (err) {
          LOG_ERR << "Failed to connect to working channel, error="
                  << strerror(err);
          close(sockfd_notifying_);
          sockfd_notifying_ = -1;
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
    close(sockfd_notifying_);
    sockfd_notifying_ = -1;
    close(sockfd_working_);
    sockfd_working_ = -1;
    return (false);
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  unsigned long ul3=1;
  ioctlsocket(sockfd_notifying_, FIONBIO, (unsigned long *)&ul3);
  ul3 = 1;
  ioctlsocket(sockfd_working_, FIONBIO, (unsigned long *)&ul3);
#else
  socket_flag = fcntl(sockfd_notifying_, F_GETFL, 0);
  fcntl(sockfd_notifying_, F_SETFL, socket_flag & (~O_NONBLOCK));
  socket_flag = fcntl(sockfd_working_, F_GETFL, 0);
  fcntl(sockfd_working_, F_SETFL, socket_flag & (~O_NONBLOCK));
#endif
  channel_is_open_ = true;

  return (true);
}

bool CanDriverZlgCanNet::CloseChannel() {
  if (channel_is_open_) {
    Int32_t ret = close(sockfd_working_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for working channel.";
    }
    sockfd_working_ = -1;
    ret = close(sockfd_notifying_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for notifying channel.";
    }
    sockfd_notifying_ = -1;

    channel_is_open_ = false;
  }

  return (true);
}

Int32_t CanDriverZlgCanNet::Send(const CanFrame* frame, Int32_t frame_num) {
  if (!channel_is_open_) {
    LOG_ERR << "This channel is close";
    return (-1);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    const CanFrame& fr = frame[i];
    Uint8_t data_buff[13] = { 0 };

    if (fr.EXT) {
      data_buff[0] |= 0x80;
    }
    if (fr.RTR) {
      data_buff[0] |= 0x40;
    }
    data_buff[0] |= (fr.data_len & 0x0F);

    data_buff[1] = fr.id >> 24;
    data_buff[2] = fr.id >> 16;
    data_buff[3] = fr.id >> 8;
    data_buff[4] = fr.id & 0xFF;

    common::com_memcpy(&data_buff[5], fr.data, 8);
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    Int32_t bytes =send(sockfd_working_, (const char*)data_buff, 13, 0);
#else
    Int32_t bytes = send(sockfd_working_, data_buff, 13, 0);
#endif
    if (bytes < 1) {
      LOG_ERR << "Failed to write can message.";
      continue;
    }

    send_count++;
  }

  return (send_count);
}

Int32_t CanDriverZlgCanNet::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  if (max_frame_num > 20) {
    max_frame_num = 20;
  }
  Uint8_t data_buff[13*20] = { 0 };

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
  Int32_t ret = select(common::Max(sockfd_notifying_, sockfd_working_)+1,
                       &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    LOG_ERR << "Failed to select fds.";
    return -1;
  }
  if (FD_ISSET(sockfd_notifying_, &read_fds)) {
    Uint8_t msg[8] = { 0 };
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    Int32_t bytes = recv(sockfd_notifying_, (char*)msg, sizeof(msg), 0);
#else
    Int32_t bytes = recv(sockfd_notifying_, msg, sizeof(msg), 0);
#endif
    if (bytes > 0) {
      Char_t message_buff[128] = { 0 };
      com_snprintf(message_buff, 127,
                   "CAN Net Error: %02X %02X %02X %02X %02X %02X %02X %02X ",
                   msg[0], msg[1], msg[2], msg[3],
                   msg[4], msg[5], msg[6], msg[7]);
      LOG_WARN << message_buff;
    }
  }
  if(!FD_ISSET(sockfd_working_, &read_fds)) {
    return -1;
  }
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  Int32_t bytes = recv(sockfd_working_, (char*)data_buff, max_frame_num*13, 0);
#else
  Int32_t bytes = recv(sockfd_working_, data_buff, max_frame_num*13, 0);
#endif
  if (bytes < 13) {
    // LOG_ERR << "Failed to receive can message.";
    return (-1);
  }
  Int32_t frame_num = bytes / 13;
  for (Int32_t i = 0; i < frame_num; ++i) {
    Uint8_t* buff = &data_buff[i * 13];
    CanFrame& fr = frame[i];

    fr.time_stamp = common::GetClockNowMs();

    if (buff[0] & 0x80) {
      fr.EXT = true;
    } else {
      fr.EXT = false;
    }
    if (buff[0] & 0x40) {
      fr.RTR = true;
    } else {
      fr.RTR = false;
    }
    fr.data_len = buff[0] & 0x0F;

    fr.id = (buff[1] << 24) | (buff[2] << 16) | (buff[3] << 8) | buff[4];

    common::com_memcpy(fr.data, &buff[5], 8);
  }

  return (frame_num);
}

#endif  // #if (ENABLE_CAN_DEV_ZLGCANNET)


}  // namespace can_dev
}  // namespace phoenix
