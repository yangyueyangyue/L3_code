//
#include "can_dev/linux_can/can_driver_linux_can.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include "utils/log.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_LINUX_CAN)

bool CanDriverLinuxCan::device_is_open_ = false;

CanDriverLinuxCan::CanDriverLinuxCan() {
  channel_is_open_ = false;
  sockfd_can_ = -1;
}

CanDriverLinuxCan::~CanDriverLinuxCan() {
  CloseChannel();
}

bool CanDriverLinuxCan::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open ZLG CAN NET device -------->";

  LOG_INFO(3) << "<-------- Open ZLG CAN NET device ---------";

  return (true);
}

bool CanDriverLinuxCan::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
  }

  return (true);
}

bool CanDriverLinuxCan::OpenChannel(const CanChannelParam& param) {
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

  struct sockaddr_can can_addr;
  struct ifreq ifr;
  // 创建套接字
  sockfd_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd_can_ < 0) {
    LOG_ERR << "Can't create can_fd for CAN channel(" << param.channel << ").";
    return (false);
  }

  switch (param.channel) {
  case (0):
    strcpy(ifr.ifr_name, "can0");
    break;
  case (1):
    strcpy(ifr.ifr_name, "can1");
    break;
  case (2):
    strcpy(ifr.ifr_name, "can2");
    break;
  case (3):
    strcpy(ifr.ifr_name, "can3");
    break;
  default:
    strcpy(ifr.ifr_name, "can0");
    break;
  }
  //指定 can 设备
  ioctl(sockfd_can_, SIOCGIFINDEX, &ifr);
  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = ifr.ifr_ifindex;
  //将套接字与 can0 绑定
  bind(sockfd_can_, (struct sockaddr *)&can_addr, sizeof(can_addr));
  // 设置过滤规则，取消当前注释为禁用过滤规则，即不接收所有报文，不设置此项（即如当前代码被注释）为接收所有ID的报文。
  // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  // fileter error frame
  can_err_mask_t err_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF);
  setsockopt(sockfd_can_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  // disable loopback
  Int32_t loopback = 0;
  setsockopt(sockfd_can_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  channel_is_open_ = true;

  return (true);
}

bool CanDriverLinuxCan::CloseChannel() {
  if (channel_is_open_) {
    Int32_t ret = close(sockfd_can_);
    if (ret < 0) {
      LOG_ERR << "Failed to close sock for can channel.";
    }
    sockfd_can_ = -1;

    channel_is_open_ = false;
  }

  return (true);
}

Int32_t CanDriverLinuxCan::Send(const CanFrame* frame, Int32_t frame_num) {
  if ((!channel_is_open_) || (sockfd_can_ < 0)) {
    LOG_ERR << "This channel is close";
    return (-1);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    const CanFrame& fr = frame[i];
    struct can_frame can_msg;

    /*
     * Controller Area Network Identifier structure
     *
     * bit 0-28	: CAN identifier (11/29 bit)
     * bit 29	: error message frame flag (0 = data frame, 1 = error message)
     * bit 30	: remote transmission request flag (1 = rtr frame)
     * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
     */
    can_msg.can_id = fr.id & 0x1FFFFFFF;
    if (fr.RTR) {
      can_msg.can_id |= 0x40000000;
    }
    if (fr.EXT) {
      can_msg.can_id |= 0x80000000;
    }

    can_msg.can_dlc = fr.data_len;

    common::com_memcpy(&can_msg.data[0], fr.data, 8);

    Int32_t bytes = write(sockfd_can_, &can_msg, sizeof(can_msg));
    if (sizeof(can_msg) != bytes) {
      LOG_ERR << "Failed to write can message, bytes=" << bytes;
      continue;
    }

    send_count++;
  }

  return (send_count);
}

Int32_t CanDriverLinuxCan::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!channel_is_open_) || (sockfd_can_ < 0) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is close or invalid parameter.";
    return (-1);
  }

  if (max_frame_num > 20) {
    max_frame_num = 20;
  }
  const static Int32_t s_frame_size = sizeof(struct can_frame);
  Uint8_t data_buff[s_frame_size*20] = { 0 };

  fd_set read_fds;  //读文件操作符
  FD_ZERO(&read_fds);
  // 每次调用select之前都要重新在read_fds中设置文件描述符，
  // 因为事件发生以后，文件描述符集合将被内核修改
  FD_SET(sockfd_can_, &read_fds);
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
  Int32_t ret = select(sockfd_can_+1, &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    LOG_ERR << "Failed to select fds.";
    return -1;
  }
  if(!FD_ISSET(sockfd_can_, &read_fds)) {
    return -1;
  }

  Int32_t bytes = read(sockfd_can_, data_buff, sizeof(data_buff));
  if (bytes < s_frame_size) {
    // LOG_ERR << "Failed to receive can message.";
    return (-1);
  }
  Int32_t frame_num = bytes / s_frame_size;
  for (Int32_t i = 0; i < frame_num; ++i) {
    Uint8_t* buff = &data_buff[i * s_frame_size];
    CanFrame& fr = frame[i];
    struct can_frame* can_msg = (struct can_frame*)buff;

    /*
     * Controller Area Network Identifier structure
     *
     * bit 0-28	: CAN identifier (11/29 bit)
     * bit 29	: error message frame flag (0 = data frame, 1 = error message)
     * bit 30	: remote transmission request flag (1 = rtr frame)
     * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
     */
    fr.id = can_msg->can_id & 0x1FFFFFFF;
    if ((can_msg->can_id >> 30) & 0x01) {
      fr.RTR = true;
    } else {
      fr.RTR = false;
    }
    if ((can_msg->can_id >> 31) & 0x01) {
      fr.EXT = true;
    } else {
      fr.EXT = false;
    }

    fr.data_len = can_msg->can_dlc;
    common::com_memcpy(fr.data, &can_msg->data[0], 8);
  }

  return (frame_num);
}

#endif  // #if (ENABLE_CAN_DEV_LINUX_CAN)


}  // namespace can_dev
}  // namespace phoenix
