/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       udp_node.h
 * @brief      UDP通信
 * @details    实现了UDP通信的相关实现函数
 *
 * @author     pengc
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "communication/udp_node.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/time.h>
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <ws2tcpip.h>
#endif

#include "utils/com_utils.h"
#include "utils/log.h"


namespace phoenix {
namespace framework {


/*
 * @brief 回调函数(发送数据包)
 * @param[in] buf_in 发送的数据
 * @param[in] buf_len 发送数据的字节长度
 * @param[in] user 发送通道信息
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void TransmitPacket(const void *buf_in, Int32_t buf_len, void *user) {
  UdpSendingChannelInfo *tinfo = (UdpSendingChannelInfo*)user;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  int res = sendto(tinfo->send_fd, (char *)buf_in, buf_len, 0,
                   (SOCKADDR*)&tinfo->send_addr, sizeof(tinfo->send_addr));
  if (res < 0) {
    LOG_ERR << "Failed to send message to network.";
  }
#else
  ssize_t res = sendto(tinfo->send_fd, buf_in, buf_len, 0,
                       (struct sockaddr*) &tinfo->send_addr,
                       sizeof(tinfo->send_addr));
  if (res < 0) {
    LOG_ERR << "Failed to send message to network.";
  }
#endif

  //std::cout << "Transmit " << res << " bytes" << std::endl;
}

/*
 * @brief 回调函数(分发接收到的消息)
 * @param[in] channel 通道名称
 * @param[in] msg_buf 接收到的消息
 * @param[in] msg_len 接收到数据的字节长度
 * @param[in] user UdpNode的指针
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void DeliverMessage(
    const Char_t* channel, const void* msg_buf, Int32_t msg_len, void* user) {
  // std::cout << "Received " << msg_len << " bytes from channel(" << channel
  //           << ")." << std::endl;

  UdpNode* udp_node = (UdpNode*)user;
  udp_node->DeliverMessageToUser(channel, msg_buf, msg_len);
}


/*
 * @brief 构造函数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
UdpNode::UdpNode() {
  receiving_flag_ = false;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  receiving_channel_info_.read_fd = INVALID_SOCKET;
  sending_channel_info_.send_fd = INVALID_SOCKET;
  common::com_memset(&sending_channel_info_.send_addr, 0,
                     sizeof(sending_channel_info_.send_addr));
#else
  receiving_channel_info_.read_fd = -1;
  sending_channel_info_.send_fd = -1;
  common::com_memset(&sending_channel_info_.send_addr, 0,
                     sizeof(sending_channel_info_.send_addr));
#endif

  subscription_count_ = 0;
  for (Int32_t i = 0; i < MAX_SUBSCRIPTION_NUM; ++i) {
    common::com_memset(subscription_list_[i].channel, 0,
                       sizeof(subscription_list_[i].channel));
    subscription_list_[i].callback = Nullptr_t;
    subscription_list_[i].user = Nullptr_t;
  }
}

/**
 * @brief 析构函数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
UdpNode::~UdpNode() {
  Stop();
}

/**
 * @brief 创建UDP连接
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool UdpNode::Start(const UdpParam& comm_param) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((INVALID_SOCKET != sending_channel_info_.send_fd) ||
      (INVALID_SOCKET != receiving_channel_info_.read_fd)) {
    LOG_ERR << "Already start.";
    return false;
  }
#else
  if ((sending_channel_info_.send_fd >= 0) ||
      (receiving_channel_info_.read_fd >= 0)) {
    LOG_ERR << "Already start.";
    return false;
  }
#endif

  // 0~1024一般给系统使用，一共可以分配到65535
  if (param_.enable_recv) {
    if ((comm_param.rcv_port < 1025) || (comm_param.rcv_port > 65535)) {
      LOG_ERR << "Invalid receiving port, must be in [1025, 65535].";
      return false;
    }
  }
  if (param_.enable_send) {
    if ((comm_param.snd_port < 1025) || (comm_param.snd_port > 65535)) {
      LOG_ERR << "Invalid sending port, must be in [1025, 65535].";
      return false;
    }
  }

  param_.enable_recv = comm_param.enable_recv;
  param_.enable_send = comm_param.enable_send;
  param_.snd_port = comm_param.snd_port;
  param_.rcv_port = comm_param.rcv_port;
  param_.mode = comm_param.mode;
  common::com_memcpy(param_.snd_addr, comm_param.snd_addr,
                     sizeof(param_.snd_addr));

  if (param_.enable_recv) {
    // 1.创建udp通信socket
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    receiving_channel_info_.read_fd =
        socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  // 指定为UDP套接字
    if (INVALID_SOCKET == receiving_channel_info_.read_fd) {
      err = WSAGetLastError();
      LOG_ERR << "Failed to create socket fd for receiving channel, err=" << err;
      return false;
    }
#else
    receiving_channel_info_.read_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiving_channel_info_.read_fd < 0) {
      LOG_ERR << "Failed to create socket fd for receiving channel.";
      return false;
    }
#endif

    if (UdpParam::MODE_GROUP == param_.mode) {
      // 加入组播, 组播地址224.0.0.0~239.255.255.255
      struct ip_mreq group;
      common::com_memset(&group, 0, sizeof(group));
      group.imr_multiaddr.s_addr = inet_addr(param_.snd_addr);  //设置组播地址
      group.imr_interface.s_addr = inet_addr("0.0.0.0");
      if (setsockopt(receiving_channel_info_.read_fd,
                     IPPROTO_IP, IP_ADD_MEMBERSHIP,
                     &group, sizeof(group)) < 0) {
        close(receiving_channel_info_.read_fd);
        receiving_channel_info_.read_fd = -1;
        LOG_ERR << "Failed to add to group.";
        return false;
      }
    }

    // 2.设置UDP的地址并绑定
    struct sockaddr_in read_addr;
    common::com_memset(&read_addr, 0, sizeof(read_addr));
    // 使用IPv4协议
    read_addr.sin_family = AF_INET;
    // 网络通信都使用大端格式
    read_addr.sin_port = htons(param_.rcv_port);
    if (UdpParam::MODE_GROUP == param_.mode) {
      // 注意：Linux下，加入组播后，绑定地址只能绑定0.0.0.0地址否则会接收不到数据
      read_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    } else {
      // 让系统检测本地网卡，自动绑定本地IP
      read_addr.sin_addr.s_addr = INADDR_ANY;
    }

    // 允许重用本地地址
    int opt = 1;
    if (setsockopt(receiving_channel_info_.read_fd,
                   SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
      close(receiving_channel_info_.read_fd);
      receiving_channel_info_.read_fd = -1;
      LOG_ERR << "Failed to setsockopt(SOL_SOCKET, SO_REUSEADDR).";
      return false;
    }

//    Uint8_t mc_ttl = 0;
//    if (setsockopt(receiving_channel_info_.read_fd,
//                   IPPROTO_IP, IP_MULTICAST_TTL, &mc_ttl, sizeof(mc_ttl)) < 0) {
//      close(receiving_channel_info_.read_fd);
//      receiving_channel_info_.read_fd = -1;
//      LOG_ERR << "Failed to setsockopt(IPPROTO_IP, IP_MULTICAST_TTL).";
//      return false;
//    }

    if (bind(receiving_channel_info_.read_fd,
             (struct sockaddr*)&read_addr, sizeof(read_addr)) < 0) {
      LOG_ERR << "Failed to bind socket.";
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
      closesocket(receiving_channel_info_.read_fd);
      receiving_channel_info_.read_fd = INVALID_SOCKET;
#else
      close(receiving_channel_info_.read_fd);
      receiving_channel_info_.read_fd = -1;
#endif
      return false;
    }
  }

  if (param_.enable_send) {
    // 1 创建udp通信socket
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    // 指定为UDP套接字
    sending_channel_info_.send_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (INVALID_SOCKET == sending_channel_info_.send_fd) {
      if (INVALID_SOCKET != receiving_channel_info_.read_fd) {
        closesocket(receiving_channel_info_.read_fd);
        receiving_channel_info_.read_fd = INVALID_SOCKET;
      }
      err = WSAGetLastError();
      printf("error! error code is %d/n", err);
      LOG_ERR << "Failed to create socket fd for sending channel, err=" << err;
      return false;
    }
#else
    sending_channel_info_.send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sending_channel_info_.send_fd < 0) {
      if (receiving_channel_info_.read_fd >= 0) {
        close(receiving_channel_info_.read_fd);
        receiving_channel_info_.read_fd = -1;
      }
      LOG_ERR << "Failed to create socket fd for sending channel.";
      return false;
    }
//    Uint8_t mc_ttl = 0;
//    if (setsockopt(sending_channel_info_.send_fd,
//                   IPPROTO_IP, IP_MULTICAST_TTL, &mc_ttl, sizeof(mc_ttl)) < 0) {
//      close(sending_channel_info_.send_fd);
//      sending_channel_info_.send_fd = -1;
//      if (receiving_channel_info_.read_fd >= 0) {
//        close(receiving_channel_info_.read_fd);
//        receiving_channel_info_.read_fd = -1;
//      }
//      LOG_ERR << "Failed to setsockopt(IPPROTO_IP, IP_MULTICAST_TTL).";
//      return false;
//    }
#endif

    if (UdpParam::MODE_BROADCAST == param_.mode) {
      // 开启发送广播数据功能
      int opt = 1;
      //设置该套接字为广播类型，
      if (setsockopt(sending_channel_info_.send_fd,
                     SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) < 0) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        closesocket(sending_channel_info_.send_fd);
        sending_channel_info_.send_fd = INVALID_SOCKET;
        if (INVALID_SOCKET != receiving_channel_info_.read_fd) {
          closesocket(receiving_channel_info_.read_fd);
          receiving_channel_info_.read_fd = INVALID_SOCKET;
        }
#else
        close(sending_channel_info_.send_fd);
        sending_channel_info_.send_fd = -1;
        if (receiving_channel_info_.read_fd >= 0) {
          close(receiving_channel_info_.read_fd);
          receiving_channel_info_.read_fd = -1;
        }
#endif
        LOG_ERR << "Failed to setsockopt(SOL_SOCKET, SO_BROADCAST).";
        return false;
      }
    }

    // 设置目的IP地址
    struct sockaddr_in send_addr;
    common::com_memset(&send_addr, 0, sizeof(send_addr));
    // 使用IPv4协议
    send_addr.sin_family = AF_INET;
    // 设置接收方端口号
    send_addr.sin_port = htons(param_.snd_port);
    // 设置接收方IP
    if (UdpParam::MODE_BROADCAST == param_.mode) {
      send_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    } else {
      send_addr.sin_addr.s_addr = inet_addr(param_.snd_addr);
    }
    sending_channel_info_.send_addr = send_addr;
  }

  Phoenix_SocketComm_Initialize(
        &socket_communication_,
        &TransmitPacket, &sending_channel_info_,
        &DeliverMessage, this);

  return true;
}

/**
 * @brief 关闭UDP连接
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool UdpNode::Stop() {
  LOG_INFO(3) << "Stop UDP ... ";

  if (receiving_flag_) {
    LOG_INFO(3) << "Waiting to stop UDP receiving thread  ... ";
    receiving_flag_ = false;
    receiving_thread_.Join();
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if (INVALID_SOCKET != sending_channel_info_.send_fd) {
    shutdown(sending_channel_info_.send_fd, SD_BOTH);
    closesocket(sending_channel_info_.send_fd);
    sending_channel_info_.send_fd = INVALID_SOCKET;
  }
  if (INVALID_SOCKET != receiving_channel_info_.read_fd) {
    shutdown(receiving_channel_info_.read_fd, SD_BOTH);
    closesocket(receiving_channel_info_.read_fd);
    receiving_channel_info_.read_fd = INVALID_SOCKET;
  }
#else
  if (sending_channel_info_.send_fd >= 0) {
    shutdown(sending_channel_info_.send_fd, SHUT_RDWR);
    close(sending_channel_info_.send_fd);
    sending_channel_info_.send_fd = -1;
  }
  if (receiving_channel_info_.read_fd >= 0) {
    shutdown(receiving_channel_info_.read_fd, SHUT_RDWR);
    close(receiving_channel_info_.read_fd);
    receiving_channel_info_.read_fd = -1;
  }
#endif

  if (subscription_count_ > 0) {
    subscription_count_ = 0;
    for (Int32_t i = 0; i < MAX_SUBSCRIPTION_NUM; ++i) {
      common::com_memset(subscription_list_[i].channel, 0,
                         sizeof(subscription_list_[i].channel));
      subscription_list_[i].callback = Nullptr_t;
      subscription_list_[i].user = Nullptr_t;
    }
  }

  LOG_INFO(3) << "Stop UDP ... [OK]";

  return true;
}

/**
 * @brief 订阅消息
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool UdpNode::Subscribe(
    const Char_t* channel,
    void (*handlerMethod)(const Char_t *channel,
                          const void *buf, Int32_t buf_len,
                          void *user),
    void* user) {
  if (subscription_count_ >= MAX_SUBSCRIPTION_NUM) {
    LOG_ERR << "Can't add subscriber any more.";
    return false;
  }

  Int32_t channel_name_len = strlen(channel);
  if (channel_name_len > SOCKET_COMM_MAX_CHANNEL_LENGTH) {
    LOG_ERR << "The length of channel name is too long.";
    return false;
  }

  UdpSubscription& sub = subscription_list_[subscription_count_];

  common::com_memcpy(sub.channel, channel, channel_name_len);
  sub.channel[channel_name_len] = 0;
  sub.callback = handlerMethod;
  sub.user = user;

  subscription_count_++;

  return true;
}

/**
 * @brief 开始接收消息
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UdpNode::StartReceiving() {
  if (receiving_flag_) {
    LOG_WARN << "Already start.";
    return true;
  }
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((!param_.enable_recv) ||
      (INVALID_SOCKET == receiving_channel_info_.read_fd)) {
    LOG_ERR << "Invalid receiving channel.";
    return false;
  }
#else
  if ((!param_.enable_recv) || (receiving_channel_info_.read_fd < 0)) {
    LOG_ERR << "Invalid receiving channel.";
    return false;
  }
#endif

  receiving_flag_ = true;
  receiving_thread_func_helper_.SetThreadFunc(
        this, &UdpNode::TheadReceivingMessages);
  common::os::Thread::Parameter thread_param;
  bool ret = receiving_thread_.Create(thread_param,
                                      &receiving_thread_func_helper_);
  if (!ret) {
    receiving_flag_ = false;
    LOG_ERR << "Failed to create UDP receiving thread.";
    return false;
  }

  return true;
}

/**
 * @brief 发布消息
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UdpNode::Publish(
    const Char_t *channel, const void *buf, Int32_t buf_len, bool enable_crc) {
  // Lock sending channel
  common::os::LockHelper lock(lock_sending_channel_);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if (param_.enable_send && (INVALID_SOCKET != sending_channel_info_.send_fd)) {
    return (0 == Phoenix_SocketComm_Publish(
              &socket_communication_, channel, buf, buf_len, enable_crc));
  }
#else
  if (param_.enable_send && (sending_channel_info_.send_fd >= 0)) {
    // std::cout << "Send " << buf_len << " bytes to channel("
    //           << channel << ")." << std::endl;
    return (0 == Phoenix_SocketComm_Publish(
              &socket_communication_, channel, buf, buf_len, enable_crc));
  }
#endif

  return false;
}

/**
 * @brief 消息接收完毕后，将完整的数据通过回调函数的形式发送给用户。
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void UdpNode::DeliverMessageToUser(
    const Char_t *channel, const void *msg, Int32_t msg_len) const {
  for (Int32_t i = 0; i < MAX_SUBSCRIPTION_NUM; ++i) {
    Int32_t good = 1;
    const UdpSubscription& sub = subscription_list_[i];

    for (Int32_t pos = 0; pos <= SOCKET_COMM_MAX_CHANNEL_LENGTH; pos++) {
      if (sub.channel[pos] == channel[pos]) {
        // end of string? if so, we're done.
        if (0 == channel[pos]) {
          break;
        }

        // proceed to the next letter
        // pos++;
        continue;
      }

      // not a match.
      good = 0;
      // printf("Ln91. Not good. pos=%d\n", pos);
      break;
    }

    if (good) {
      sub.callback(channel, msg, msg_len, sub.user);
      break;
    } else {
      // continue
    }
  }
}

/**
 * @brief 线程函数(接收并解析UDP报文)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void UdpNode::TheadReceivingMessages() {
  LOG_INFO(3) << "UDP Receiving Thread ... [Started]";

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((!param_.enable_recv) ||
      (INVALID_SOCKET == receiving_channel_info_.read_fd)) {
    LOG_ERR << "Invalid receiving channel.";
    return;
  }
#else
  if ((!param_.enable_recv) || (receiving_channel_info_.read_fd < 0)) {
    LOG_ERR << "Invalid receiving channel.";
    return;
  }
#endif

  struct sockaddr_in from_addr; // only IPv4 compatible
  socklen_t from_addr_sz = sizeof(from_addr);
  while (receiving_flag_) {
    fd_set read_fds;  //读文件操作符
    FD_ZERO(&read_fds);
    // 每次调用select之前都要重新在read_fds中设置文件描述符，
    // 因为事件发生以后，文件描述符集合将被内核修改
    FD_SET(receiving_channel_info_.read_fd, &read_fds);
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
    timeout.tv_sec = 0;
    timeout.tv_usec = 50 * 1000;
    Int32_t ret = select(receiving_channel_info_.read_fd+1,
                         &read_fds, Nullptr_t, Nullptr_t, &timeout);
    if (ret >= 0) {
      if (FD_ISSET(receiving_channel_info_.read_fd, &read_fds)) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        Int32_t msg_len =
            recvfrom(receiving_channel_info_.read_fd,
                     receiving_msg_buf_, sizeof(receiving_msg_buf_), 0,
                     (struct sockaddr*)&from_addr, &from_addr_sz);
#else
        ssize_t msg_len =
            recvfrom(receiving_channel_info_.read_fd,
                     receiving_msg_buf_, sizeof(receiving_msg_buf_), 0,
                     (struct sockaddr*)&from_addr, &from_addr_sz);
#endif

        //std::cout << "Receiving " << msg_len << " bytes, from \""
        //          << inet_ntoa(from_addr.sin_addr) << "\"" << std::endl;

        /* For test only (begin) */
        struct timeval tv;
        gettimeofday(&tv, NULL);
        Float64_t now = (static_cast<Int64_t>(tv.tv_sec) * 1000000 + tv.tv_usec)*0.000001;
        time_t timestamp = static_cast<time_t>(now);
        struct ::tm tm_time;
        localtime_r(&timestamp, &tm_time);
        Int32_t usecs = static_cast<Int32_t>((now - timestamp) * 1000000);

        com_snprintf(receiving_msg_head_buf_,
                     sizeof(receiving_msg_head_buf_)-1,
                     "%s  %02d-%02d %02d:%02d:%02d.%06d :\n",
                     inet_ntoa(from_addr.sin_addr),
                     1+tm_time.tm_mon,
                     tm_time.tm_mday,
                     tm_time.tm_hour,
                     tm_time.tm_min,
                     tm_time.tm_sec,
                     usecs);
        //std::cout << "Head : " << receiving_msg_head_buf_ << std::endl;
        /* For test only (end) */

        ret = Phoenix_SocketComm_RecvPacket(
              &socket_communication_, receiving_msg_buf_, msg_len,
              from_addr.sin_addr.s_addr | ((Uint64_t)from_addr.sin_port << 32));
        if (ret < 0) {
          LOG_WARN << "Failed to receive packet. ERR=" << ret;
        }
      }
    }
  }

  LOG_INFO(3) << "UDP Receiving Thread ... [Stopped]";
}


}  // framework
}  // phoenix

