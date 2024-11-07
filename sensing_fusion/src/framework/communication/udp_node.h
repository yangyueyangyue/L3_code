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

#ifndef PHOENIX_FRAMEWORK_UDP_NODE_H_
#define PHOENIX_FRAMEWORK_UDP_NODE_H_

#include <stdint.h>
#include <sys/types.h>

#include "utils/macros.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <WinSock2.h>
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif // #if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)

#include "utils/com_utils.h"
#include "os/mutex.h"
#include "os/thread.h"

#include "socket_communication.h"


namespace phoenix {
namespace framework {


/**
 * @struct UdpReceivingChannelInfo
 * @brief 接收通道信息
 */
struct UdpReceivingChannelInfo {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  SOCKET read_fd;
#else
  Int32_t read_fd;
#endif
};

/**
 * @struct UdpSendingChannelInfo
 * @brief 发送通道信息
 */
struct UdpSendingChannelInfo {
  /// 发送地址
  sockaddr_in send_addr;
  /// 发送描述符
  Int32_t send_fd;
};

/// 消息预订结构体
/**
 * @struct UdpSubscription
 * @brief 消息预订结构体。
 */
struct UdpSubscription {
  /// 消息的通道名
  Char_t channel[SOCKET_COMM_MAX_CHANNEL_LENGTH+1];
  /// 消息接收完毕后的回调函数
  void (*callback)(const Char_t *channel,
                   const void *buf, Int32_t buf_len, void *user);
  /// 用户自定义数据
  void *user;
};

/**
 * @class UdpNode
 * @brief 消息通信
 */
class UdpNode {
public:
  /**
   * @struct UdpParam
   * @brief 消息通信连接参数
   */
  struct UdpParam {
    enum {
      MODE_UNICAST = 0,
      MODE_GROUP,
      MODE_BROADCAST
    };
    /// 发送使能
    bool enable_send;
    /// 接收使能
    bool enable_recv;
    /// 发送时是否为广播模式
    Int32_t     mode;
    /// 接收端的端口
    Uint16_t    rcv_port;
    /// 发送端的端口
    Uint16_t    snd_port;
    /// 发送端的IP地址
    Char_t   		snd_addr[20];

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      enable_send = false;
      enable_recv = false;
      rcv_port = 0;
      snd_port = 0;
      mode = MODE_UNICAST;
      common::com_memset(snd_addr, 0, sizeof(snd_addr));
    }

    /**
     * @brief 构造函数
     */
    UdpParam() {
      Clear();
    }
  };

public:
  /**
   * @brief 构造函数
   */
  UdpNode();
  /**
   * @brief 析构函数
   */
  ~UdpNode();

  /**
   * @brief 创建UDP连接
   * @param[in] comm_param      消息通信连接参数
   * @return true - 成功；false - 失败。
   */
  bool Start(const UdpParam& comm_param);

  /**
   * @brief 关闭UDP连接
   */
  bool Stop();

  /**
   * @brief 订阅消息
   * @param[in] channel  通道名称
   * @param[in] handlerMethod 回调函数(收到消息后，调用此函数通知用户)
   * @param[in] user 回调函数中的用户数据
   * @return true ~ 成功, false ~ 失败
   */
  bool Subscribe(const Char_t* channel,
                 void (*handlerMethod)(const Char_t *channel,
                                       const void *buf, Int32_t buf_len,
                                       void *user),
                 void* user);

  /**
   * @brief 开始接收消息
   * @return true ~ 成功, false ~ 失败
   */
  bool StartReceiving();

  /**
   * @brief 发布消息
   * @param[in] channel      通道名
   * @param[in] buf          消息内容
   * @param[in] buf_len      消息内容的字节数
   * @return true ~ 成功, false ~ 失败
   * @note 通道名的字符个数（不含结尾符'\0'）不能超过4，超过的部分将会被舍弃。
   */
  bool Publish(
      const Char_t *channel, const void *buf, Int32_t buf_len,
      bool enable_crc = false);

  /**
   * @brief 消息接收完毕后，将完整的数据通过回调函数的形式发送给用户。
   * @param[in] channel    消息的通道名
   * @param[in] msg        消息的完整数据
   * @param[in] msg_len    消息的完整数据的字节大小
   * @detail Called by socket communication internally when a packet is decoded.
   *         (Provides a common delivery code path for fragmented and
   *         non-fragmented packets.)
   */
  void DeliverMessageToUser(
      const Char_t *channel, const void *msg, Int32_t msg_len) const;

  /* For test only (begin) */
  const Char_t* GetReceivingMsgHead() const { return receiving_msg_head_buf_; }
  /* For test only (end) */

private:
  /*
   * @brief 线程函数(接收并解析UDP报文)
   */
  void TheadReceivingMessages();

private:
  // 通信连接参数
  UdpParam param_;

  // 是否已经开始接收消息
  bool receiving_flag_;
  // 接收相关的信息
  UdpReceivingChannelInfo receiving_channel_info_;
  // 发送相关的信息
  UdpSendingChannelInfo sending_channel_info_;
  // socket通讯的句柄
  SocketCommInstance_t socket_communication_;
  // 保存用户订阅信息
  enum { MAX_SUBSCRIPTION_NUM = 20 };
  Int32_t subscription_count_;
  UdpSubscription subscription_list_[MAX_SUBSCRIPTION_NUM];
  // 发送消息用的锁
  common::os::Mutex lock_sending_channel_;

  // 消息接收线程
  common::os::Thread receiving_thread_;
  common::os::ThreadFuncHelper<UdpNode> receiving_thread_func_helper_;
  Char_t receiving_msg_buf_[65536];

  /* For test only (begin) */
  Char_t receiving_msg_head_buf_[1024] = { 0 };
  /* For test only (end) */
};


}  // framework
}  // phoenix

#endif  // PHOENIX_FRAMEWORK_UDP_NODE_H_


