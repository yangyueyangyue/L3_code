/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       socket_communication.h
 * @brief      套接字通信
 * @details    实现了套接字通信(包含了分包/组包机制)的相关实现函数
 *
 * @author     boc
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * <tr><td>2021/09/06  <td>1.1      <td>pengc     <td>优化了程序架构
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_FRAMEWORK_SOCKET_COMMUNICATION_H_
#define PHOENIX_FRAMEWORK_SOCKET_COMMUNICATION_H_

#include "utils/macros.h"

/**
 * @brief Total memory allocated is roughly:
 *
 * MAX_BUFFER_NUM*(MAX_PACKET_SIZE + MAX_FRAGMENTS + CHANNEL_LENGTH) +
 *   PUBLISH_BUFFER_SIZE
 *
 */

/**
 * @macro SOCKET_COMM_MAX_CHANNEL_LENGTH
 * @brief 通道名称的最大字符数目（不含结束符号'\0', 不可超过255）
 */
#define SOCKET_COMM_MAX_CHANNEL_LENGTH (63)

/**
 * @macro SOCKET_COMM_MAX_BUFFER_NUM
 * @brief 接收端单个时刻的最大消息的个数
 */
#define SOCKET_COMM_MAX_BUFFER_NUM (4)

/**
 * @macro SOCKET_COMM_MAX_PACKET_SIZE
 * @brief 单个消息发送的最大字节数
 */
#define SOCKET_COMM_MAX_PACKET_SIZE (1*1024*1024)

/**
 * @macro SOCKET_COMM_MAX_FRAGMENTS
 * @brief 单个消息能够分配的最大包数目
 */
#define SOCKET_COMM_MAX_FRAGMENTS (800)

/**
 * @macro SOCKET_COMM_PUBLISH_BUFFER_SIZE
 * @brief 发送消息时单个包的最大字节数
 * @par Note:
 * @code
 *    Socket communication will allocate a single buffer of the size below for
 *    publishing messages. The SOCKET_COMM_3 fragmentation option will be used
 *    to send messages larger than this.
 *    1472[UDP DATA] = 1500 - 20[IP] - 8[UDP]
 * @endcode
 */
#define SOCKET_COMM_PUBLISH_BUFFER_SIZE (1472)

/**
 * @macro SOCKET_COMM_MAX_HEADER_LENGTH
 * @brief 消息中的包头的最大字节数
 * @par Note:
 * @code
 *    header contains a channel plus our usual header;
 *    24 ~ 为分包时的一些包信息;
 *    “SOCKET_COMM_MAX_CHANNEL_LENGTH + 1” ~ 为消息通道名称占的最大字节数.
 * @endcode
 */
#define SOCKET_COMM_MAX_HEADER_LENGTH (24 + SOCKET_COMM_MAX_CHANNEL_LENGTH + 1)

/**
 * @macro SOCKET_COMM_MAGIC_2
 * @brief 标记单包通讯
 */
#define SOCKET_COMM_MAGIC_2 0x4c433032

/**
 * @macro SOCKET_COMM_MAGIC_3
 * @brief 标记多包通讯
 */
#define SOCKET_COMM_MAGIC_3 0x4c433033

/**
 * @macro SOCKET_COMM_MAX_FRAGMENT_COUNT
 * @brief 包计数的最大值（不能超过这个值）
 */
#define SOCKET_COMM_MAX_FRAGMENT_COUNT (65536)


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct SocketCommFragmentBuffer
 * @brief 发送单个包时的缓存结构体
 */
typedef struct SocketCommFragmentBuffer SocketCommFragmentBuffer_t;
struct SocketCommFragmentBuffer {
  /// 当前包在对应消息中的包的数目
  Int32_t last_fragment_count;

  /// 服务器地址
  Uint64_t from_addr;
  /// 消息序号
  Uint32_t msg_seq;
  /// 通道名的字节数
  Uint32_t channel_len;
  /// 消息的字节数
  Uint32_t msg_len;

  /// CRC校验
  struct {
    /// 使能CRC校验
    Uint8_t enable;
    /// 通道名的校验码
    Uint8_t channel_crc;
    /// 消息的校验码
    Uint32_t msg_crc;
  } crc;

  /// 消息的通道名称
  Char_t channel[SOCKET_COMM_MAX_CHANNEL_LENGTH + 1];
  /// 该消息剩下的包的数目
  Int32_t fragments_remaining;
  /// 整个消息的数据缓存
  Char_t buf[SOCKET_COMM_MAX_PACKET_SIZE];
  /// 所有包是否已经接收的标记
  Uint8_t frag_received[SOCKET_COMM_MAX_FRAGMENTS];
};

/**
 * @struct SocketCommInstance
 * @brief 套接字通信结构体
 */
typedef struct SocketCommInstance SocketCommInstance_t;
struct SocketCommInstance {
  /// 数据发送对应的回调函数
  void (*func_transmit_packet)(
      const void* packet_buf, Int32_t packet_len, void *user);
  /// 发送回调函数对应的用户数据句柄
  void *transmit_user;

  /// 数据接收对应的回调函数
  void (*func_deliver_msg)(
      const Char_t* channel, const void* msg_buf, Int32_t msg_len, void *user);
  /// 接收回调函数对应的用户数据句柄
  void *deliver_user;

  /// 消息序号
  Uint32_t msg_seq;

  /** every time we receive a fragment, we increment this counter
        and write the value to the corresponding fragment buffer. This
        allows us to measure how "stale" a fragment is (how long it's
        been since we received a fragment for it).
    **/
  Int32_t last_fragment_count;

  /// 单个包中数据发送的缓存区
  Uint8_t publish_buffer[SOCKET_COMM_PUBLISH_BUFFER_SIZE];

  /** Buffers for reassembling multi-fragment messages. **/
  SocketCommFragmentBuffer_t fragment_buffers[SOCKET_COMM_MAX_BUFFER_NUM];
};

/**
 * @brief 对套接字通信进行初始化
 * @param[in] instance 套接字通信结构体的句柄
 * @param[in] transmit_packet 发送数据的回调函数
 * @param[in] transmit_user_data 发送时需要的一些信息
 * @return 0 - 成功；其他 - 失败。
 */
Int32_t Phoenix_SocketComm_Initialize(
    SocketCommInstance_t* instance,
    void (*transmit_packet)(const void* packet_buf, Int32_t packet_len,
                            void* user),
    void *transmit_user,
    void (*deliver_msg)(const Char_t* channel, const void* msg_buf,
                        Int32_t msg_len, void* user),
    void *deliver_user);

/**
 * @brief 发送消息。消息的字节数较大时会分为多个数据包进行发送。
 * @param[in] instance 套接字通信结构体的句柄
 * @param[in] channel 消息的通道名
 * @param[in] msg_buf 消息内容
 * @param[in] msg_len 消息的字节数
 * @detail Publish a message. Will call transmit_packet function one or more
 *         times synchronously. Returns 0 on success. This function should not
 *         be called concurrently with itself, but can be called concurrently
 *         with SocketCommReceivePacket.
 * @return 0 - 成功; 其他 - 失败
 */
Int32_t Phoenix_SocketComm_Publish(
    SocketCommInstance_t* instance,
    const Char_t* channel,
    const void* msg_buf,
    Int32_t msg_len,
    Uint8_t enable_crc);

/**
 * @brief 接收从套接字通信获取的消息，并对分包的数据进行合并。
 * @param[in] instance 套接字通信结构体的句柄
 * @param[in] packet_buf 接收到的单个数据包的数据
 * @param[in] packet_len 接收到的单个数据包的数据长度
 * @param[in] from_addr 数据来源的地址
 * @detail The user is responsible for creating and listening on a UDP
 *         multicast socket. When a packet is received, call this function. Do
 *         not call this function from more than one thread at a time. Returns
 *         zero if the packet was successfully handled, however no special
 *         action is required by the caller when an error occurs.
 * @return 0 - 成功；其他 - 失败。
 */
Int32_t Phoenix_SocketComm_RecvPacket(
    SocketCommInstance_t* instance,
    const void* packet_buf,
    Int32_t packet_len,
    Uint64_t from_addr);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_FRAMEWORK_SOCKET_COMMUNICATION_H_


