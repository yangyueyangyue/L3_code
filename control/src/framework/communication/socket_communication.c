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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "communication/socket_communication.h"

#include <string.h>
#include <stdio.h>


/******************************************************************************/
/* 类型定义                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/

/*
 * @brief 计算CRC8校验码
 * @param[in] data_array 输入的需要校验的数据
 * @param[in] data_len 需要校验的数据的长度
 * @param[in] start_value_crc CRC初始值
 * @param[in] generator_polynom CRC参数
 * @param[in] final_ex_or CRC参数
 * @return 校验码
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Uint8_t CalculateCrc8(
    const Uint8_t data_array[], Int32_t data_len, Uint8_t start_value_crc,
    Uint8_t generator_polynom, Uint8_t final_ex_or) {
  Int32_t data_index = 0;
  Int32_t bit_index = 0;
  Uint8_t crc_u8 = 0;
  crc_u8 = start_value_crc;
  for (data_index = 0; data_index < data_len; ++data_index) {
    crc_u8 = data_array[data_index] ^ crc_u8;
    for (bit_index = 0; bit_index < 8; ++bit_index) {
      if((crc_u8 & 0x80) != 0 ) {
        crc_u8 = (crc_u8 << 1) ^ generator_polynom;
      } else {
        crc_u8 = (crc_u8 << 1);
      }
    }
  }
  crc_u8 = crc_u8 ^ final_ex_or;

  return (crc_u8);
}

/*
 * @brief 计算CRC32校验码
 * @param[in] i 输入的需要校验的数据
 * @return 校验码
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
#define CRC32_POLYNOMIAL 0xEDB88320L
// Calculate a CRC value to be used by CRC calculation functions.
static Uint32_t CalcCRC32Value(Int32_t i) {
  Int32_t j = 0;
  Uint32_t crc = i;
  for (j = 8 ; j > 0; j--) {
    if (crc & 1) {
      crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

/*
 * @brief Calculates the CRC-32 of a block of data all at once
 * @param[in] buffer 输入的需要校验的数据
 * @param[in] count 需要校验的数据的长度
 * @return 校验码
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Uint32_t CalculateBlockCRC32(
    const Uint8_t* buffer, /* Data block */
    Uint32_t count /* Number of bytes in the data block */) {
  Uint32_t temp1;
  Uint32_t temp2;
  Uint32_t crc = 0;
  while ((count--) != 0) {
    temp1 = ( crc >> 8 ) & 0x00FFFFFFL;
    temp2 = CalcCRC32Value( ((Int32_t) crc ^ *buffer++ ) & 0xff );
    crc = temp1 ^ temp2;
  }
  return (crc);
}

/*
 * @brief 将无符号32位整数编码为大端模式的字节流
 * @param[out] p 字节流
 * @param[in] v 无符号32位整数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
static void EncodeU32(Uint8_t *p, Uint32_t v) {
  // big endian. p[3] gets lowest 8 bits.
  p[3] = v & 0xff;
  v >>= 8;
  p[2] = v & 0xff;
  v >>= 8;
  p[1] = v & 0xff;
  v >>= 8;
  p[0] = v & 0xff;
}

/*
 * @brief 将大端模式的字节流解码为无符号32位整数
 * @param[in] p 字节流
 * @return 无符号32位整数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
static Uint32_t DecodeU32(const Uint8_t *p) {
  Uint32_t v = 0;

  // big endian. p[0] gets most significant bits.
  v |= p[0];
  v <<= 8;
  v |= p[1];
  v <<= 8;
  v |= p[2];
  v <<= 8;
  v |= p[3];

  return v;
}

/*
 * @brief 将无符号16位整数编码为大端模式的字节流
 * @param[out] p 字节流
 * @param[in] v 无符号16位整数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
static void EncodeU16(Uint8_t *p, Uint16_t v) {
  // big endian. p[1] gets lowest 8 bits.
  p[1] = v & 0xff;
  v >>= 8;
  p[0] = v & 0xff;
}

/*
 * @brief 将大端模式的字节流解码为无符号16位整数
 * @param[in] p     字节流
 * @return 无符号16位整数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/09/06  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
static Uint16_t DecodeU16(const Uint8_t *p) {
  Uint16_t v = 0;

  // big endian. p[0] gets most significant bits.
  v |= p[0];
  v <<= 8;
  v |= p[1];

  return v;
}


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 对套接字通信进行初始化
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t Phoenix_SocketComm_Initialize(
    SocketCommInstance_t* instance,
    void (*transmit_packet)(const void* packet_buf, Int32_t packet_len,
                            void* user),
    void *transmit_user,
    void (*deliver_msg)(const Char_t* channel, const void* msg_buf,
                        Int32_t msg_len, void *user),
    void *deliver_user) {
  // The caller allocates permanent storage for Socket communication.

  // Clear
  memset(instance, 0, sizeof(SocketCommInstance_t));

  // Set information for transmition
  instance->func_transmit_packet = transmit_packet;
  instance->transmit_user = transmit_user;

  // Set information for receiving
  instance->func_deliver_msg = deliver_msg;
  instance->deliver_user = deliver_user;

  return (0);
}

/*
 * @brief 发送消息。消息的字节数较大时会分为多个数据包进行发送。
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t Phoenix_SocketComm_Publish(
    SocketCommInstance_t* instance,
    const Char_t* channel,
    const void* msg_buf,
    Int32_t msg_len,
    Uint8_t enable_crc) {
  Uint32_t buf_pos = 0;
  Uint32_t start_pos = 0;
  Uint32_t msg_crc = 0;
  Uint32_t msg_seq = 0;
  Uint32_t fragment_offset = 0;
  Uint32_t max_fragment_size = 0;
  Uint32_t fragment_id = 0;
  Uint32_t fragments_in_msg = 0;
  Uint32_t this_fragment_size = 0;
  // channel length
  Uint32_t channel_len = strlen(channel);
  if (channel_len > SOCKET_COMM_MAX_CHANNEL_LENGTH) {
    channel_len = SOCKET_COMM_MAX_CHANNEL_LENGTH;
  }

  if (msg_len <
      (SOCKET_COMM_PUBLISH_BUFFER_SIZE - SOCKET_COMM_MAX_HEADER_LENGTH)) {
    // publish non-fragmented message
    buf_pos = 0;
    start_pos = buf_pos;

    /// Set Message Head
    // Packet type
    EncodeU32(&instance->publish_buffer[buf_pos], SOCKET_COMM_MAGIC_2);
    buf_pos += 4;
    // Message sequence
    EncodeU32(&instance->publish_buffer[buf_pos], instance->msg_seq);
    buf_pos += 4;

    // Message length
    EncodeU32(&instance->publish_buffer[buf_pos], msg_len);
    buf_pos += 4;
    // Length of channel name
    instance->publish_buffer[buf_pos] = channel_len;
    buf_pos += 1;
    // Enable CRC of Message
    instance->publish_buffer[buf_pos] = enable_crc;
    buf_pos += 1;

    // CRC for Packet Head
    instance->publish_buffer[buf_pos] =
        CalculateCrc8(
          &instance->publish_buffer[start_pos], buf_pos-start_pos,
          0x1B, 0x9B, 0xFF);
    buf_pos += 1;

    if (enable_crc) {
      // CRC for Channel Name
      instance->publish_buffer[buf_pos] =
          CalculateCrc8(
            (const Uint8_t*)&channel[0], channel_len, 0x1B, 0x9B, 0xFF);
      buf_pos += 1;
      // CRC for message
      msg_crc = CalculateBlockCRC32(msg_buf, msg_len);
      EncodeU32(&instance->publish_buffer[buf_pos], msg_crc);
      buf_pos += 4;
    }

    /// Increase Message Sequence
    instance->msg_seq++;

    // copy channel
    memcpy(&instance->publish_buffer[buf_pos], channel, channel_len);
    buf_pos += channel_len;

    // copy message
    memcpy(&instance->publish_buffer[buf_pos], msg_buf, msg_len);
    buf_pos += msg_len;

    instance->func_transmit_packet(instance->publish_buffer, buf_pos,
                                   instance->transmit_user);
  } else {
    // send fragmented message
    msg_seq = instance->msg_seq++;
    fragment_offset = 0;
    max_fragment_size =
        SOCKET_COMM_PUBLISH_BUFFER_SIZE - SOCKET_COMM_MAX_HEADER_LENGTH;
    fragment_id = 0;
    fragments_in_msg =
        (msg_len + max_fragment_size - 1) / max_fragment_size;

    while ((Int32_t)(fragment_offset) < msg_len) {
      buf_pos = 0;
      start_pos = buf_pos;

      EncodeU32(&instance->publish_buffer[buf_pos], SOCKET_COMM_MAGIC_3);
      buf_pos += 4;
      EncodeU32(&instance->publish_buffer[buf_pos], msg_seq);
      buf_pos += 4;
      EncodeU32(&instance->publish_buffer[buf_pos], fragment_offset);
      buf_pos += 4;
      EncodeU16(&instance->publish_buffer[buf_pos], fragment_id);
      buf_pos += 2;
      EncodeU16(&instance->publish_buffer[buf_pos], fragments_in_msg);
      buf_pos += 2;

      // first packet
      if (0 == fragment_id) {
        EncodeU32(&instance->publish_buffer[buf_pos], msg_len);
        buf_pos += 4;
        // Length of channel name
        instance->publish_buffer[buf_pos] = channel_len;
        buf_pos += 1;
        // Enable CRC of Message
        instance->publish_buffer[buf_pos] = enable_crc;
        buf_pos += 1;

        // CRC for Packet Head
        instance->publish_buffer[buf_pos] =
            CalculateCrc8(
              &instance->publish_buffer[start_pos], buf_pos-start_pos,
              0x1B, 0x9B, 0xFF);
        buf_pos += 1;

        if (enable_crc) {
          // CRC for Channel Name
          instance->publish_buffer[buf_pos] =
              CalculateCrc8(
                (const Uint8_t*)&channel[0], channel_len, 0x1B, 0x9B, 0xFF);
          buf_pos += 1;
          // CRC for message
          msg_crc = CalculateBlockCRC32(msg_buf, msg_len);
          EncodeU32(&instance->publish_buffer[buf_pos], msg_crc);
          buf_pos += 4;
        }

        // copy channel
        memcpy(&instance->publish_buffer[buf_pos], channel, channel_len);
        buf_pos += channel_len;
      } else {
        // CRC for Packet Head
        instance->publish_buffer[buf_pos] =
            CalculateCrc8(
              &instance->publish_buffer[start_pos], buf_pos-start_pos,
              0x1B, 0x9B, 0xFF);
        buf_pos += 1;
      }

      this_fragment_size = msg_len - fragment_offset;
      if (this_fragment_size > max_fragment_size) {
        this_fragment_size = max_fragment_size;
      }

      memcpy(&instance->publish_buffer[buf_pos],
             &((Char_t*) msg_buf)[fragment_offset], this_fragment_size);
      buf_pos += this_fragment_size;

      instance->func_transmit_packet(instance->publish_buffer,
                                     buf_pos, instance->transmit_user);

      fragment_offset += this_fragment_size;
      fragment_id++;
    }
  }

  return (0);
}

/*
 * @brief 接收从套接字通信获取的消息，并对分包的数据进行合并。
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t Phoenix_SocketComm_RecvPacket(
    SocketCommInstance_t* instance,
    const void* packet_buf,
    Int32_t packet_len,
    Uint64_t from_addr) {
  /** Call this function whenever an Socket UDP packet is
   * received. Registered Socket communication handlers will be called
   * synchronously. When the function returns, the buffer can be safely
   * reused. Returns non-zero if the packet was not decoded properly,
   * but no special action is required by the caller.
   *
   * from_addr is opaque, but should uniquely identify the sender's IP
   * address and port.
   **/

  Uint8_t *buf = (Uint8_t*)packet_buf;
  Uint32_t buf_pos = 0;
  Uint32_t start_pos = buf_pos;
  Uint32_t magic = 0;
  Uint32_t msg_seq = 0;
  Uint32_t msg_len = 0;
  Uint8_t channel_len = 0;
  Uint8_t enable_crc = 0;
  Uint8_t head_crc = 0;
  Uint8_t head_crc_check = 0;
  Uint8_t channel_crc = 0;
  Uint8_t channel_crc_check = 0;
  Uint32_t msg_crc = 0;
  Uint32_t msg_crc_check = 0;

  Uint32_t fragment_offset = 0;
  Uint32_t fragment_id = 0;
  Uint32_t fragments_in_msg = 0;
  SocketCommFragmentBuffer_t* fbuf = Nullptr_t;
  Int32_t payload_len = 0;
  Int32_t max_age = -1;
  Int32_t age = 0;

  Int32_t idx = 0;
  Char_t channel[SOCKET_COMM_MAX_CHANNEL_LENGTH + 1];

  // not even a header's length
  if (packet_len < 4) {
    return (-1);
  }

  magic = DecodeU32(&buf[buf_pos]);
  buf_pos += 4;

  if (SOCKET_COMM_MAGIC_2 == magic) {
    // Message sequence
    msg_seq = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    (void)msg_seq; // quiet unused variable warning.
    // Message length
    msg_len = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    // Channel name length
    channel_len = buf[buf_pos];
    buf_pos += 1;
    // Enable CRC ?
    enable_crc = buf[buf_pos];
    buf_pos += 1;
    // CRC of Head
    head_crc = buf[buf_pos];
    head_crc_check = CalculateCrc8(
          &buf[start_pos], buf_pos-start_pos, 0x1B, 0x9B, 0xFF);

    if (head_crc != head_crc_check) {
      // printf("[ERR][SOCKET_COMM] Failed to check CRC of heading (%d != %d).\n",
      //        head_crc, head_crc_check);
      return (-10);
    }
    buf_pos += 1;
    if (channel_len > SOCKET_COMM_MAX_CHANNEL_LENGTH) {
      // printf("[ERR][SOCKET_COMM] Invalid chennel length (%d).\n", channel_len);
      return (-11);
    }

    if (enable_crc) {
      // CRC of Channel name
      channel_crc = buf[buf_pos];
      buf_pos += 1;
      // CRC of message
      msg_crc = DecodeU32(&buf[buf_pos]);
      buf_pos += 4;
    }

    // copy out string holding the channel
    memcpy(&channel[0], &buf[buf_pos], channel_len);
    channel[channel_len] = 0;
    buf_pos += channel_len;
    if (msg_len != (packet_len - buf_pos)) {
      // printf("[ERR][SOCKET_COMM] Invalid message length (%d).\n", msg_len);
      return (-12);
    }

    if (enable_crc) {
      channel_crc_check = CalculateCrc8(
            (const Uint8_t*)&channel[0], channel_len, 0x1B, 0x9B, 0xFF);
      msg_crc_check =
          CalculateBlockCRC32(&buf[buf_pos], packet_len - buf_pos);
      if (channel_crc != channel_crc_check) {
        // printf("[ERR][SOCKET_COMM] Failed to check CRC of channel (%d != %d).\n",
        //        channel_crc, channel_crc_check);
        return (-13);
      }
      if (msg_crc != msg_crc_check) {
        // printf("[ERR][SOCKET_COMM] Failed to check CRC of message (%d != %d).\n",
        //        msg_crc, msg_crc_check);
        return (-14);
      }
    }

    // DeliverPacket(instance, channel, &buf[buf_pos], buf_len - buf_pos);
    instance->func_deliver_msg(
          channel, &buf[buf_pos], packet_len-buf_pos, instance->deliver_user);
  } else if (SOCKET_COMM_MAGIC_3 == magic) {
    msg_seq = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    fragment_offset = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    fragment_id = DecodeU16(&buf[buf_pos]);
    buf_pos += 2;
    fragments_in_msg = DecodeU16(&buf[buf_pos]);
    buf_pos += 2;

    if (fragments_in_msg > SOCKET_COMM_MAX_FRAGMENTS) {
      // printf("[ERR][SOCKET_COMM] Invalid fragments number (%d).\n",
      //        fragments_in_msg);
      return (-30);
    }
    if (fragment_id >= fragments_in_msg) {
      // printf("[ERR][SOCKET_COMM] Invalid fragment ID (%d).\n",
      //        fragment_id);
      return (-31);
    }

    // find the fragment. Use a simple linear search; this is
    // cheap in comparison to how much work we're spending to
    // decode the large packet...
    fbuf = Nullptr_t;

    // try to find a reassembly buffer for this from_addr that's
    // already in progress
    for (idx = 0; idx < SOCKET_COMM_MAX_BUFFER_NUM; idx++) {
      if (instance->fragment_buffers[idx].from_addr == from_addr &&
          instance->fragment_buffers[idx].msg_seq == msg_seq) {
        fbuf = &instance->fragment_buffers[idx];
        break;
      }
    }

    if (Nullptr_t == fbuf) {
      // didn't find one. Pick a new buffer to use.

      // Priorities:
      //   1) an idle (complete) buffer
      //   2) the incomplete buffer that received a valid fragment the
      //      longest time ago.
      max_age = -1; // low scores are good.
      for (idx = 0; idx < SOCKET_COMM_MAX_BUFFER_NUM; idx++) {
        if (0 == instance->fragment_buffers[idx].fragments_remaining) {
          fbuf = &instance->fragment_buffers[idx];
          break;
        } else {
          age = instance->last_fragment_count -
              instance->fragment_buffers[idx].last_fragment_count;
          if (age < 0) {
            age = SOCKET_COMM_MAX_FRAGMENT_COUNT -
                instance->fragment_buffers[idx].last_fragment_count +
                instance->last_fragment_count;
          }
          if (age > max_age) {
            fbuf = &instance->fragment_buffers[idx];
            max_age = age;
          }
        }
      }

      if (Nullptr_t == fbuf) {
        // printf("[ERR][SOCKET_COMM] Failed to find message buffer.\n");
        return (-32); // this should never happen
      }

      // initialize the fragment buffer
      for (idx = 0; idx < fragments_in_msg; idx++) {
        fbuf->frag_received[idx] = 0;
      }

      fbuf->from_addr = from_addr;
      fbuf->msg_seq = msg_seq;
      fbuf->fragments_remaining = fragments_in_msg;
    }

    // now, handle this fragment
    fbuf->last_fragment_count = instance->last_fragment_count;
    instance->last_fragment_count++;
    if (instance->last_fragment_count >= SOCKET_COMM_MAX_FRAGMENT_COUNT) {
      instance->last_fragment_count = 0;
    }

    if (0 == fragment_id) {
      msg_len = DecodeU32(&buf[buf_pos]);
      buf_pos += 4;
      // this fragment contains the channel name plus data
      channel_len = buf[buf_pos];
      buf_pos += 1;
      // Enable CRC ?
      enable_crc = buf[buf_pos];
      buf_pos += 1;
      // CRC of Head
      head_crc = buf[buf_pos];
      head_crc_check = CalculateCrc8(
            &buf[start_pos], buf_pos-start_pos, 0x1B, 0x9B, 0xFF);
      if (head_crc != head_crc_check) {
        // printf("[ERR][SOCKET_COMM] Failed to check CRC of head (%d != %d).\n",
        //        head_crc, head_crc_check);
        return (-33);
      }
      buf_pos += 1;
      if (channel_len > SOCKET_COMM_MAX_CHANNEL_LENGTH) {
        // printf("[ERR][SOCKET_COMM] Invalid channel length (%d).\n",
        //        channel_len);
        return (-34);
      }
      if (msg_len > SOCKET_COMM_MAX_PACKET_SIZE) {
        // printf("[ERR][SOCKET_COMM] Invalid message length (%d).\n",
        //        msg_len);
        return (-35);
      }

      payload_len = packet_len - buf_pos;
      if ((fragment_offset + payload_len) > (msg_len + channel_len)) {
        // printf("[ERR][SOCKET_COMM] Invalid fragment offset (%d).\n",
        //        fragment_offset);
        return (-36);
      }

      fbuf->channel_len = channel_len;
      fbuf->msg_len = msg_len;

      fbuf->crc.enable = enable_crc;
      if (enable_crc) {
        // CRC of Channel name
        fbuf->crc.channel_crc = buf[buf_pos];
        buf_pos += 1;
        // CRC of message
        fbuf->crc.msg_crc = DecodeU32(&buf[buf_pos]);
        buf_pos += 4;
      }
      // copy channel name
      memcpy(&fbuf->channel[0], &buf[buf_pos], channel_len);
      fbuf->channel[channel_len] = 0;
      buf_pos += channel_len;
    } else {
      // CRC of Head
      head_crc = buf[buf_pos];
      head_crc_check = CalculateCrc8(
            &buf[start_pos], buf_pos-start_pos, 0x1B, 0x9B, 0xFF);
      if (head_crc != head_crc_check) {
        // printf("[ERR][SOCKET_COMM] Failed to check CRC of head (%d != %d).\n",
        //        head_crc, head_crc_check);
        return (-37);
      }
      buf_pos += 1;

      payload_len = packet_len - buf_pos;
      if ((fragment_offset + payload_len) > fbuf->msg_len) {
        // printf("[ERR][SOCKET_COMM] Invalid fragment offset (%d) "
        //        "in fragment(%d), (offset(%d)+payload(%d)) > msg_len(%d)\n",
        //        fragment_offset, fragment_id,
        //        fragment_offset, payload_len, fbuf->msg_len);
        return (-38);
      }
    }

    if (buf_pos < packet_len) {
      memcpy(&fbuf->buf[fragment_offset], &buf[buf_pos], packet_len - buf_pos);
    }

    // record reception of this packet
    if (0 == fbuf->frag_received[fragment_id]) {
      fbuf->frag_received[fragment_id] = 1;
      fbuf->fragments_remaining--;

      if (0 == fbuf->fragments_remaining) {
        // DeliverPacket(instance, fbuf->channel, fbuf->buf, msg_len);
        if (fbuf->crc.enable) {
          channel_crc_check = CalculateCrc8(
                (const Uint8_t*)&fbuf->channel[0], fbuf->channel_len,
                0x1B, 0x9B, 0xFF);
          msg_crc_check =
              CalculateBlockCRC32(&fbuf->buf[0], fbuf->msg_len);
          if (fbuf->crc.channel_crc != channel_crc_check) {
            // printf("[ERR][SOCKET_COMM] Failed to check CRC of channel (%d != %d).\n",
            //        fbuf->crc.channel_crc, channel_crc_check);
            return (-39);
          }
          if (fbuf->crc.msg_crc != msg_crc_check) {
            // printf("[ERR][SOCKET_COMM] Failed to check CRC of message (%d != %d).\n",
            //        fbuf->crc.msg_crc, msg_crc_check);
            return (-40);
          }
        }
        instance->func_deliver_msg(
              fbuf->channel, fbuf->buf, fbuf->msg_len, instance->deliver_user);
      }
    }
  }

  return (0);
}

