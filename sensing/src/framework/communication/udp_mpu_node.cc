/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       socket_communication.cc
 * @brief      套接字通信
 * @details    实现了套接字通信的相关实现函数
 *
 * @author     boc
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "utils/macros.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "communication/udp_mpu_node.h"

#include <functional>
#include <bitset>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/time.h>

#include <map>
#include <iostream>
#include <bitset>

#ifdef ROS
#include <ros/time.h>
#endif

#include "utils/com_utils.h"
#include "utils/log.h"


// 定位显示数据模块
static Uint32_t  mod_position_map_data_size=0;
static Uint8_t  *mod_position_map_data = 0;

namespace phoenix {
namespace framework {
namespace mpu{

struct St_DeliverPacket {
    socket_comm_t *handle;
    std::string channel_name;
    Uint8_t* cache;
    Uint32_t cache_size;
    Int64_t timestamp;
};

class UDPCommData
{
public:
    virtual ~UDPCommData() {}

    UdpParam param_; // 通信连接参数

    bool receiving_flag_; // 是否已经开始接收消息
    void* communication_handle_;

    SocketReceivingChannelInfo receiving_channel_info_;// 接收相关的信息
    SocketSendingChannelInfo sending_channel_info_; // 发送相关的信息

    enum { MAX_SUBSCRIPTION_NUM = 40 };
    Int32_t subscription_count_;
    SocketSubscription subscription_list_[MAX_SUBSCRIPTION_NUM];
};

class UDPCommPrivate:public UDPCommData
{
};

class UdpNodePrivate: public UDPCommPrivate
{
};


Uint8_t g_MpuBuf[1004]={0};
Uint64_t errSum = 0;

/// 套接字通信收发时需要的结构体的实体。
/// 由于需要的内存较多，放到静态全局变量中。
static const Int32_t s_max_socket_comm_handle_num = 10;
static Int32_t s_socket_common_handle_allocate_index = 0;
static socket_comm_t s_socket_comm_handle[s_max_socket_comm_handle_num];

static Int32_t RECEIVING_UDP_MSG_BUF_SIZE = 65536;


std::string get_dec2hex_str(Int32_t dec_num_)
{
  std::string res = "";
  if(dec_num_ == 0) {
    res = "0";
    return res;
  }

  Int32_t dec_num = dec_num_;
  if(dec_num_ < 0) {
    dec_num = -dec_num_;
  }

  while(dec_num) {
    Int32_t mod = dec_num % 16;
    if(mod > 9) {
      switch (mod) {
        case 10:
          res.insert(0, "A");
          break;
        case 11:
          res.insert(0, "B");
          break;
        case 12:
          res.insert(0, "C");
          break;
        case 13:
          res.insert(0, "D");
          break;
        case 14:
          res.insert(0, "E");
          break;
        case 15:
          res.insert(0, "F");
      }
    }
    else {
      res.insert(0, std::to_string(mod));
    }
    dec_num = dec_num / 16;
  }

  return res;
}


std::string get_dec2oct_str(Int32_t dec_num_)
{
  std::string res = "";
  if(dec_num_ == 0) {
    res = "0";
    return res;
  }

  Int32_t dec_num = dec_num_;
  if(dec_num_ < 0) {
    dec_num = -dec_num_;
  }

  while(dec_num) {
    Int32_t mod = dec_num % 8;
    res.insert(0, std::to_string(mod));
    dec_num = dec_num / 8;
  }

  return res;
}

std::string get_dec2bin_str(Int32_t dec_num_)
{
  std::string res="";
  if(dec_num_ == 0) {
    res = "0";
    return res;
  }

  Int32_t dec_num = dec_num_;
  if(dec_num_ < 0) {
    dec_num = dec_num_;
  }

  while(dec_num) {
    Int32_t mod = dec_num % 2;
    res.insert(0, std::to_string(mod));
    dec_num = dec_num / 2;
  }

  return res;
}

static Int32_t check_crc_8(Uint8_t *buf, Uint32_t len)
{
  Uint8_t sum = 0;
  for(int i = 0; i < len; i++) {
    sum += buf[i];
  }

  if(sum == 0)
    return 1;

  return 0;
}


static uint8_t KTCloudGetChecKS(uint8_t* data, uint16_t len)
{
    uint8_t checksum = 0;
    if(data && len > 1)
    {
        uint32_t tSum = 0;
        uint16_t index = 0;
        while ((index < len) && (len - index > 4))
        {
            tSum ^= *(uint32_t *)(data + index);
            index += sizeof(uint32_t);
        }
        for (uint16_t i = 0; i < sizeof(uint32_t); i++)
        {
            checksum ^= *((uint8_t *) & tSum + i);
        }
        for(; index < len; index++)
        {
            checksum ^= data[index];
        }
    }
    return checksum;
}

static socket_comm_t* AllocateSocketCommHandle() {
  socket_comm_t* handle = Nullptr_t;

  if (s_socket_common_handle_allocate_index < s_max_socket_comm_handle_num) {
    handle = &s_socket_comm_handle[s_socket_common_handle_allocate_index];
    s_socket_common_handle_allocate_index++;
  }

  return (handle);
}

static inline void EncodeProtocol_1(Uint8_t *p, Uint32_t message_id)
{
    Uint16_t payload_len = 1024;
    static Uint16_t sequence =0;
    static Uint32_t couter = 0;
    sequence++;
    couter++;
    Uint64_t timestamp=common::GetClockNowMs();

    p[0]=payload_len >> 8;
    p[1]=payload_len;

    p[2] = timestamp >> 56;
    p[3] = timestamp >> 48;
    p[4] = timestamp >> 40;
    p[5] = timestamp >> 32;
    p[6] = timestamp >> 24;
    p[7] = timestamp >> 16;
    p[8] = timestamp >> 8;
    p[9] = timestamp;

    p[10]=sequence >> 8;
    p[11]=sequence;

    p[12]=message_id >> 24;
    p[13]=message_id >> 16;
    p[14]=message_id >> 8;
    p[15]=message_id;

    p[16]=couter >>24;
    p[17]=couter >> 16;
    p[18]=couter >> 8;
    p[19]=couter;
}

/**
 * @brief 将无符号32位整数编码为大端模式的字节流
 * @param[out] p     字节流
 * @param[in] v      无符号32位整数
 */
static inline void EncodeU32(Uint8_t *p, Uint32_t v) {
  // big endian. p[3] gets lowest 8 bits.
  p[3] = v & 0xff;
  v >>= 8;
  p[2] = v & 0xff;
  v >>= 8;
  p[1] = v & 0xff;
  v >>= 8;
  p[0] = v & 0xff;
}

/**
 * @brief 将大端模式的字节流解码为无符号32位整数
 * @param[in] p     字节流
 * @return 无符号32位整数
 */
static inline Uint32_t DecodeU32(const Uint8_t *p) {
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

/**
 * @brief 将无符号16位整数编码为大端模式的字节流
 * @param[out] p     字节流
 * @param[in] v      无符号16位整数
 */
static inline void EncodeU16(Uint8_t *p, Uint16_t v) {
  // big endian. p[1] gets lowest 8 bits.
  p[1] = v & 0xff;
  v >>= 8;
  p[0] = v & 0xff;
}

/**
 * @brief 将大端模式的字节流解码为无符号16位整数
 * @param[in] p     字节流
 * @return 无符号16位整数
 */
static inline Uint16_t DecodeU16(const Uint8_t *p) {
  Uint16_t v = 0;

  // big endian. p[0] gets most significant bits.
  v |= p[0];
  v <<= 8;
  v |= p[1];

  return v;
}


static uint64_t DeCode64Intel(uint8_t *u_data, uint16_t &u_pos)
{
    if (!u_data)
    {
        std::cout << __func__ << ":" << __LINE__ << " invalid parameter!" << std::endl;
        return 0;
    }
    uint64_t u_64 = 0;
    for (int i = 8 - 1; i >= 0; --i)
    {
        u_64 <<= 8;
        u_64 |= u_data[i];
    }
    u_pos += 8;
    return u_64;
}


static void PrintBuf(const char*func, int32_t lines, Uint8_t *buf, Uint32_t buf_len)
{
    std::cout<<func<<":"<<lines<<" buf_len: "<<buf_len<<" data："<<std::endl;
    if((buf != NULL) && (buf_len))
    {
        std::ios::fmtflags f(std::cout.flags());
        for(uint16_t i = 0; i < buf_len; ++i)
        {
            std::cout << std::hex <<std::setfill('0')<<std::setw(2)<<int(buf[i])<<" ";
        }
        std::cout<<std::dec<<std::endl;
        std::cout.flags(f);
    }
}


/**
 * @brief 消息接收完毕后，将完整的数据通过回调函数的形式发送给用户。
 * @param[in] handle     套接字通信结构体的句柄
 * @param[in] channel    消息的通道名
 * @param[in] buf        消息的完整数据
 * @param[in] buf_len    消息的完整数据的字节大小
 * @detail Called by socket communication internally when a packet is decoded.
 *         (Provides a common delivery code path for fragmented and
 *         non-fragmented packets.)
 */
static void DeliverPacket(socket_comm_t *handle, const Char_t *channel, Uint64_t timestamp,
                          const void *buf, Int32_t buf_len) {
//   printf("deliver packet, channel %-64s, size %10d\n", channel, buf_len);
  // printf("-------------------------------\n");
  for (socket_subscription_t *sub = handle->first_subscription;
        sub != Nullptr_t; sub = sub->next) {
    Int32_t good = 1;

    // printf("%s --- %s\n", channel, sub->channel);

    for (Int32_t pos = 0; pos <= SOCKET_COMM_MAX_CHANNEL_LENGTH; pos++) {
      // printf("pos=%d, sub->channel[pos]=%c, channel[pos]=%c\n",
      //        pos, sub->channel[pos], channel[pos]);
      if (sub->channel[pos] == channel[pos]) {
        // end of string? if so, we're done.
        if (channel[pos] == 0) {
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
      sub->callback(channel, timestamp, buf, buf_len, false, sub->user);
    } else {
      // printf("Ln99. Not good.\n");
    }
  }
}

/**
 * @brief 对套接字通信进行初始化
 * @param[in] handle 套接字通信结构体的句柄
 * @param[in] transmit_packet 发送数据的回调函数
 * @param[in] transmit_user 发送时需要的一些变量
 * @return 0 - 成功；其他 - 失败。
 */
static Int32_t SocketCommInit(socket_comm_t *handle,
                              void (*transmit_packet)(const void *buf_in,
                                                      Int32_t buf_len, void *user),
                              void *transmit_user) {
  // The caller allocates permanent storage for Socket communication.

  common::com_memset(handle, 0, sizeof(socket_comm_t));
  handle->transmit_packet = transmit_packet;
  handle->transmit_user = transmit_user;

  return 0;
}

static Int32_t gSocketCommReceiveNoProtocolPacket(socket_comm_t *handle,
                                                  const void *buf_in,
                                                  Int32_t buf_len,
                                                  uint64_t from_addr) {

}

/**
 * @brief 接收从套接字通信获取的消息，并对分包的数据进行合并。
 * @param[in] handle 套接字通信结构体的句柄
 * @param[in] buf_in 接收到的单个数据包的数据
 * @param[in] buf_len 接收到的单个数据包的数据长度
 * @param[in] from_addr 数据来源的地址
 * @detail The user is responsible for creating and listening on a UDP
 *         multicast socket. When a packet is received, call this function. Do
 *         not call this function from more than one thread at a time. Returns
 *         zero if the packet was successfully handled, however no special
 *         action is required by the caller when an error occurs.
 * @return 0 - 成功；其他 - 失败。
 */
static Int32_t gSocketCommReceivePacket(socket_comm_t *handle,
                                       const void *buf_in,
                                       Int32_t buf_len,
                                       uint64_t from_addr,
                                       Uint64_t timestamp) {
  /** Call this function whenever an Socket UDP packet is
   * received. Registered Socket communication handlers will be called
   * synchronously. When the function returns, the buffer can be safely
   * reused. Returns non-zero if the packet was not decoded properly,
   * but no special action is required by the caller.
   *
   * from_addr is opaque, but should uniquely identify the sender's IP
   * address and port.
   **/

  Uint8_t *buf = (Uint8_t*)buf_in;
  Int32_t buf_pos = 0;
  // printf("\nLn181. buf_len=%d\n", buf_len);

  // not even a header's length
  if (buf_len < 4) {
    return -1;
  }

  Uint32_t magic = DecodeU32(&buf[buf_pos]);
  buf_pos += 4;
  // printf("Ln190. magic=0x%x\n", magic);

  if (magic == MAGIC_SOCKET_COMM_2) {
    Uint32_t msg_seq = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    (void)msg_seq; // quiet unused variable warning.

    // copy out zero-terminated string holding the channel #.
    char channel[SOCKET_COMM_MAX_CHANNEL_LENGTH + 1]={0};
    Int32_t channel_len = 0;

    while (buf[buf_pos] != 0) {
      // malformed packet.
      if ((buf_pos >= buf_len) ||
          (channel_len >= SOCKET_COMM_MAX_CHANNEL_LENGTH)) {
        return -2;
      }

      channel[channel_len++] = buf[buf_pos++];
    }
    channel[channel_len] = 0;
    buf_pos++; // skip the zero.

    DeliverPacket(handle, channel, timestamp, &buf[buf_pos], buf_len - buf_pos);
  } else if (magic == MAGIC_SOCKET_COMM_3) {
    if (SOCKET_COMM_3_NUM_BUFFERS == 0) {
      return -3;
    }

    Uint32_t msg_seq = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    Uint32_t msg_size = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    Uint32_t fragment_offset = DecodeU32(&buf[buf_pos]);
    buf_pos += 4;
    Uint32_t fragment_id = DecodeU16(&buf[buf_pos]);
    buf_pos += 2;
    Uint32_t fragments_in_msg = DecodeU16(&buf[buf_pos]);
    buf_pos += 2;

    int payload_len = buf_len - buf_pos;

    // printf("0x%016lx:%08x %d / %d\n",
    //        from_addr, msg_seq, fragment_id, fragments_in_msg);

    // validate packet metadata
    if (msg_size > SOCKET_COMM_3_MAX_PACKET_SIZE) {
      return -4;
    }

    if (fragments_in_msg > SOCKET_COMM_3_MAX_FRAGMENTS) {
      return -5;
    }

    if (fragment_id >= fragments_in_msg) {
      // printf("fragment_id=%d,fragments_in_msg=%d\n",
      //        (int)fragment_id, (int)fragments_in_msg);
      return -61;
    }
    // 分包时，第1个包的payload_len还包含通道名
    if (fragment_id == 0) {
      if ((fragment_offset + payload_len) >
          (msg_size + SOCKET_COMM_MAX_CHANNEL_LENGTH + 1)) {
        // printf("Ln190. fragment_offset=%d,payload_len=%d,msg_size=%d\n",
        //        fragment_offset, payload_len, msg_size);
        return -62;
      }
    } else if ((fragment_offset + payload_len) > msg_size) {
      // printf("Ln258. fragment_offset=%d,payload_len=%d,msg_size=%d\n",
      //        fragment_offset, payload_len, msg_size);
      return -63;
    }

    // find the fragment. Use a simple linear search; this is
    // cheap in comparison to how much work we're spending to
    // decode the large packet...
    struct FragmentBuffer *fbuf = Nullptr_t;

    // try to find a reassembly buffer for this from_addr that's
    // already in progress
    for (Int32_t idx = 0; idx < SOCKET_COMM_3_NUM_BUFFERS; idx++) {
      if (handle->fragment_buffers[idx].from_addr == from_addr &&
          handle->fragment_buffers[idx].msg_seq == msg_seq) {
        fbuf = &handle->fragment_buffers[idx];
        break;
      }
    }

    if (fbuf == Nullptr_t) {
      // didn't find one. Pick a new buffer to use.

      // Priorities:
      //   1) an idle (complete) buffer
      //   2) the incomplete buffer that received a valid fragment the
      //      longest time ago.
      Int32_t max_age = -1; // low scores are good.
      for (Int32_t idx = 0; idx < SOCKET_COMM_3_NUM_BUFFERS; idx++) {
        if (handle->fragment_buffers[idx].fragments_remaining == 0) {
          fbuf = &handle->fragment_buffers[idx];
          break;
        } else {
          Int32_t age = handle->last_fragment_count -
              handle->fragment_buffers[idx].last_fragment_count;
          if (age > max_age) {
            fbuf = &handle->fragment_buffers[idx];
            max_age = age;
          }
        }
      }

      if (fbuf == Nullptr_t) {
        return -7; // this should never happen
      }

      // initialize the fragment buffer
      for (Uint32_t i = 0; i < fragments_in_msg; i++) {
        fbuf->frag_received[i] = 0;
      }

      fbuf->from_addr = from_addr;
      fbuf->msg_seq = msg_seq;
      fbuf->fragments_remaining = fragments_in_msg;
    }

    // now, handle this fragment
    fbuf->last_fragment_count = handle->last_fragment_count;
    handle->last_fragment_count++;

    if (fragment_id == 0) {
      // this fragment contains the channel name plus data
      Int32_t channel_len = 0;
      while (buf[buf_pos] != 0) {
        if ((buf_pos >= buf_len) ||
            (channel_len >= SOCKET_COMM_MAX_CHANNEL_LENGTH)) {
          return -8;
        }
        fbuf->channel[channel_len++] = buf[buf_pos++];
      }
      fbuf->channel[channel_len] = 0;
      buf_pos++; // skip the zero.
    }

    if (buf_pos < buf_len) {
      common::com_memcpy(&fbuf->buf[fragment_offset],
                         &buf[buf_pos], buf_len - buf_pos);
    }

    // record reception of this packet
    if (fbuf->frag_received[fragment_id] == 0) {
      fbuf->frag_received[fragment_id] = 1;
      fbuf->fragments_remaining--;

      if (fbuf->fragments_remaining == 0) {
        DeliverPacket(handle, fbuf->channel, timestamp, fbuf->buf, msg_size);
      }
    }
  }

  return (0);
}


/**
 * @brief DecodeProtocol_1 解析应用层协议1 Motorola 格式
 * @param buf  缓存入口
 * @param udpp_1  返回值指针
 */
void DecodeProtocol_1(Uint8_t *buf, UDPP1_t *udpp_1)
{
    udpp_1->payload_len = buf[0] << 8 | buf[1];
    udpp_1->timestamp= Uint64_t(buf[2]) << 56 | Uint64_t(buf[3]) << 48 |
                       Uint64_t(buf[4]) << 40 | Uint64_t(buf[5]) << 32 |
                       buf[6] << 24 | buf[7] << 16 | buf[8] << 8 | buf[9];
    udpp_1->sequence_number=buf[10] << 8 | buf[11];

    udpp_1->message_id=buf[12] << 24 | buf[13] << 16 | buf[14] << 8 | buf[15];

    udpp_1->message_counter=buf[16] << 24 | buf[17] << 16 | buf[18] << 8 | buf[19];
//    std::cout<<"udpp_1->timestamp: "<<udpp_1->timestamp<<std::endl;
//    std::cout<<"udpp_1->sequence_number: "<<udpp_1->sequence_number<<std::endl;
//    std::cout<<"udpp_1->message_id: "<<udpp_1->message_id<<std::endl;
//    std::cout<<"udpp_1->message_counter: "<<udpp_1->message_counter<<std::endl;
}

void DecodeProtocol_2(Uint8_t *buf, UDPP2_t *udpp_2)
{
    udpp_2->fragment_type= buf[0] & 0x0f;
    udpp_2->payload_len = ((buf[1] >> 4) << 8) | ((buf[0] >> 4) | (buf[1] << 4));
    udpp_2->buf = buf + 2;
}



void DecodeProtocol_2_ext(Uint8_t *buf, UDPP2_EXT_t *udpp_2_ext)
{
    udpp_2_ext->fragment_type=buf[0] & 0x0f;
    udpp_2_ext->payload_len=((buf[1] >> 4) << 8) | ((buf[0] >> 4) | (buf[1] << 4));
    udpp_2_ext->all_fragment_sequence
             =Uint64_t(buf[9]) << 56 |
              Uint64_t(buf[8]) << 48 |
              Uint64_t(buf[7]) << 40 |
              Uint64_t(buf[6]) << 32 |
              Uint64_t(buf[5]) << 24 | Uint64_t(buf[4]) << 16 | Uint64_t(buf[3]) << 8 | Uint64_t(buf[2]);
    udpp_2_ext->single_group_number=buf[13] << 24 | buf[12] << 16 | buf[11] << 8 | buf[10];
    udpp_2_ext->single_group_fragment_count=buf[15] << 8 | buf[14];
    udpp_2_ext->single_group_sequence=buf[17] << 8 | buf[16];
    udpp_2_ext->checksum = buf[18];
    udpp_2_ext->buf = buf+18;
//    std::cout<<"udpp_2_ext->payload_len: "<<udpp_2_ext->payload_len<<std::endl;
//    std::cout<<"udpp_2_ext->fragment_type111: "<<(uint16_t)udpp_2_ext->fragment_type<<std::endl;
//    uint16_t upos = 2;
//    uint64_t allSN = DeCode64Intel(buf + upos, upos);
//    std::cout<<"all_fragment_sequence: "<<udpp_2_ext->all_fragment_sequence << " allSN:" << allSN <<std::endl;

//    std::cout<<"single_group_number: "<<udpp_2_ext->single_group_number<<std::endl;
//    std::cout<<"single_group_fragment_count: "<<udpp_2_ext->single_group_fragment_count<<std::endl;
//    std::cout<<"single_group_sequence: "<<udpp_2_ext->single_group_sequence<<std::endl;
//    PrintBuf(__func__, __LINE__, buf, (Int32_t)udpp_2_ext->payload_len);
}


void DecodeProtocol_2_ext(Uint8_t *buf, Uint32_t buf_len, UDPP2_EXT_t *udpp_2_ext)
{
//    PrintBuf(__func__, __LINE__, buf, (Int32_t)buf_len);
    udpp_2_ext->fragment_type=buf[0] & 0x0f;
    udpp_2_ext->payload_len=((buf[1] >> 4) << 8) | ((buf[0] >> 4) | (buf[1] << 4));
    udpp_2_ext->all_fragment_sequence
             =Uint64_t(buf[9]) << 56 |
              Uint64_t(buf[8]) << 48 |
              Uint64_t(buf[7]) << 40 |
              Uint64_t(buf[6]) << 32 |
              Uint64_t(buf[5]) << 24 | Uint64_t(buf[4]) << 16 | Uint64_t(buf[3]) << 8 | Uint64_t(buf[2]);
    udpp_2_ext->single_group_number=buf[13] << 24 | buf[12] << 16 | buf[11] << 8 | buf[10];
    udpp_2_ext->single_group_fragment_count=buf[15] << 8 | buf[14];
    udpp_2_ext->single_group_sequence=buf[17] << 8 | buf[16];
    udpp_2_ext->checksum = buf[18];
    udpp_2_ext->buf = buf+18;
//    std::cout<< __func__ << ":" << __LINE__ << " payload_len:"<<udpp_2_ext->payload_len
//        <<" fragment_type:"<<(uint16_t)udpp_2_ext->fragment_type
//        <<" all_fragment_sequence:"<<udpp_2_ext->all_fragment_sequence
//        <<" single_group_number:"<<udpp_2_ext->single_group_number
//        <<" single_group_fragment_count:"<<udpp_2_ext->single_group_fragment_count
//        <<" single_group_sequence:"<<udpp_2_ext->single_group_sequence<<std::endl;
}


Int32_t UdpIDNode::SocketCommReceivePacketByMessageID(socket_comm_t *handle,
                                                       const void *buf_in,
                                                       Int32_t buf_len,
                                                       uint64_t from_addr,
                                                       Uint64_t timestamp) {
    Uint8_t *buf = (Uint8_t*)buf_in;
    Int32_t buf_pos = 0;

    if(buf_len < 22) { // 协议1+协议2 头部字节
        return  -1;
    }

    UDPP1_t udpp_1;
    DecodeProtocol_1(buf, &udpp_1);  // 解析协议1头

    if(udpp_1.payload_len <= 22 || udpp_1.payload_len > 1024)
        return -2;

//    LOG_INFO(3) << "protocol_1 message id,sequence=" << udpp_1.message_id <<',' << udpp_1.sequence_number;

    buf_pos += 20;

#if 0
    if(udpp_1.message_id == 0x8001101) { // 视觉车道线&定位数据
        return ReceiveLanePositionDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x8001102 || udpp_1.message_id == 0x8001103 ||
            udpp_1.message_id == 0x8001104 || udpp_1.message_id == 0x8001105) { // 地图数据
        return ReceiveMapDisplayData(handle, buf + buf_pos, buf_len - buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x800110A) { // 感知数据
        return ReceivePerceptionDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x800110B) { // 系统显示数据
        return ReceiveSystemDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x800110C) { // 规划显示数据：行为规划结果、轨迹规划结果、决策结果和速度规划结果 33492
        return ReceivePlanningSpdPlnBhvPlnDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x800110D ||
            udpp_1.message_id == 0x800110E) { // 规划地图数据  1543997
        return ReceivePlanningPathPlnDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }
    else if(udpp_1.message_id == 0x8001106) { // 控制模块数据
      return ReceiveControlDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    }

    return 0;

#else

  MSG_BUF_META *msg_buf_meta = Nullptr_t;
  if(udpp_1.message_id == 0x8001101) { // 视觉车道线&定位数据
    msg_buf_meta= Nullptr_t;
  } else if (udpp_1.message_id == 0x8001102 || udpp_1.message_id == 0x8001103 ||
          udpp_1.message_id == 0x8001104 || udpp_1.message_id == 0x8001105) { // 地图数据
    msg_buf_meta = &msg_meta_map;
  } else if (udpp_1.message_id == 0x800110A) { // 感知数据
//    return ReceivePerceptionDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    msg_buf_meta = &msg_meta_perception;

  } else if (udpp_1.message_id == 0x800110B) { // 系统显示数据
//  return ReceiveSystemDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    msg_buf_meta = &msg_meta_system;
  } else if (udpp_1.message_id == 0x800110C) { // 规划显示数据：行为规划结果、轨迹规划结果、决策结果和速度规划结果 33492
//    return ReceivePlanningSpdPlnBhvPlnDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    msg_buf_meta = &msg_meta_spdpln;
  } else if (udpp_1.message_id == 0x800110D ||
          udpp_1.message_id == 0x800110E) { // 规划地图数据  1543997
//          return ReceivePlanningPathPlnDisplayData(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1);
    msg_buf_meta = &msg_meta_pathpln;
  } else if (udpp_1.message_id == 0x8001106) { // 控制模块数据
    msg_buf_meta=Nullptr_t;
  }

  return SocketCommReceivePacketByMessageIDCommon(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1, msg_buf_meta, timestamp);

#endif
}

std::string get_id_channel_name(Uint32_t mid)
{
  std::string channel_name="";
  if(mid == 0x8001101) { // 视觉车道线&定位数据
    channel_name = "adecu/Position_LanePosition";
  }
  else if(mid == 0x8001102 || mid == 0x8001103 ||
          mid == 0x8001104 || mid == 0x8001105) { // 地图数据
    channel_name = "adecu/Position_Map";
  }
  else if(mid == 0x800110A) { // 感知数据
    channel_name = "adecu/Perception_Display_Data";
  }
  else if(mid == 0x800110B) { // 系统显示数据
    channel_name = "adecu/System_Display_Data";
  }
  else if(mid == 0x800110C) { // 规划显示数据：行为规划结果、轨迹规划结果、决策结果和速度规划结果 33492
    channel_name = "adecu/PlanningSpdPlnBhvPlnDisplayData";
  }
  else if(mid == 0x800110D ||
          mid == 0x800110E) { // 规划地图数据  1543997
    channel_name = "adecu/PlanningPathPlnDisplayData";
  }
  else if(mid == 0x8001106) { // 控制模块数据
    channel_name = "adecu/Control_Display_Data";
  }
  else if(mid == 0x8005020 || mid== 0x8005021 || mid == 0x8005022 || mid == 0x8005023 ||
          mid == 0x8005024 || mid == 0x8005025 || mid == 0x8005026 || mid == 0x8005027 ||
          mid == 0x8005028 || mid == 0x8005029 || mid == 0x800502A || mid == 0x800502B) // FitSamplePoint
  {
    channel_name = "mpu/FitSamplePoint";
  }
  else if(mid == 0x8005001) // VehicleCorrectState
  {
    channel_name = "mpu/VehicleCorrectState";
  }
  else if(mid == 0x8005002) // VehicleState
  {
    channel_name = "mpu/VehicleState";
  }
  else if(mid == 0x8005003) { // mpu CorrectPosition数据
    channel_name = "mpu/CorrectPosition";
  }
  else if(mid == 0x8005004 || mid == 0x8005005 || mid == 0x8005006 || mid == 0x8005007)  // PartialMap
  {
    channel_name = "mpu/PartialMap";
  }
  else if(mid == 0x8005008 || mid == 0x8005009 || mid == 0x800500A || mid == 0x800500B) // PartialMapLineFit
  {
    channel_name = "mpu/PartialMapLineFit";
  }
  else if(mid == 0x800500C || mid == 0x800500D || mid == 0x800500E || mid == 0x800500F) // OrigCamera
  {
    channel_name = "mpu/OrigCamera";
  }
  else if(mid == 0x800501C || mid == 0x800501D || mid == 0x800501E || mid == 0x800501F)
  {
      channel_name = "mpu/CameraFitLine";
  }
  else if(mid == 0x8005010 || mid == 0x8005011 || mid == 0x8005012 || mid == 0x8005013 ||
          mid == 0x8005014 || mid == 0x8005015 || mid == 0x8005016 || mid == 0x8005017 ||
          mid == 0x8005018 || mid == 0x8005019) //MapLocationEhr
  {
    channel_name = "mpu/MapLocationEhr";
  }
  else if(mid == 0x800501a) //LocationPosition
  {
    channel_name = "mpu/LocationPosition";
  }
  else if(mid == 0x800501b)
  {
    channel_name = "mpu/DecisionPosition";
  }
  else if(mid == 0x8005000)
  {
    channel_name = "mpu/State";
  }

  return channel_name;
}


Uint16_t get_request_err_type_by_message_id(Uint32_t mid)
{
  Uint16_t req_err_id = 0;

  if(mid == 0x8001102 || mid == 0x8001103 ||
     mid == 0x8001104 || mid == 0x8001105) { // 地图数据
    req_err_id=0b10;
  }
  else if(mid == 0x800110A) { // 感知数据
    req_err_id=0B100;
  }
  else if(mid == 0x800110B) { // 系统显示数据
    req_err_id=0b1000;
  }
  else if(mid == 0x800110C) { // 规划显示数据：行为规划结果、轨迹规划结果、决策结果和速度规划结果 33492
    req_err_id=0B10000;
  }
  else if(mid == 0x800110D ||
          mid == 0x800110E) { // 规划地图数据  1543997
    req_err_id=0B1000000;
  }
  else if(mid == 0x8005003) { // mpu CorrectPosition数据
    req_err_id = 0B10000000000;
  }

  return req_err_id;
}

uint64_t get_request_err_type_by_message_id_mpu(Uint32_t mid, int& pos)
{
  uint64_t req_err_id = 0;

  if(mid == 0x8001102 || mid == 0x8001103 ||
     mid == 0x8001104 || mid == 0x8001105) { // 地图数据
    req_err_id=0b10;
    pos = 2;
  }
  else if(mid == 0x800110A) { // 感知数据
    req_err_id=0B100;
    pos = 3;
  }
  else if(mid == 0x800110B) { // 系统显示数据
    req_err_id=0b1000;
    pos = 4;
  }
  else if(mid == 0x800110C) { // 规划显示数据：行为规划结果、轨迹规划结果、决策结果和速度规划结果 33492
    req_err_id=0B10000;
    pos = 5;
  }
  else if(mid == 0x800110D ||
          mid == 0x800110E) { // 规划地图数据  1543997
    req_err_id=0B1000000;
    pos = 7;
  }
  else if(mid == 0x8005020 || mid== 0x8005021 || mid == 0x8005022 || mid == 0x8005023 ||
          mid == 0x8005024 || mid == 0x8005025 || mid == 0x8005026 || mid == 0x8005027 ||
          mid == 0x8005028 || mid == 0x8005029 || mid == 0x800502A || mid == 0x800502B) // FitSamplePoint
  {
    req_err_id = 0B100000000;
    pos = 9;
  }
  else if(mid == 0x8005001) // VehicleCorrectState
  {
    req_err_id = 0B1000000000000000;
    pos = 16;
  }
  else if(mid == 0x8005002) // VehicleState
  {
    req_err_id = 0B100000000000000;
    pos = 15;
  }
  else if(mid == 0x8005003) { // mpu CorrectPosition数据
    req_err_id = 0B10000000000;
    pos = 11;
  }
  else if(mid == 0x8005004 || mid == 0x8005005 || mid == 0x8005006 || mid == 0x8005007)  // PartialMap
  {
    req_err_id = 0B1000000000000;
    pos = 13;
  }
  else if(mid == 0x8005008 || mid == 0x8005009 || mid == 0x800500A || mid == 0x800500B) // PartialMapLineFit
  {
    req_err_id = 0B100000000000;
    pos = 12;
  }
  else if(mid == 0x800500C || mid == 0x800500D || mid == 0x800500E || mid == 0x800500F) // OrigCamera
  {
    req_err_id = 0B10000000000000;
    pos = 14;
  }
  else if(mid == 0x8005010 || mid == 0x8005011 || mid == 0x8005012 || mid == 0x8005013 ||
          mid == 0x8005014 || mid == 0x8005015 || mid == 0x8005016 || mid == 0x8005017 ||
          mid == 0x8005018 || mid == 0x8005019) //MapLocationEhr
  {
    req_err_id = 0B1000000000;
    pos = 10;
  }
  else if(mid == 0x800501a)
  {
    req_err_id = 0B10000000000000000;
    pos = 17;
  }
  else if(mid == 0x800501b)
  {
    req_err_id = 0B100000000000000000;
    pos = 18;
  }


  return req_err_id;
}


Uint16_t UdpIDNode::GetRequestResendErrorCode()
{
  Uint16_t req_err_code = 0;
  if(msg_meta_map.request_resend) {
    req_err_code |= 0b10;  // map position
  } else {
    req_err_code |= ~0b10;
  }

  if(msg_meta_perception.request_resend) { // perception
    req_err_code |= 0B100;
  } else {
    req_err_code |= ~0B100;
  }

  if(msg_meta_system.request_resend) { // system
    req_err_code |= 0b1000;
  } else {
    req_err_code |= ~0b1000;
  }

  if(msg_meta_spdpln.request_resend) { // plan spd
    req_err_code |= 0B10000;
  } else {
    req_err_code |= ~0B10000;
  }

  if(msg_meta_pathpln.request_resend) { // plan
    req_err_code |= 0B1000000;
  } else {
    req_err_code |= ~0B1000000;
  }

  return req_err_code;
}

Int32_t UdpIDNode::SocketCommReceivePacketByMessageIDCommon(
    socket_comm_t *handle,
    Uint8_t *buf, Uint32_t buf_len,
    UDPP1_t *udpp_1,
    MSG_BUF_META *msg_buf_meta,
    Uint64_t timestamp)
{
  std::string channel_name = get_id_channel_name(udpp_1->message_id);

  Uint32_t buf_pos=0;
  Uint8_t fragment_type = 0;
  Int32_t ret=0;

  fragment_type= buf[0] & 0x0f;

  // 单个包接收
  if(fragment_type == FRAGMENT_TYPE_NOSPLIT) {
    UDPProtocol_2 udpp_2;
    DecodeProtocol_2(buf, &udpp_2);
    buf_pos += 2;
    ret=buf_len-buf_pos;

    std::string channel_name = get_id_channel_name(udpp_1->message_id);
    DeliverPacket(handle, channel_name.c_str(), timestamp, buf + buf_pos, buf_len - buf_pos);
    return ret;
  }
  else if(fragment_type == FRAGMENT_TYPE_START ||
          fragment_type == FRAGMENT_TYPE_SPLIT ||
          fragment_type == FRAGMENT_TYPE_END) {

    // 拆包组包处理
    Uint8_t* cache = msg_buf_meta->cache;
    Uint32_t& cache_size=msg_buf_meta->cache_size;
    Uint64_t& all_fragment_sequence=msg_buf_meta->all_fragment_sequence;
    bool& recv_completed=msg_buf_meta->recv_completed;
    bool& request_resend=msg_buf_meta->request_resend;
    Uint64_t& request_sequence=msg_buf_meta->request_sequence;
    Uint64_t& single_group_fragment_count=msg_buf_meta->single_group_fragment_count; // 记录单组接收到的分包总数
    Uint64_t& ts_exception=msg_buf_meta->ts_exception; // 记录不正常帧的开始时间
    bool& first_frame = msg_buf_meta->first_frame;
    Int64_t& recv_fist_frame_timestamp=msg_buf_meta->recv_fist_frame_timestamp;
    Int64_t& recv_last_frame_timestamp=msg_buf_meta->recv_last_frame_timestamp;

    UDPP2_EXT_t udpp_2_ext;

    DecodeProtocol_2_ext(buf + buf_pos, &udpp_2_ext);
    if(udpp_2_ext.payload_len <= 17 || udpp_2_ext.payload_len > 1002)
      return -3;

//    LOG_INFO(3) << "----------- position map mid,(group),frag_type,last_seq,seq= 0x"
//                << get_dec2hex_str(udpp_1->message_id).c_str() << " ("
//                << udpp_2_ext.single_group_number <<','
//                << udpp_2_ext.single_group_fragment_count <<','
//                << udpp_2_ext.single_group_sequence << ") "
//                << udpp_2_ext.fragment_type<<','
//                << all_fragment_sequence << ','
//                << udpp_2_ext.all_fragment_sequence;

    // 检查校验码
    if(check_crc_8(buf, buf_len) == 0) { // 校验码不正确
      LOG_INFO(3) << channel_name.c_str() << " mid= 0x" << get_dec2hex_str(udpp_1->message_id).c_str() << ", seq=" << udpp_2_ext.all_fragment_sequence << ", chksum mismatch, dropped";
      return -4;
    }
    else {

      if(request_resend) {
        if(udpp_2_ext.all_fragment_sequence == request_sequence) {
          request_resend = false;
          LOG_INFO(3) << channel_name.c_str() << " mid= 0x" << get_dec2hex_str(udpp_1->message_id).c_str() << " get request seq(" << udpp_2_ext.all_fragment_sequence << ")";
          if(udpp_2_ext.all_fragment_sequence == FRAGMENT_TYPE_END) {
            recv_completed=true;
          }

          ts_exception=0;
          all_fragment_sequence=request_sequence;
        }
        else {
          Uint64_t ts_now = phoenix::common::GetClockNowUs();
          Int64_t ts_gap=phoenix::common::CalcElapsedClockUs(ts_exception, ts_now);
          if(ts_gap > 100000) { // 丢帧请求超时，重新接收起始总帧序号
            all_fragment_sequence=0;
            request_sequence=0;
            single_group_fragment_count=0;
            ts_exception=0;
            first_frame=true;
            request_resend=false;
            cache_size=0;

            LOG_INFO(3) << channel_name.c_str() << " connection reset";
            return 0;
          }
        }
      }
      else {
        if(first_frame == true) {
          if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_START) {
            first_frame=false;
            all_fragment_sequence = udpp_2_ext.all_fragment_sequence;
            recv_fist_frame_timestamp=timestamp;
          }
          else {
            LOG_INFO(3) << channel_name.c_str() << " wait for first frame";
            return 0;
          }
        }
        else {
          if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_START ||
             udpp_2_ext.fragment_type == FRAGMENT_TYPE_SPLIT ||
             udpp_2_ext.fragment_type == FRAGMENT_TYPE_END ) {
            if(all_fragment_sequence+1 == udpp_2_ext.all_fragment_sequence) {
              all_fragment_sequence = udpp_2_ext.all_fragment_sequence;
              if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_END) {
                recv_completed=true;
              }
            }
            else {
              ts_exception=phoenix::common::GetClockNowUs();
              request_resend=true;
              request_sequence=all_fragment_sequence+1;
            }
          }
        }
      }
    }

    if(request_resend) {

      char buf[1004]={0};
      Uint32_t resend_msg_id=0x8005201;
      Uint16_t err=GetRequestResendErrorCode();
      buf[0]=err;
      buf[1]=err >> 8;

      // map position
      buf[6]= msg_meta_map.request_sequence;
      buf[7]= msg_meta_map.request_sequence >> 8;
      buf[8]= msg_meta_map.request_sequence >> 16;
      buf[9]= msg_meta_map.request_sequence >> 24;

      // perception
      buf[10]= msg_meta_perception.request_sequence;
      buf[11]= msg_meta_perception.request_sequence >> 8;
      buf[12]= msg_meta_perception.request_sequence >> 16;
      buf[13]= msg_meta_perception.request_sequence >> 24;

      // system
      buf[14]= msg_meta_system.request_sequence;
      buf[15]= msg_meta_system.request_sequence >> 8;
      buf[16]= msg_meta_system.request_sequence >> 16;
      buf[17]= msg_meta_system.request_sequence >> 24;

      // plan spd
      buf[18]= msg_meta_system.request_sequence;
      buf[19]= msg_meta_system.request_sequence >> 8;
      buf[20]= msg_meta_system.request_sequence >> 16;
      buf[21]= msg_meta_system.request_sequence >> 24;
      // plan path
      buf[26]= msg_meta_pathpln.request_sequence;
      buf[27]= msg_meta_pathpln.request_sequence >> 8;
      buf[28]= msg_meta_pathpln.request_sequence >> 16;
      buf[29]= msg_meta_pathpln.request_sequence >> 24;

      usleep(10000);
      Publish(resend_msg_id, buf, 1004);

      LOG_INFO(3) << channel_name.c_str() <<", "
                  << "mid= 0x" << get_dec2hex_str(udpp_1->message_id).c_str() <<" ,"
                  << "err code= 0B" << get_dec2bin_str(err).c_str() << ", "
                  << "seq=" << request_sequence << " request sended, "
                  <<"current seq=" << udpp_2_ext.all_fragment_sequence
                  <<" dropped";

      return 0;
    }
    else {
      Uint32_t cur_data_len = udpp_2_ext.payload_len - 17;
      buf_pos += 19;
      memcpy(cache + cache_size, buf + buf_pos, cur_data_len);

      cache_size += cur_data_len;
      single_group_fragment_count++;
    }

    if(recv_completed) {
      if(udpp_2_ext.single_group_fragment_count == single_group_fragment_count) {
        recv_last_frame_timestamp = timestamp;
//        LOG_INFO(3) << channel_name.c_str() << " recvive ok, "
//                    << "size=" << cache_size << ", "
//                    << "fragment count=" << single_group_fragment_count
//                    <<" (require " << udpp_2_ext.single_group_fragment_count << ")";
        timestamp = common::GetClockNowUs();
        DeliverPacket(handle, channel_name.c_str(), timestamp, cache, cache_size);
      }
      else {
        LOG_INFO(3) << channel_name.c_str() << " recive fragment count mismatch , "
                    << "count=" << single_group_fragment_count << ", "
                    << "require count=" << udpp_2_ext.single_group_fragment_count;
      }

      single_group_fragment_count=0;
      recv_completed=false;
      ret=cache_size;
      cache_size=0;

      return ret;
    }
  }
  else {
    // not impossible here, error
    LOG_INFO(3) << "error, here impossible";
    return 0;
  }
  return 0;
}



//void UdpIDNode::ThreadCycleReceiveMessage()
//{
//  while(true) {
//      Uint64_t timestamp=common::GetClockNowUs();
//      if(started == false || send_flag_ == false) {
//          usleep(100000);
//          LOG_INFO(3) << "started, send_flag_=" << started << ',' << send_flag_;
//          continue;
//        }
//      char buf[1004]={0};
//      Uint16_t err = 0b10;
//      buf[0] =err;
//      buf[1]=err >> 8;

//      Uint32_t request_sequence=8;
//      buf[6]=request_sequence;
//      buf[7]=request_sequence >> 8;
//      buf[8]=request_sequence >> 16;
//      buf[9]=request_sequence >> 24;

//      Uint32_t resend_msg_id=0x8005201;
//      Publish(resend_msg_id, buf, 1004);
//      LOG_INFO(3) <<"msgid, udpp_2_ext.sequence=" << resend_msg_id <<',' << request_sequence << " request resend (" << started << ','<< send_flag_ << ")";
//      sleep(3);
//    }
//}

/**
 * @brief 预订消息
 * @param[in] handle 套接字通信结构体的句柄
 * @param[in] sub 预订的消息
 * @detail Not thread safe WRT SocketCommReceivePacket. Caller allocates
 *         subscription record and initializes its fields.  Note that the
 *         "channel" field does not support regular expressions, but ending
 *         with ".*" is supported as a special case.
 */
static void SocketCommSubscribe(socket_comm_t *handle,
                                socket_subscription_t *sub) {
  sub->next = handle->first_subscription;
  handle->first_subscription = sub;
}

/**
 * @brief 发送消息。消息的字节数较大时会分为多个数据包进行发送。
 * @param[in] handle 套接字通信结构体的句柄
 * @param[in] channel 消息的通道名
 * @param[in] buf_in 消息内容
 * @param[in] buf_len 消息的字节数
 * @detail Publish a message. Will call transmit_packet function one or more
 *         times synchronously. Returns 0 on success. This function should not
 *         be called concurrently with itself, but can be called concurrently
 *         with SocketCommReceivePacket.
 * @return true - 成功; false - 失败。
 */
static bool gSocketCommPublish(socket_comm_t *handle, const Char_t *channel,
                              const void *buf_in, Int32_t buf_len) {
  // 通道长度
  Int32_t channel_length = 0;
  if (buf_len < SOCKET_COMM_PUBLISH_BUFFER_SIZE - MAXIMUM_HEADER_LENGTH) {
    // publish non-fragmented message
    Uint32_t buf_pos = 0;

    EncodeU32(&handle->publish_buffer[buf_pos], MAGIC_SOCKET_COMM_2);
    buf_pos += 4;
    EncodeU32(&handle->publish_buffer[buf_pos], handle->msg_seq);
    buf_pos += 4;
    handle->msg_seq++;

    // copy channel
    while (*channel != 0) {
      handle->publish_buffer[buf_pos++] = *channel;
      channel++;
      channel_length++;
      // 通道名称的长度不能超过最大长度
      if (channel_length >= SOCKET_COMM_MAX_CHANNEL_LENGTH) {
        break;
      }
    }
    handle->publish_buffer[buf_pos++] = 0;

    common::com_memcpy(&handle->publish_buffer[buf_pos], buf_in, buf_len);
    buf_pos += buf_len;

    handle->transmit_packet(handle->publish_buffer, buf_pos,
                            handle->transmit_user);
  } else {
    // send fragmented message
    Uint32_t msg_seq = handle->msg_seq;
    handle->msg_seq++;

    Uint32_t fragment_offset = 0;
    Uint32_t max_fragment_size =
        SOCKET_COMM_PUBLISH_BUFFER_SIZE - MAXIMUM_HEADER_LENGTH;
    Uint32_t fragment_id = 0;
    Uint32_t fragments_in_msg =
        (buf_len + max_fragment_size - 1) / max_fragment_size;
    // printf("fragments_in_msg = %d, buf_len = %d\n",
    //        fragments_in_msg, buf_len);

    while (static_cast<Int32_t>(fragment_offset) < buf_len) {
      Uint32_t buf_pos = 0;

      EncodeU32(&handle->publish_buffer[buf_pos], MAGIC_SOCKET_COMM_3);
      buf_pos += 4;
      EncodeU32(&handle->publish_buffer[buf_pos], msg_seq);
      buf_pos += 4;
      EncodeU32(&handle->publish_buffer[buf_pos], buf_len);
      buf_pos += 4;
      EncodeU32(&handle->publish_buffer[buf_pos], fragment_offset);
      buf_pos += 4;
      EncodeU16(&handle->publish_buffer[buf_pos], fragment_id);
      buf_pos += 2;
      EncodeU16(&handle->publish_buffer[buf_pos], fragments_in_msg);
      buf_pos += 2;

      // copy channel
      if (fragment_id == 0) {
        while (*channel != 0) {
          handle->publish_buffer[buf_pos++] = *channel;
          channel++;
          channel_length++;
          // 通道名称的长度不能超过最大长度
          if (channel_length >= SOCKET_COMM_MAX_CHANNEL_LENGTH) {
            break;
          }
        }
        handle->publish_buffer[buf_pos++] = 0;
      }

      Uint32_t this_fragment_size = buf_len - fragment_offset;
      if (this_fragment_size > max_fragment_size) {
        this_fragment_size = max_fragment_size;
      }

      common::com_memcpy(&handle->publish_buffer[buf_pos],
             &((char*) buf_in)[fragment_offset], this_fragment_size);
      buf_pos += this_fragment_size;

      handle->transmit_packet(handle->publish_buffer,
                              buf_pos, handle->transmit_user);

      fragment_offset += this_fragment_size;
      fragment_id++;
    }
  }

  return true;
}

static bool gSocketCommPublishByID(socket_comm_t *handle, Uint32_t message_id,
                              const void *buf_in, Int32_t buf_len) {
    Uint32_t buf_pos = 0;
    // 构建协议头
    EncodeProtocol_1(&handle->publish_buffer[buf_pos], message_id);
    buf_pos+=20;

    // 拷贝发送数据
    common::com_memcpy(&handle->publish_buffer[buf_pos], buf_in, buf_len);
    buf_pos += buf_len;

    handle->transmit_packet(handle->publish_buffer, buf_pos, handle->transmit_user);

    return true;
}

/**
 * @brief 发送函数
 * @param[in] buf_in 发送的数据
 * @param[in] buf_len 发送数据的字节长度
 * @param[in] user 发送通道信息
 */
static void TransmitPacket(const void *buf_in, Int32_t buf_len, void *user) {
  SocketSendingChannelInfo *tinfo = (SocketSendingChannelInfo*)user;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  int res = sendto(tinfo->send_fd, (char *)buf_in, buf_len, 0,
                   (SOCKADDR*)&tinfo->send_addr, sizeof(tinfo->send_addr));
  if (res < 0) {
//    LOG_ERR << "Failed to send message to network.";
  }
#else
  ssize_t res = sendto(tinfo->send_fd, buf_in, buf_len, 0,
                       (struct sockaddr*) &tinfo->send_addr,
                       sizeof(tinfo->send_addr));
  if (res < 0) {
      char err[512]={0};
      snprintf(err,511,
         "send to host(%s:%d) failed",
         inet_ntoa(tinfo->send_addr.sin_addr),
         ntohs(tinfo->send_addr.sin_port));
    LOG_ERR << err;
  }
#endif

//    std::cout << __func__ << ":" << __LINE__ << " Transmit time:" <<common::GetClockNowMs() << " len:" << buf_len << " ret:" << res << std::endl;
//    PrintBuf("SendTo", __LINE__, (Uint8_t*)buf_in, buf_len);
  //std::cout << "Transmit " << res << " bytes" << std::endl;
}


/// 对外接口的实现代码


/**
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
UDPComm::UDPComm()
    :d_ptr(new UDPCommPrivate){
  init();
}

UDPComm::UDPComm(UDPCommPrivate &dd)
    :d_ptr(&dd)
{
  init();
}

void UDPComm::init()
{
  d_ptr->receiving_flag_ = false;
  proc_flag_ = false;
  d_ptr->communication_handle_ = Nullptr_t;

  receiving_msg_buf_ = new Char_t[RECEIVING_UDP_MSG_BUF_SIZE];

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    d_ptr->receiving_channel_info_.read_fd = INVALID_SOCKET;
    d_ptr->sending_channel_info_.send_fd = INVALID_SOCKET;
    common::com_memset(&d_ptr->sending_channel_info_.send_addr, 0,
                       sizeof(d_ptr->sending_channel_info_.send_addr));
#else
  d_ptr->receiving_channel_info_.read_fd = -1;
  d_ptr->sending_channel_info_.send_fd = -1;
  common::com_memset(&d_ptr->sending_channel_info_.send_addr, 0,
                     sizeof(d_ptr->sending_channel_info_.send_addr));
#endif

  d_ptr->subscription_count_ = 0;
  for (Int32_t i = 0; i < UDPCommData::MAX_SUBSCRIPTION_NUM; ++i) {
    common::com_memset(d_ptr->subscription_list_[i].channel, 0,
                       sizeof(d_ptr->subscription_list_[i].channel));
    d_ptr->subscription_list_[i].callback = Nullptr_t;
    d_ptr->subscription_list_[i].user = Nullptr_t;
    d_ptr->subscription_list_[i].next = Nullptr_t;
  }
}

UDPComm::~UDPComm()
{
    Stop();
}

/**
 * @brief 创建套接字连接
 * @param[in] comm_param      消息通信连接参数
 * @return true - 成功；false - 失败。
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UDPComm::Start(const UdpParam& comm_param) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((INVALID_SOCKET != d_ptr->sending_channel_info_.send_fd) ||
      (INVALID_SOCKET != d_ptr->receiving_channel_info_.read_fd)) {
    LOG_ERR << "Already start.";
    return false;
  }
#else
  if ((d_ptr->sending_channel_info_.send_fd >= 0) ||
      (d_ptr->receiving_channel_info_.read_fd >= 0)) {
    LOG_ERR << "Already start.";
    return false;
  }
#endif

  if (Nullptr_t == d_ptr->communication_handle_) {
    d_ptr->communication_handle_ = AllocateSocketCommHandle();
    if (Nullptr_t == d_ptr->communication_handle_) {
      LOG_ERR << "Failed to allocate socket communiation handle.";
      return false;
    }
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData)  != 0) {
    LOG_ERR << "WSAStartup failed";
    return false;
  }
#endif

  // 0~1024一般给系统使用，一共可以分配到65535
  if (d_ptr->param_.enable_recv) {
    if ((comm_param.rcv_port < 1025) || (comm_param.rcv_port > 65535)) {
      LOG_ERR << "Invalid receiving port, must be in [1025, 65535].";
      return false;
    }
  }
  if (d_ptr->param_.enable_send) {
    if ((comm_param.snd_remote_port < 1025) || (comm_param.snd_remote_port > 65535)) {
      LOG_ERR << "Invalid sending port, must be in [1025, 65535].";
      return false;
    }
  }

  d_ptr->param_.enable_recv = comm_param.enable_recv;
  d_ptr->param_.enable_send = comm_param.enable_send;
  d_ptr->param_.snd_remote_port = comm_param.snd_remote_port;
  d_ptr->param_.snd_local_port=comm_param.snd_local_port;
  d_ptr->param_.rcv_port = comm_param.rcv_port;
  d_ptr->param_.mode = comm_param.mode;
  common::com_memcpy(d_ptr->param_.snd_addr, comm_param.snd_addr,
                     sizeof(d_ptr->param_.snd_addr));

  if (d_ptr->param_.enable_recv) {
    // 1.创建udp通信socket
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    d_ptr->receiving_channel_info_.read_fd =
        socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  // 指定为UDP套接字
    if (INVALID_SOCKET == d_ptr->receiving_channel_info_.read_fd) {
      int err = WSAGetLastError();
      LOG_ERR << "Failed to create socket fd for receiving channel, err=" << err;
      return false;
    }
#else
    d_ptr->receiving_channel_info_.read_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (d_ptr->receiving_channel_info_.read_fd < 0) {
      LOG_ERR << "Failed to create socket fd for receiving channel.";
      return false;
    }
#endif

    if (UdpParam::MODE_GROUP == d_ptr->param_.mode) {
      // 加入组播, 组播地址224.0.0.0~239.255.255.255
      struct ip_mreq group;
      common::com_memset(&group, 0, sizeof(group));
      LOG_INFO(3) << "group ip = " << d_ptr->param_.snd_addr;
      group.imr_multiaddr.s_addr = inet_addr(d_ptr->param_.snd_addr);  //设置组播地址
      group.imr_interface.s_addr =  inet_addr("0.0.0.0");//htonl(INADDR_ANY);//
      int ret = -1;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
      ret = setsockopt(d_ptr->receiving_channel_info_.read_fd,
                     IPPROTO_IP, IP_ADD_MEMBERSHIP,
                           (const char*)&group, sizeof(group));
#else
      ret = setsockopt(d_ptr->receiving_channel_info_.read_fd,
                       IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       &group, sizeof(group));
#endif
      if (ret < 0) {
        close(d_ptr->receiving_channel_info_.read_fd);
        d_ptr->receiving_channel_info_.read_fd = -1;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        LOG_ERR << "Failed to add to group.";
#else
        LOG_ERR << strerror(errno) <<", please add group ip to route table";
#endif
        return false;
      }
    }

    // 2.设置UDP的地址并绑定
    struct sockaddr_in read_addr;
    common::com_memset(&read_addr, 0, sizeof(read_addr));
    // 使用IPv4协议
    read_addr.sin_family = AF_INET;
    // 网络通信都使用大端格式
    read_addr.sin_port = htons(d_ptr->param_.rcv_port);
    if (UdpParam::MODE_GROUP == d_ptr->param_.mode) {
      // 注意：Linux下，加入组播后，绑定地址只能绑定0.0.0.0地址否则会接收不到数据
      read_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    } else {
      // 让系统检测本地网卡，自动绑定本地IP
      read_addr.sin_addr.s_addr = INADDR_ANY;
    }

    // 允许重用本地地址
    int opt = 1;
    int ret = -1;
    int recv_buf_size=40*1024*1024;
    int recv_buf_result=0;
    int recv_buf_result_len=sizeof(recv_buf_result);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    ret =setsockopt(d_ptr->receiving_channel_info_.read_fd,
                       SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    ret = getsockopt(d_ptr->receiving_channel_info_.read_fd,
               SOL_SOCKET, SO_RCVBUF, (char*)&recv_buf_result, &recv_buf_result_len);
    if(-1 == ret) {
      LOG_INFO(3) << "failed to get read_fd recv buf size";
    }
    else {
//      LOG_INFO(3) << "default recv buf size =" << recv_buf_result;
    }

    ret = setsockopt(d_ptr->receiving_channel_info_.read_fd,
                     SOL_SOCKET, SO_RCVBUF, (const char*)&recv_buf_size, sizeof(recv_buf_size));

    ret = getsockopt(d_ptr->receiving_channel_info_.read_fd,
               SOL_SOCKET, SO_RCVBUF, (char*)&recv_buf_result, &recv_buf_result_len);
//    LOG_INFO(3) << "reset recv buf size =" << recv_buf_result;
#else
    ret = setsockopt(d_ptr->receiving_channel_info_.read_fd,
                       SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif
    if (ret < 0) {
      close(d_ptr->receiving_channel_info_.read_fd);
      d_ptr->receiving_channel_info_.read_fd = -1;
      LOG_ERR << "Failed to setsockopt(SOL_SOCKET, SO_REUSEADDR).";
      return false;
    }

//    Uint8_t mc_ttl = 0;
//    if (setsockopt(d_ptr->receiving_channel_info_.read_fd,
//                   IPPROTO_IP, IP_MULTICAST_TTL, &mc_ttl, sizeof(mc_ttl)) < 0) {
//      close(d_ptr->receiving_channel_info_.read_fd);
//      d_ptr->receiving_channel_info_.read_fd = -1;
//      LOG_ERR << "Failed to setsockopt(IPPROTO_IP, IP_MULTICAST_TTL).";
//      return false;
//    }

    if (bind(d_ptr->receiving_channel_info_.read_fd,
             (struct sockaddr*)&read_addr, sizeof(read_addr)) < 0) {
      LOG_ERR << "Failed to bind socket.";
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
      closesocket(d_ptr->receiving_channel_info_.read_fd);
      d_ptr->receiving_channel_info_.read_fd = INVALID_SOCKET;
#else
      close(d_ptr->receiving_channel_info_.read_fd);
      d_ptr->receiving_channel_info_.read_fd = -1;
#endif
      return false;
    }
  }

  if (d_ptr->param_.enable_send) {
    // 1 创建udp通信socket
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    // 指定为UDP套接字
    d_ptr->sending_channel_info_.send_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (INVALID_SOCKET == d_ptr->sending_channel_info_.send_fd) {
      if (INVALID_SOCKET != d_ptr->receiving_channel_info_.read_fd) {
        closesocket(d_ptr->receiving_channel_info_.read_fd);
        d_ptr->receiving_channel_info_.read_fd = INVALID_SOCKET;
      }
      int err = WSAGetLastError();
      printf("error! error code is %d/n", err);
      LOG_ERR << "Failed to create socket fd for sending channel, err=" << err;
      return false;
    }
#else
    d_ptr->sending_channel_info_.send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (d_ptr->sending_channel_info_.send_fd < 0) {
      if (d_ptr->receiving_channel_info_.read_fd >= 0) {
        close(d_ptr->receiving_channel_info_.read_fd);
        d_ptr->receiving_channel_info_.read_fd = -1;
      }
      LOG_ERR << "Failed to create socket fd for sending channel.";
      return false;
    }
//    Uint8_t mc_ttl = 0;
//    if (setsockopt(d_ptr->sending_channel_info_.send_fd,
//                   IPPROTO_IP, IP_MULTICAST_TTL, &mc_ttl, sizeof(mc_ttl)) < 0) {
//      close(d_ptr->sending_channel_info_.send_fd);
//      d_ptr->sending_channel_info_.send_fd = -1;
//      if (d_ptr->receiving_channel_info_.read_fd >= 0) {
//        close(d_ptr->receiving_channel_info_.read_fd);
//        d_ptr->receiving_channel_info_.read_fd = -1;
//      }
//      LOG_ERR << "Failed to setsockopt(IPPROTO_IP, IP_MULTICAST_TTL).";
//      return false;
//    }
#endif

    if (UdpParam::MODE_BROADCAST == d_ptr->param_.mode) {
      // 开启发送广播数据功能
      int opt = 1;
      //设置该套接字为广播类型，
      int ret = -1;
#ifdef PROJECT_PLATFORM_WINDOWS
      ret =setsockopt(d_ptr->sending_channel_info_.send_fd,
                      SOL_SOCKET, SO_BROADCAST, (const char*)&opt, sizeof(opt));
#else
      ret = setsockopt(d_ptr->sending_channel_info_.send_fd,
                       SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
#endif
      if (ret < 0) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        closesocket(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd = INVALID_SOCKET;
        if (INVALID_SOCKET != d_ptr->receiving_channel_info_.read_fd) {
          closesocket(d_ptr->receiving_channel_info_.read_fd);
          d_ptr->receiving_channel_info_.read_fd = INVALID_SOCKET;
        }
#else
        close(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd = -1;
        if (d_ptr->receiving_channel_info_.read_fd >= 0) {
          close(d_ptr->receiving_channel_info_.read_fd);
          d_ptr->receiving_channel_info_.read_fd = -1;
        }
#endif
        LOG_ERR << "Failed to setsockopt(SOL_SOCKET, SO_BROADCAST).";
        return false;
      }
    }

    // 绑定本地IP和端口地址
    if(d_ptr->param_.snd_local_port > 0) {
      // 绑定之后允许重用端口
      int opt=1;
      int ret=-1;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
      ret=setsockopt(d_ptr->sending_channel_info_.send_fd, SOL_SOCKET,
                      SO_REUSEADDR, (const char*)&opt, sizeof(opt));
#else
      ret=setsockopt(d_ptr->sending_channel_info_.send_fd,
                      SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif
      if(ret < 0) {
#if(PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        closesocket(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd=INVALID_SOCKET;
#else
        close(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd=-1;
#endif
        return false;
      }

      // 开始绑定本地端口
      struct sockaddr_in localAddr;
      common::com_memset(&localAddr, 0, sizeof(localAddr));
      localAddr.sin_family=AF_INET;
      localAddr.sin_port=htons(d_ptr->param_.snd_local_port);
      localAddr.sin_addr.s_addr=INADDR_ANY;

      if(bind(d_ptr->sending_channel_info_.send_fd, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0) {
        LOG_ERR << "Failed to bind send locket socket";
#if(PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        closesocket(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd=INVALID_SOCKET;
#else
        close(d_ptr->sending_channel_info_.send_fd);
        d_ptr->sending_channel_info_.send_fd=-1;
#endif
        return false;
      }
    }

    // 设置目的IP地址
    struct sockaddr_in send_addr;
    common::com_memset(&send_addr, 0, sizeof(send_addr));
    // 使用IPv4协议
    send_addr.sin_family = AF_INET;
    // 设置接收方端口号
    send_addr.sin_port = htons(d_ptr->param_.snd_remote_port);
    // 设置接收方IP
    if (UdpParam::MODE_BROADCAST == d_ptr->param_.mode) {
      send_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    } else {
      send_addr.sin_addr.s_addr = inet_addr(d_ptr->param_.snd_addr);
    }
    d_ptr->sending_channel_info_.send_addr = send_addr;
  }

//  std::cout<<" send_addr.sin_port: "<< htons(param_.snd_remote_port) <<std::endl;
  SocketCommInit((socket_comm_t*)d_ptr->communication_handle_,
                 TransmitPacket, &d_ptr->sending_channel_info_);





#if (DEBUG_RECEIVE)
    int N = 128;
    struct sockaddr_in groupcastaddr, addr;
    socklen_t addrlen = sizeof(groupcastaddr);
    char buf[N] = {};

    ssize_t bytes;
    while(1)
    {
        if((bytes = recvfrom(d_ptr->receiving_channel_info_.read_fd, buf, N, 0,\
                        (struct sockaddr *)&addr, &addrlen)) < 0)
        {
            LOG_ERR << "fail to recvfrom !!";
        }
        else
        {
            printf("ip: %s, port: %d\n",\
                    inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));

            printf("groupcast : %s\n", buf);
            PrintBuf("接收数据", __LINE__, (Uint8_t *)buf, bytes);
        }
    }

#endif
  return true;
}

/**
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UDPComm::Stop() {
  LOG_INFO(3) << "Stop UDP ... ";

  if (d_ptr->receiving_flag_) {
    LOG_INFO(3) << "Waiting to stop UDP receiving thread  ... ";
    d_ptr->receiving_flag_ = false;
    receiving_thread_.Join();
  }

  if (proc_flag_) {
    LOG_INFO(3) << "Waiting to stop proc thread ...";
    proc_flag_ = false;
//    proc_thread_.Join();
    cv_msg_buf_.notify_one();
    if(thread_proc_.joinable())
      thread_proc_.join();
  }

  if(m_PacketFlag)
  {
      LOG_INFO(3) << "Waiting to stop packet thread ...";
      m_PacketFlag = false;
      cv_mpu_Packet_.notify_one();
      if(m_DeliverPacketThread.joinable())
      {
          m_DeliverPacketThread.join();
      }
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if (INVALID_SOCKET != d_ptr->sending_channel_info_.send_fd) {
    shutdown(d_ptr->sending_channel_info_.send_fd, SD_BOTH);
    closesocket(d_ptr->sending_channel_info_.send_fd);
    d_ptr->sending_channel_info_.send_fd = INVALID_SOCKET;
  }
  if (INVALID_SOCKET != d_ptr->receiving_channel_info_.read_fd) {
    shutdown(d_ptr->receiving_channel_info_.read_fd, SD_BOTH);
    closesocket(d_ptr->receiving_channel_info_.read_fd);
    d_ptr->receiving_channel_info_.read_fd = INVALID_SOCKET;
  }
#else
  if (d_ptr->sending_channel_info_.send_fd >= 0) {
    shutdown(d_ptr->sending_channel_info_.send_fd, SHUT_RDWR);
    close(d_ptr->sending_channel_info_.send_fd);
    d_ptr->sending_channel_info_.send_fd = -1;
  }
  if (d_ptr->receiving_channel_info_.read_fd >= 0) {
    shutdown(d_ptr->receiving_channel_info_.read_fd, SHUT_RDWR);
    close(d_ptr->receiving_channel_info_.read_fd);
    d_ptr->receiving_channel_info_.read_fd = -1;
  }
#endif

  if (d_ptr->subscription_count_ > 0) {
    d_ptr->subscription_count_ = 0;
    for (Int32_t i = 0; i < UDPCommData::MAX_SUBSCRIPTION_NUM; ++i) {
      common::com_memset(d_ptr->subscription_list_[i].channel, 0,
                         sizeof(d_ptr->subscription_list_[i].channel));
      d_ptr->subscription_list_[i].callback = Nullptr_t;
      d_ptr->subscription_list_[i].user = Nullptr_t;
      d_ptr->subscription_list_[i].next = Nullptr_t;
    }
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  WSACleanup();
#endif

  MSG_BUF * p;
  for(int i = 0; i < v_msg_buf_.size(); i++) {
      p=v_msg_buf_.front();
      v_msg_buf_.pop();
      if(p)
      {
          delete p;
          p = nullptr;
      }
  }

  if(receiving_msg_buf_ != nullptr) {
    delete[] receiving_msg_buf_;
    receiving_msg_buf_=nullptr;
  }

  LOG_INFO(3) << "Stop UDP ... [OK]";

  return true;
}

/**
 * @brief 预订消息
 * @param[in] sub      需要预订的消息
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UDPComm::Subscribe(
    const Char_t* channel,
    void (*handlerMethod)(const Char_t *channel,Uint64_t timestamp,
                          const void *buf, Int32_t buf_len,
                          bool has_time, void *user),
    void* user) {
  if (d_ptr->subscription_count_ >= UDPCommData::MAX_SUBSCRIPTION_NUM) {
    LOG_ERR << "Can't add subscriber any more.";
    return false;
  }

  Int32_t channel_name_len = strlen(channel);
  if (channel_name_len > SOCKET_COMM_MAX_CHANNEL_LENGTH) {
    LOG_ERR << "The length of this channel name is too long.";
    return false;
  }

  SocketSubscription& sub = d_ptr->subscription_list_[d_ptr->subscription_count_];

  common::com_memcpy(sub.channel, channel, channel_name_len);
  sub.channel[channel_name_len] = 0;
  sub.callback = handlerMethod;
  sub.user = user;
  sub.next = Nullptr_t;
  SocketCommSubscribe((socket_comm_t*)d_ptr->communication_handle_, &sub);

  d_ptr->subscription_count_++;

//  LOG_INFO(3) << "subscribe \""<< channel <<"\" , count=" << subscription_count_;

  return true;
}

/**
 * @brief 开始接收消息
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UDPComm::StartReceiving() {
  if (d_ptr->receiving_flag_) {
    LOG_ERR << "receiving thread already start.";
    return false;
  }

  if(proc_flag_) {
    LOG_ERR << "proc thread already start";
    return false;
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((!d_ptr->param_.enable_recv) ||
      (INVALID_SOCKET == d_ptr->receiving_channel_info_.read_fd)) {
    LOG_ERR << "Invalid receiving channel.";
    return false;
  }
#else
  if ((!d_ptr->param_.enable_recv) || (d_ptr->receiving_channel_info_.read_fd < 0)) {
    LOG_ERR << "Invalid receiving channel.";
    return false;
  }
#endif

  d_ptr->receiving_flag_ = true;
  receiving_thread_func_helper_.SetThreadFunc(
        this, &UDPComm::TheadReceivingMessages);
  common::os::Thread::Parameter thread_param_receiving;
  bool ret = receiving_thread_.Create(thread_param_receiving,
                                      &receiving_thread_func_helper_);
  if (!ret) {
    d_ptr->receiving_flag_ = false;
    LOG_ERR << "Failed to create UDP receiving thread.";
    return false;
  }

  proc_flag_ = true;

#if ENABLE_THREADPROCMESSAGE
  proc_thread_func_helper_.SetThreadFunc(
        this, &UDPComm::ThreadProcMessage);
  common::os::Thread::Parameter thread_param_proc;
  ret = proc_thread_.Create(thread_param_proc,
                                  &proc_thread_func_helper_);
  if (!ret) {
    proc_flag_ = false;
    LOG_ERR << "Failed to create proc thread.";
    return false;
  }
#endif

  thread_proc_ = std::thread(std::bind(&UDPComm::ThreadProcMessage, this));

  m_PacketFlag = true;
  m_DeliverPacketThread = std::thread(std::bind(&UDPComm::ThreadDeliverPacket, this));
  return true;
}


UdpComNode::UdpComNode()
    :UDPComm(*new UdpNodePrivate)
{
  init();
}

UdpComNode::UdpComNode(UdpNodePrivate &dd)
    :UDPComm(dd)
{
  init();
}

void UdpComNode::init()
{

}

/**
 * @brief 发送消息
 * @param[in] channel      通道名
 * @param[in] buf          消息内容
 * @param[in] buf_len      消息内容的字节数
 * @return 0 - 成功；其他 - 失败。
 * @note 通道名的字符个数（不含结尾符'\0'）不能超过4，超过的部分将会被舍弃。
 * @date       2020.09.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool UdpComNode::Publish(const Char_t *channel, const void *buf,
                         Int32_t buf_len) {
  // Lock sending channel
  common::os::LockHelper lock(lock_sending_channel_);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if (d_ptr->param_.enable_send && (INVALID_SOCKET != d_ptr->sending_channel_info_.send_fd)) {
    return gSocketCommPublish((socket_comm_t*)d_ptr->communication_handle_,
                             channel, buf, buf_len);
  }
#else
  if (d_ptr->param_.enable_send && (d_ptr->sending_channel_info_.send_fd >= 0)) {
    return gSocketCommPublish((socket_comm_t*)d_ptr->communication_handle_,
                             channel, buf, buf_len);
  }
#endif

  return false;
}


bool UdpComNode::Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len)
{
    return false;
}

void UDPComm::TheadReceivingMessages() {
//  LOG_INFO(3) << "UDP Receiving Thread ... [Started]";

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if ((!d_ptr->param_.enable_recv) ||
      (INVALID_SOCKET == d_ptr->receiving_channel_info_.read_fd)) {
    LOG_ERR << "Invalid receiving channel.";
    return;
  }
#else
  if ((!d_ptr->param_.enable_recv) || (d_ptr->receiving_channel_info_.read_fd < 0)) {
    LOG_ERR << "Invalid receiving channel.";
    return;
  }
#endif

  struct sockaddr_in from_addr; // only IPv4 compatible
  unsigned int from_addr_sz = sizeof(from_addr);
  while (this->d_ptr->receiving_flag_) {
    fd_set read_fds;  //读文件操作符
    FD_ZERO(&read_fds);
    // 每次调用select之前都要重新在read_fds中设置文件描述符，
    // 因为事件发生以后，文件描述符集合将被内核修改
    FD_SET(d_ptr->receiving_channel_info_.read_fd, &read_fds);
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
    timeout.tv_usec = 1;
    Int32_t ret = select(d_ptr->receiving_channel_info_.read_fd+1,
                          &read_fds, Nullptr_t, Nullptr_t, &timeout);
    if (ret >= 0) {
      if (FD_ISSET(d_ptr->receiving_channel_info_.read_fd, &read_fds)) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
        Int32_t msg_len =
            recvfrom(d_ptr->receiving_channel_info_.read_fd,
                      receiving_msg_buf_, RECEIVING_UDP_MSG_BUF_SIZE, 0,
                      (struct sockaddr*)&from_addr, (int*)&from_addr_sz);
#else
        ssize_t msg_len =
            recvfrom(d_ptr->receiving_channel_info_.read_fd,
                      receiving_msg_buf_, RECEIVING_UDP_MSG_BUF_SIZE, 0,
                      (struct sockaddr*)&from_addr, &from_addr_sz);
#endif

//        if(ntohs(from_addr.sin_port) == 25001)
//        {
//           std::cout << "Receiving " << msg_len << " bytes, from \""
//                     << inet_ntoa(from_addr.sin_addr) <<" from_addr.sin_port: "<< ntohs(from_addr.sin_port)  << "\"" << std::endl;
//            std::cout<<"receiving_msg_buf_: "<<receiving_msg_buf_<<std::endl;


//            PrintBuf(__func__, __LINE__, receiving_msg_buf_, msg_len);
//        }

        /* For test only (begin) */
//        struct timeval tv;
//        gettimeofday(&tv, NULL);
//        Float64_t now = (static_cast<Int64_t>(tv.tv_sec) * 1000000 + tv.tv_usec)*0.000001;
//        time_t timestamp = static_cast<time_t>(now);
//        struct ::tm tm_time;
//        localtime_r(&timestamp, &tm_time);
//        Int32_t usecs = static_cast<Int32_t>((now - timestamp) * 1000000);
//        Char_t receiving_msg_head_buf_[1024] = { 0 };
//        com_snprintf(receiving_msg_head_buf_,
//                     sizeof(receiving_msg_head_buf_)-1,
//                     "%s  %02d-%02d %02d:%02d:%02d.%06d :\n",
//                     inet_ntoa(from_addr.sin_addr),
//                     1+tm_time.tm_mon,
//                     tm_time.tm_mday,
//                     tm_time.tm_hour,
//                     tm_time.tm_min,
//                     tm_time.tm_sec,
//                     usecs);
//        std::cout << "Head : " << receiving_msg_head_buf_ << std::endl;
        /* For test only (end) */

//        ret = SocketCommReceivePacket(
//              communication_handle_,
//              receiving_msg_buf_, msg_len,
//              from_addr.sin_addr.s_addr | ((Uint64_t)from_addr.sin_port << 32));
//        if (ret < 0) {
//          LOG_WARN << "Failed to receive packet. ERR=" << ret;
////          usleep(1);
//        }

        if(msg_len <= RECEIVING_UDP_MSG_BUF_SIZE) {
          // 将接收数据放入接收缓冲队列
          MSG_BUF *msg_buf =
              new MSG_BUF(
                (const Uint8_t *)receiving_msg_buf_, msg_len,
                  from_addr.sin_addr.s_addr | ((Uint64_t)from_addr.sin_port << 32)
              );
#ifdef ROS
          msg_buf->time_stamp = ros::Time::now().toNSec();
#else
          msg_buf->time_stamp = common::GetClockNowUs();
#endif

          {
            std::unique_lock<std::mutex> lock(mtx_msg_buf_);
            v_msg_buf_.push(msg_buf);
            if(v_msg_buf_.size() > 100) {
              LOG_WARN << "cache too more packet (" << v_msg_buf_.size() << ")";
            }
          }
          cv_msg_buf_.notify_one();
        }
        else {
          LOG_INFO(3) << "udp recv " << msg_len << " > " << RECEIVING_UDP_MSG_BUF_SIZE << ", invalid data, discard";
        }
      }
    }
  }

  LOG_INFO(3) << "UDP Receiving Thread ... [Stopped]";
}

void UDPComm::ThreadProcMessage()
{
//  LOG_INFO(3) << "ThreadProcMessage start ...";
  MSG_BUF *buf;
  while(this->proc_flag_)
  {
    {
      std::unique_lock<std::mutex> lock(mtx_msg_buf_);
      if(v_msg_buf_.size() == 0) {
        cv_msg_buf_.wait(lock);
        if(!this->proc_flag_) {
          lock.unlock();
          return;
        }
      }

      if(v_msg_buf_.size() > 0) {
          buf=v_msg_buf_.front();
          v_msg_buf_.pop();
      }
    }

    if(buf == nullptr)
    {
        continue;
    }
    Int32_t ret=SocketCommReceivePacket(
          d_ptr->communication_handle_,
          buf->data, buf->len,
          buf->from_addr,
          buf->time_stamp
          );
    if (ret < 0) {
//      LOG_WARN << "Failed to receive packet. ERR=" << ret;
//          usleep(1);
    }

    if(buf) {
      delete buf;
      buf=Nullptr_t;
    }
  }
  LOG_INFO(3) << "ThreadProcMessage stopped";
}


static void FillRequestSequence(Uint8_t* buf, Uint64_t StartPos, uint16_t idx)
{
    for(int i = 0; i < 8; i++)
    {
       buf[idx+i] =  (StartPos >> (8 * i)) & 0xFF;
    }
}



#if 0
Int32_t UDPComm::SocketCommReceivePacketByMessageCommon(
    socket_comm_t *handle,
    Uint8_t *buf, Uint32_t buf_len,
    UDPP1_t *udpp_1,
    MSG_BUF_META *msg_buf_meta, Uint64_t timestamp)
{
    if (msg_buf_meta == nullptr)
    {
      return 0;
    }

    std::string channel_name = get_id_channel_name(udpp_1->message_id);

    LOG_INFO(3) << "channel name:" << channel_name.c_str();
    Uint32_t buf_pos=0;
    Uint8_t fragment_type = 0;
    Int32_t ret=0;

    fragment_type= buf[0] & 0x0f;


    UDPP2_EXT_t udpp_2_ext;

    DecodeProtocol_2_ext(buf + buf_pos, buf_len, &udpp_2_ext);

    // 检查校验码
    if(KTCloudGetChecKS(buf, udpp_2_ext.payload_len+19) != 0) { // 校验码不正确
        LOG_INFO(3) << channel_name.c_str() << " mid= 0x" << get_dec2hex_str(udpp_1->message_id).c_str() << ", seq=" << udpp_2_ext.all_fragment_sequence << ", chksum mismatch, dropped";
        return -4;
    }

    if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_START || udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence+1)
    {
        msg_buf_meta->request_resend = false;
        if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_END)
        {
            msg_buf_meta->recv_completed = true;
        }

        if(udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence || udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence+1)
        {
            msg_buf_meta->request_sequence = udpp_2_ext.all_fragment_sequence;
        }
        else
        {
            msg_buf_meta->request_resend = true;
        }
    }
    else
    {
        if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_END && udpp_2_ext.all_fragment_sequence == 0)
        {
            msg_buf_meta->request_resend = false;
            msg_buf_meta->recv_completed = true;
        }
        else
        {
            msg_buf_meta->request_resend = true;
        }
    }

    if(udpp_2_ext.all_fragment_sequence == 0 && msg_buf_meta->m_FirstFlage)
    {
        //mpu断开重启,对参数初始化
        msg_buf_meta->request_sequence = 0;
        msg_buf_meta->request_resend = true;
        msg_buf_meta->recv_completed = false;
        {
            std::unique_lock<std::mutex> lock(mtx_msg_send);
            VecSt_DeliverPacketPtr.clear();
        }
        msg_buf_meta->recv_completed=false;
        ret=msg_buf_meta->cache_size;
        msg_buf_meta->cache_size=0;
    }

    if(udpp_2_ext.all_fragment_sequence > 0)
    {
        msg_buf_meta->m_FirstFlage = true;
    }


    int pos = 0;
    Uint64_t err = get_request_err_type_by_message_id_mpu(udpp_1->message_id, pos);
    if(err != 0 && pos != 0)
    {
        errSum |= err;
        uint16_t idx =  (8 * pos);
        FillRequestSequence(g_MpuBuf, msg_buf_meta->request_sequence+1, idx);

//        if(channel_name == "mpu/FitSamplePoint")
//        {
//            std::cout<<common::GetClockNowMs()<<" channel_name:"<<channel_name<<" udpp_2_ext.fragment_type:"<<(Uint16_t)udpp_2_ext.fragment_type<<" udpp_2_ext.all_fragment_sequence:"<<
//                    udpp_2_ext.all_fragment_sequence<<" msg_buf_meta->request_sequence:"<<msg_buf_meta->request_sequence<<std::endl;
//            if(msg_buf_meta->request_sequence == msg_buf_meta->request_sequence_save)
//            {
//                msg_buf_meta->sum++;
//            }
//            msg_buf_meta->request_sequence_save = msg_buf_meta->request_sequence;
//            if(msg_buf_meta->sum > 10)
//            {
//                if(msg_buf_meta->sum > 13)
//                {
//                    msg_buf_meta->sum = 0;
//                }
//                PrintBuf(__func__, __LINE__, g_MpuBuf, 1004);
//            }
//        }
//        if(common::GetClockNowMs() - msg_buf_meta->time >= timeTem)
//        {
              Uint32_t resend_msg_id=0x8005201;

//            if(common::GetClockNowMs() - msg_buf_meta->time > 10)
//            {
                FillRequestSequence(g_MpuBuf, errSum, 0);
                Publish(resend_msg_id, g_MpuBuf, 1004);
//                msg_buf_meta->time = common::GetClockNowMs();
//            }
//        }
    }


    if(!msg_buf_meta->request_resend) {
        Uint32_t cur_data_len = udpp_2_ext.payload_len;
        buf_pos += 18;
        memcpy(msg_buf_meta->cache + msg_buf_meta->cache_size, buf + buf_pos, cur_data_len);
        msg_buf_meta->cache_size += cur_data_len;
    }

    if(msg_buf_meta->recv_completed) {
        cv_mpu_Packet_.notify_one();
        std::shared_ptr<St_DeliverPacket> St_DeliverPacketPtr = std::make_shared<St_DeliverPacket>();
        St_DeliverPacketPtr->handle = handle;
        St_DeliverPacketPtr->channel_name = channel_name;
        St_DeliverPacketPtr->cache = msg_buf_meta->cache;
        St_DeliverPacketPtr->cache_size = msg_buf_meta->cache_size;
        St_DeliverPacketPtr->timestamp = timestamp;
        {
            std::unique_lock<std::mutex> lock(mtx_msg_send);
            VecSt_DeliverPacketPtr.emplace_back(St_DeliverPacketPtr);
        }

        msg_buf_meta->recv_completed=false;
        ret=msg_buf_meta->cache_size;
        msg_buf_meta->cache_size=0;
        return ret;
    }

    return 0;
}

#else

Int32_t UDPComm::SocketCommReceivePacketByMessageCommon(
    socket_comm_t *handle,
    Uint8_t *buf, Uint32_t buf_len,
    UDPP1_t *udpp_1,
    MSG_BUF_META *msg_buf_meta, Uint64_t timestamp)
{
    if (msg_buf_meta == nullptr)
    {
      return 0;
    }

    std::string channel_name = get_id_channel_name(udpp_1->message_id);
    // LOG_INFO(3) << "channel name:" << channel_name.c_str();

    Uint32_t buf_pos=0;
    Uint8_t fragment_type = 0;
    Int32_t ret=0;

    fragment_type= buf[0] & 0x0f;


    UDPP2_EXT_t udpp_2_ext;

    if(channel_name == "mpu/MpuState")
    {
        std::cout<<"udpp_1->message_id:"<<udpp_1->message_id<<std::endl;
        PrintBuf(__func__, __LINE__, buf, (Int32_t)buf_len);
    }
    DecodeProtocol_2_ext(buf + buf_pos, buf_len, &udpp_2_ext);

    // 检查校验码
    if(KTCloudGetChecKS(buf, udpp_2_ext.payload_len+19) != 0) { // 校验码不正确
        LOG_INFO(3) << channel_name.c_str() << " mid= 0x" << get_dec2hex_str(udpp_1->message_id).c_str() << ", seq=" << udpp_2_ext.all_fragment_sequence << ", chksum mismatch, dropped";
        return -4;
    }

    if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_START || udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence+1)
    {
        msg_buf_meta->request_resend = false;
        if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_END)
        {
            msg_buf_meta->recv_completed = true;
        }
        if(udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence || udpp_2_ext.all_fragment_sequence == msg_buf_meta->request_sequence+1)
        {
            msg_buf_meta->request_sequence = udpp_2_ext.all_fragment_sequence;
        }
        else
        {
            msg_buf_meta->request_resend = true;
        }
    }
    else
    {
        msg_buf_meta->request_sequence = udpp_2_ext.all_fragment_sequence;
        if(udpp_2_ext.fragment_type == FRAGMENT_TYPE_END && udpp_2_ext.all_fragment_sequence == 0)
        {
            msg_buf_meta->request_resend = false;
            msg_buf_meta->recv_completed = true;
        }
        else
        {
            msg_buf_meta->request_resend = true;
        }
    }

    if(udpp_2_ext.all_fragment_sequence == 0 && msg_buf_meta->m_FirstFlage)
    {
        //mpu断开重启,对参数初始化
        msg_buf_meta->request_sequence = 0;
        msg_buf_meta->request_resend = true;
        msg_buf_meta->recv_completed = false;
        {
            std::unique_lock<std::mutex> lock(mtx_msg_send);
            VecSt_DeliverPacketPtr.clear();
        }
        msg_buf_meta->recv_completed=false;
        ret=msg_buf_meta->cache_size;
        msg_buf_meta->cache_size=0;
    }

    if(udpp_2_ext.all_fragment_sequence > 0)
    {
        msg_buf_meta->m_FirstFlage = true;
    }


//    int pos = 0;
//    Uint64_t err = get_request_err_type_by_message_id_mpu(udpp_1->message_id, pos);
//    if(err != 0 && pos != 0)
//    {
//        errSum |= err;
//        uint16_t idx =  (8 * pos);
//        FillRequestSequence(g_MpuBuf, msg_buf_meta->request_sequence+1, idx);
//        Uint32_t resend_msg_id=0x8005201;
//        FillRequestSequence(g_MpuBuf, errSum, 0);
//        Publish(resend_msg_id, g_MpuBuf, 1004);
//    }


    if(!msg_buf_meta->request_resend) {
        Uint32_t cur_data_len = udpp_2_ext.payload_len;
        buf_pos += 18;
        memcpy(msg_buf_meta->cache + msg_buf_meta->cache_size, buf + buf_pos, cur_data_len);
        msg_buf_meta->cache_size += cur_data_len;
    }

    if(msg_buf_meta->recv_completed) {
        cv_mpu_Packet_.notify_one();
        std::shared_ptr<St_DeliverPacket> St_DeliverPacketPtr = std::make_shared<St_DeliverPacket>();
        St_DeliverPacketPtr->handle = handle;
        St_DeliverPacketPtr->channel_name = channel_name;
        St_DeliverPacketPtr->cache = msg_buf_meta->cache;
        St_DeliverPacketPtr->cache_size = msg_buf_meta->cache_size;
        St_DeliverPacketPtr->timestamp = timestamp;
        {
            std::unique_lock<std::mutex> lock(mtx_msg_send);
            VecSt_DeliverPacketPtr.emplace_back(St_DeliverPacketPtr);
        }
        // LOG_INFO(3) << "msg_buf_meta->recv_completed:" << msg_buf_meta->recv_completed;
        msg_buf_meta->recv_completed=false;
        ret=msg_buf_meta->cache_size;
        msg_buf_meta->cache_size=0;
        return ret;
    }

    return 0;
}

#endif


void UDPComm::ThreadDeliverPacket()
{
    std::unique_lock<std::mutex> lck(mtx_mpu_Packet_);
    while(m_PacketFlag)
    {
        cv_mpu_Packet_.wait(lck);
        {
            std::unique_lock<std::mutex> lock(mtx_msg_send);
            for(std::vector<std::shared_ptr<St_DeliverPacket>>::iterator iter = VecSt_DeliverPacketPtr.begin(); iter != VecSt_DeliverPacketPtr.end();)
            {
                std::shared_ptr<St_DeliverPacket> St_DeliverPacketPtrTem = *iter;
                DeliverPacket(St_DeliverPacketPtrTem->handle, St_DeliverPacketPtrTem->channel_name.c_str(), St_DeliverPacketPtrTem->timestamp, St_DeliverPacketPtrTem->cache, St_DeliverPacketPtrTem->cache_size);
                VecSt_DeliverPacketPtr.erase(iter);
            }
        }
    }
}



Int32_t UdpComNode::SocketCommReceivePacket(void *handle, const void *buf_in, Int32_t buf_len, uint64_t from_addr, Uint64_t timestamp)
{
    int ret = gSocketCommReceivePacket(
          (socket_comm_t*)d_ptr->communication_handle_,
          buf_in, buf_len, from_addr, timestamp);
    return ret;
}

Int32_t UdpIDNode::SocketCommReceivePacket(void *handle, const void *buf_in, Int32_t buf_len, uint64_t from_addr, Uint64_t timestamp)
{
    int ret = SocketCommReceivePacketByMessageID(
          (socket_comm_t*)d_ptr->communication_handle_,
          buf_in, buf_len, from_addr, timestamp);
    return ret;
}

Int32_t UDPNoProtocolComm::SocketCommReceivePacket(void *handle, const void *buf_in, Int32_t buf_len, uint64_t from_addr, Uint64_t timestamp)
{
  DeliverPacket((socket_comm_t*)handle, ChannelName.c_str(), timestamp, buf_in, buf_len);
  return 0;
}

bool UDPNoProtocolComm::Publish(const Char_t *channel, const void *buf, Int32_t buf_len)
{
  socket_comm_t *handle = (socket_comm_t *)d_ptr->communication_handle_;

  common::com_memcpy(&handle->publish_buffer, buf, buf_len);

  handle->transmit_packet(handle->publish_buffer, buf_len,
                          handle->transmit_user);

  return true;
}

UdpIDNode::UdpIDNode()
{
  map_first_frame=true;
  perception_first_frame=true;
  system_first_frame=true;
  spdpln_first_frame=true;
  pathpln_first_frame=true;

#if 0
    send_thread_func_helper_.SetThreadFunc(
          this, &UdpIDNode::ThreadCycleReceiveMessage);
    common::os::Thread::Parameter thread_param;
    bool ret = send_thread_.Create(thread_param,
                                        &send_thread_func_helper_);
    if (!ret) {
      receiving_flag_ = false;
      LOG_ERR << "Failed to create UDP UDPIDNode send thread.";
    }
    send_flag_ = true;
#endif
}

bool UdpIDNode::Publish(const Char_t *channel, const void *buf, Int32_t buf_len)
{
    // not implement
    return false;
}

bool UdpIDNode::Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len)
{
    // Lock sending channel
    common::os::LockHelper lock(lock_sending_channel_);

  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    if (d_ptr->param_.enable_send && (INVALID_SOCKET != d_ptr->sending_channel_info_.send_fd)) {
      return gSocketCommPublishByID((socket_comm_t*)d_ptr->communication_handle_,
                               message_id, buf, buf_len);
    }
  #else
    if (d_ptr->param_.enable_send && (d_ptr->sending_channel_info_.send_fd >= 0)) {
      return gSocketCommPublishByID((socket_comm_t*)d_ptr->communication_handle_,
                               message_id, buf, buf_len);
    }
  #endif

    return false;
}


UdpMPUNode::UdpMPUNode()
{
  map_first_frame=true;
  perception_first_frame=true;
  system_first_frame=true;
  spdpln_first_frame=true;
  pathpln_first_frame=true;

#if 0
    send_thread_func_helper_.SetThreadFunc(
          this, &UdpIDNode::ThreadCycleReceiveMessage);
    common::os::Thread::Parameter thread_param;
    bool ret = send_thread_.Create(thread_param,
                                        &send_thread_func_helper_);
    if (!ret) {
      receiving_flag_ = false;
      LOG_ERR << "Failed to create UDP UDPIDNode send thread.";
    }
    send_flag_ = true;
#endif
}

bool UdpMPUNode::Publish(const Char_t *channel, const void *buf, Int32_t buf_len)
{
    // not implement
    return false;
}

bool UdpMPUNode::Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len)
{
    // Lock sending channel
    common::os::LockHelper lock(lock_sending_channel_);

  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
    if (d_ptr->param_.enable_send && (INVALID_SOCKET != d_ptr->sending_channel_info_.send_fd)) {
      return gSocketCommPublishByID((socket_comm_t*)d_ptr->communication_handle_,
                               message_id, buf, buf_len);
    }
  #else
    if (d_ptr->param_.enable_send && (d_ptr->sending_channel_info_.send_fd >= 0)) {
      return gSocketCommPublishByID((socket_comm_t*)d_ptr->communication_handle_,
                               message_id, buf, buf_len);
    }
  #endif

    return false;
}


Int32_t UdpMPUNode::SocketCommReceivePacket(void *handle, const void *buf_in, Int32_t buf_len, uint64_t from_addr, Uint64_t timestamp)
{
    int ret = SocketCommReceivePacketByMessageMPU(
          (socket_comm_t*)d_ptr->communication_handle_,
          buf_in, buf_len, from_addr, timestamp);
    return ret;
}


Int32_t UdpMPUNode::SocketCommReceivePacketByMessageMPU(socket_comm_t *handle,
                                                       const void *buf_in,
                                                       Int32_t buf_len,
                                                       uint64_t from_addr, Uint64_t timestamp) {
    if(handle == nullptr || buf_in == nullptr)
    {
        return -1;
    }
    Uint8_t *buf = (Uint8_t*)buf_in;
    Int32_t buf_pos = 0;

    if(buf_len < 22) { // 协议1+协议2 头部字节
        return  -1;
    }


    memset(&udpp_1, 0, sizeof(UDPP1_t));
    DecodeProtocol_1(buf, &udpp_1);  // 解析协议1头

    if(udpp_1.payload_len <= 22 || udpp_1.payload_len > 1024)
        return -2;

//    LOG_INFO(3) << "protocol_1 message id,sequence=" << udpp_1.message_id <<',' << udpp_1.sequence_number;

    buf_pos += 20;

  if(udpp_1.message_id == 0x8005020 || udpp_1.message_id == 0x8005021 || udpp_1.message_id == 0x8005022 || udpp_1.message_id == 0x8005023 ||
          udpp_1.message_id == 0x8005024 || udpp_1.message_id == 0x8005025 || udpp_1.message_id == 0x8005026 || udpp_1.message_id == 0x8005027 ||
          udpp_1.message_id == 0x8005028 || udpp_1.message_id == 0x8005029 || udpp_1.message_id == 0x800502A || udpp_1.message_id == 0x800502B) // FitSamplePoint
  {
      msg_buf_meta = &msg_meta_FitSamplePoint;
  }
  else if(udpp_1.message_id == 0x8005000) // MPUState
  {
      msg_buf_meta = &msg_meta_MpuState;
  }
  else if(udpp_1.message_id == 0x8005001) // VehicleCorrectState
  {
      msg_buf_meta = &msg_meta_VehicleCorrectState;
  }
  else if(udpp_1.message_id == 0x8005002) // VehicleState
  {
      msg_buf_meta = &msg_meta_VehicleState;
  }
  else if(udpp_1.message_id == 0x8005003) { // mpu CorrectPosition数据
      msg_buf_meta = &msg_meta_CorrectPosition;
  }
  else if(udpp_1.message_id == 0x8005004 || udpp_1.message_id == 0x8005005 || udpp_1.message_id == 0x8005006 || udpp_1.message_id == 0x8005007)  // PartialMap
  {
      msg_buf_meta = &msg_meta_PartialMap;
  }
  else if(udpp_1.message_id == 0x8005008 || udpp_1.message_id == 0x8005009 || udpp_1.message_id == 0x800500A || udpp_1.message_id == 0x800500B) // PartialMapLineFit
  {
      msg_buf_meta = &msg_meta_PartialMapLineFit;
  }
  else if(udpp_1.message_id == 0x800500C || udpp_1.message_id == 0x800500D || udpp_1.message_id == 0x800500E || udpp_1.message_id == 0x800500F) // OrigCamera
  {
      msg_buf_meta = &msg_meta_OrigCamera;
  }
  else if(udpp_1.message_id == 0x800501C || udpp_1.message_id == 0x800501D || udpp_1.message_id == 0x800501E || udpp_1.message_id == 0x800501F)
  {
      msg_buf_meta = &msg_meta_CameraFitLine;
  }
  else if(udpp_1.message_id ==0x8005010 || udpp_1.message_id == 0x8005011 || udpp_1.message_id == 0x8005012 || udpp_1.message_id == 0x8005013 ||
          udpp_1.message_id == 0x8005014 || udpp_1.message_id == 0x8005015 || udpp_1.message_id == 0x8005016 || udpp_1.message_id == 0x8005017 ||
          udpp_1.message_id == 0x8005018 || udpp_1.message_id == 0x8005019) //MapLocationEhr
  {
    msg_buf_meta = &msg_meta_MapLocationEhr;
  }
  else if(udpp_1.message_id == 0x800501a) // LocationPosition
  {
      msg_buf_meta = &msg_meta_LocationPosition;
  }
  else if(udpp_1.message_id == 0x800501b) // DecisionPosition
  {
      msg_buf_meta = &msg_meta_DecisionPosition;
  }
  else
  {
      return 0;
  }

  return SocketCommReceivePacketByMessageCommon(handle, buf+buf_pos, buf_len-buf_pos, &udpp_1, msg_buf_meta, timestamp);

}


}  // mpu
}  // framework
}  // phoenix

