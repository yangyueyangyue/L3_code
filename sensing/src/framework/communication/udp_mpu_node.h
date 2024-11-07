/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       socket_communication.h
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

#ifndef PHOENIX_FRAMEWORK_MPU_UDP_NODE_H_
#define PHOENIX_FRAMEWORK_MPU_UDP_NODE_H_

#include <stdint.h>
#include <sys/types.h>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#include "utils/com_utils.h"
#include "utils/macros.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif

#include "os/mutex.h"
#include "os/thread.h"

namespace phoenix {
namespace framework {
namespace mpu{



#define DEBUG_RECEIVE        (0)
#define ENABLE_THREADPROCMESSAGE  (0)



// 通道名称的最大字符数目（不含结束符号'\0'）
#define SOCKET_COMM_MAX_CHANNEL_LENGTH 64

// Total memory allocated is roughly:
//
// NUM_BUFFERS*(MAX_PACKET_SIZE + MAX_FRAGMENTS + CHANNEL_LENGTH) +
//   PUBLISH_BUFFER_SIZE
//

// 接收端单个时刻的最大消息的个数
#define SOCKET_COMM_3_NUM_BUFFERS 4
// 单个消息发送的最大数据量
#define SOCKET_COMM_3_MAX_PACKET_SIZE (1000000)
// 单个消息能够分配的最大包数目
#define SOCKET_COMM_3_MAX_FRAGMENTS 800

// 发送消息时单个包的最大字节数
// Socket communication will allocate a single buffer of the size below for
// publishing messages. The SOCKET_COMM_3 fragmentation option will be used to
// send messages larger than this.
// 1472[UDP DATA] = 1500 - 20[IP] - 8[UDP]
#define SOCKET_COMM_PUBLISH_BUFFER_SIZE 1472

// header can contain a channel plus our usual header.
// 消息中的包头的最大字节数
// 20为分包时的一些包信息，
// “SOCKET_COMM_MAX_CHANNEL_LENGTH + 1”为消息通道名称占的最大字节数
#define MAXIMUM_HEADER_LENGTH (20 + SOCKET_COMM_MAX_CHANNEL_LENGTH + 1)

#define MAGIC_SOCKET_COMM_2 0x4c433032
#define MAGIC_SOCKET_COMM_3 0x4c433033

#define MODULE_SYSTEM_DISPLAY_DATA_SIZE  1350



typedef struct SocketSubscription socket_subscription_t;  /// 消息预订结构体
typedef struct SocketCommunication socket_comm_t;  /// 套接字通信收发时需要的结构体


/**
 * @struct FragmentBuffer
 * @brief 发送单个包时的缓存结构体。
 */
struct FragmentBuffer {
  /// 当前包在对应消息中的包的数目
  Int32_t last_fragment_count;

  /// 服务器地址
  Uint64_t from_addr;
  /// 消息序号
  Uint32_t msg_seq;

  /// 消息的通道名称
  Char_t channel[SOCKET_COMM_MAX_CHANNEL_LENGTH + 1];
  /// 该消息剩下的包的数目
  Int32_t fragments_remaining;
  /// 整个消息的数据缓存
  Char_t buf[SOCKET_COMM_3_MAX_PACKET_SIZE];
  /// 所有包是否已经接收的标记
  Uint8_t frag_received[SOCKET_COMM_3_MAX_FRAGMENTS];
};


/**
 * @struct SocketCommunication
 * @brief 套接字通信结构体
 */
struct SocketCommunication {
  /** Buffers for reassembling multi-fragment messages. **/
  struct FragmentBuffer fragment_buffers[SOCKET_COMM_3_NUM_BUFFERS];

  /** every time we receive a fragment, we increment this counter
        and write the value to the corresponding fragment buffer. This
        allows us to measure how "stale" a fragment is (how long it's
        been since we received a fragment for it).
    **/
  Int32_t last_fragment_count;

  /// 数据发送对应的回调函数
  void (*transmit_packet)(const void *buf_in, Int32_t buf_len, void *user);
  /// 发送地址和发送通道的句柄
  void *transmit_user;

  /// 单个包中数据发送的缓存区
  Uint8_t publish_buffer[SOCKET_COMM_PUBLISH_BUFFER_SIZE];
  /// 消息序号
  Uint32_t msg_seq;

  /// 首个消息预订的指针
  socket_subscription_t *first_subscription;
};


/**
 * @struct SocketReceivingChannelInfo
 * @brief 接收通道信息
 */
struct SocketReceivingChannelInfo {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  SOCKET read_fd;
#else
  Int32_t read_fd;
#endif
};

/**
 * @struct SocketSendingChannelInfo
 * @brief 发送通道信息
 */
struct SocketSendingChannelInfo {
  /// 发送地址
  sockaddr_in send_addr;
  /// 发送描述符
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  SOCKET send_fd;
#else
  Int32_t send_fd;
#endif
};


/**
 * @struct SocketSubscription
 * @brief 消息预订结构体。
 */
struct SocketSubscription {
  /// 消息的通道名
  Char_t channel[SOCKET_COMM_MAX_CHANNEL_LENGTH+1];
  /// 消息接收完毕后的回调函数
  void (*callback)(const Char_t *channel,Uint64_t timestamp,
                   const void *buf, Int32_t buf_len, bool has_time, void *user);
  /// 用户自定义数据
  void *user;
  /// 'next' field is for socket_comm_t internal use.
  /// 下一个预订消息的指针
  socket_subscription_t *next;
};


struct ShareSendDataMpu {
    Uint32_t message_id = 0;
    Uint64_t request_sequence = 0;
};

class MSG_BUF
{
public:
  Uint8_t *data;
  Int32_t len;
  Uint64_t from_addr;
#ifdef ROS
  Uint64_t time_stamp;
#else
  Int64_t time_stamp;
#endif

public:
  MSG_BUF(const Uint8_t *data_, Int32_t len_, Uint64_t from_addr_)
    :data(new Uint8_t[len_]), len(len_), from_addr(from_addr_){
    memcpy(data, data_, len);
  }

  ~MSG_BUF() {
    if(data) {
      delete[] data;
      data = nullptr;
    }
    len=0;
    from_addr=0;
  }
};


class MSG_BUF_META
{
public:
  enum {MSG_BUF_META_CACHE_SIZE=4096000};
  // 拆包组包处理
  Uint8_t *cache=0;
  Uint32_t cache_size;
  Uint64_t all_fragment_sequence;
  bool recv_completed;
  bool request_resend;
  Uint64_t request_sequence = 0;
  Uint64_t single_group_fragment_count; // 记录单组接收到的分包总数
  Uint64_t ts_exception; // 记录不正常帧的开始时间

  Uint32_t single_group_number;
  Uint16_t single_group_sequence;

  Uint32_t message_id = 0;

  bool first_frame; // 第一帧

  Int64_t recv_fist_frame_timestamp;
  Int64_t recv_last_frame_timestamp;

  Uint8_t m_MpuBuf[1004]={0};
  Int64_t time;
  bool m_FirstFlage = false;

  Uint64_t errSum;
  Uint64_t request_sequence_save;
  Uint16_t sum;

  MSG_BUF_META() {
    first_frame = true;
    request_resend=false;
    request_sequence=0;
    cache = new Uint8_t[MSG_BUF_META_CACHE_SIZE];
    cache_size = 0;
  }
  ~MSG_BUF_META(){
    if(cache != 0) {
      delete[] cache;
      cache = 0;
    }
  }

  void Clear() {
    cache_size = 0;
    all_fragment_sequence = 0;
    recv_completed = false;
    request_resend = false;
    request_sequence = 0;
    single_group_fragment_count=0;
    ts_exception=0;
    first_frame=true;
    recv_fist_frame_timestamp=0;
    recv_last_frame_timestamp=0;
  }
};


class MSG_BUF_META_MPU
{
public:
  enum {MSG_BUF_META_MPU_CACHE_SIZE=4096000};
  // 拆包组包处理
  Uint8_t *cache=0;
  Uint32_t cache_size;
  Uint64_t all_fragment_sequence;
  bool recv_completed;
  bool request_resend = true;
  Uint64_t request_sequence=1;
  Uint64_t single_group_fragment_count; // 记录单组接收到的分包总数
  Uint64_t ts_exception; // 记录不正常帧的开始时间

  bool first_frame; // 第一帧

  MSG_BUF_META_MPU() {
    first_frame = true;
    cache = new Uint8_t[MSG_BUF_META_MPU_CACHE_SIZE];
    cache_size = 0;
  }
  ~MSG_BUF_META_MPU() {
    if(cache != 0) {
      delete[] cache;
      cache = 0;
    }
  }

  void Clear() {
    cache_size = 0;
    all_fragment_sequence = 0;
    recv_completed = false;
    request_resend = true;
    request_sequence=1;
    single_group_fragment_count=0;
    ts_exception=0;
    first_frame=true;
  }
};

template <typename T>
class ThreadFuncHelper {
public:
  ThreadFuncHelper() {
    udp_node_ = Nullptr_t;
    thread_func_ = Nullptr_t;
  }

  void SetThreadFunc(T* comm_node, void (T::*func)(void)) {
    udp_node_ = comm_node;
    thread_func_ = func;
  }

  void operator ()() {
    if ((Nullptr_t != udp_node_) && (Nullptr_t != thread_func_)) {
      (udp_node_->*thread_func_)();
    } else {
      LOG_ERR << "Invalid thread function.";
    }
  }

private:
  T* udp_node_;
  void (T::*thread_func_)(void);
};

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
  bool enable_send;
  bool enable_recv;
  Int32_t     mode; // 发送时是否为广播模式
  Uint16_t    rcv_port; // 接收端的端口
  Uint16_t    snd_remote_port; // 发送端的端口
  Uint16_t    snd_local_port; // 发送端本地绑定端口
  Char_t   snd_addr[20]; // 发送端的IP地址

  void Clear() {
    enable_send = false;
    enable_recv = false;
    rcv_port = 0;
    snd_remote_port = 0;
    snd_local_port=0;
    mode = MODE_UNICAST;
    common::com_memset(snd_addr, 0, sizeof(snd_addr));
  }

  UdpParam() {
    Clear();
  }
};

/**
 * @brief The UDPProtocol_1 struct 应用层协议1数据定义 Motorola格式
 */
struct UDPProtocol_1 {
    Uint32_t payload_len;  // 有效数据长度 32-bit
    Uint64_t timestamp; // 时间戳   48-bit
    Uint16_t sequence_number; // 序列号 16-bit
    Uint32_t message_id; // 消息ID  32-bit
    Uint32_t message_counter; // 消息计数器 32-bit
};

struct UDPProtocol_2 {
    Uint8_t fragment_type; // 拆包类型 4-bit
    Uint16_t payload_len;  // 有效数据长度 12-bit
    Uint8_t *buf; //  数据部分  ， 最大存储空间 8*1002 bit
};

struct UDPProtocol_2_ext {
    Uint8_t fragment_type; // 拆包类型 4-bit
    Uint16_t payload_len;  // 有效数据长度 12-bit
    Uint64_t all_fragment_sequence; // 总帧序号  64-bit
    Uint32_t single_group_number; //组编号 32-bit
    Uint16_t single_group_fragment_count; //单组总包数 16-bit
    Uint16_t single_group_sequence; // 单组序号  16-bit
    Uint8_t checksum; // 校验和   8-bit
    Uint8_t *buf; // 数据指针  最大存储空间 8*1002 bit
};

typedef UDPProtocol_1           UDPP1_t;
typedef UDPProtocol_2           UDPP2_t;
typedef UDPProtocol_2_ext   UDPP2_EXT_t;


struct St_DeliverPacket;


class UDPCommData;
class UDPCommPrivate;
class UdpNodePrivate;

class UDPComm
{
public:
    enum {
      FRAGMENT_TYPE_NOSPLIT=0,  // 无拆包
      FRAGMENT_TYPE_START=0x01, // 拆包——起始包
      FRAGMENT_TYPE_SPLIT=0x02, // 拆包——拆分包
      FRAGMENT_TYPE_END=0x03    // 拆包——结束包
    };
    UDPComm();
    UDPComm(UDPCommPrivate &dd);
    void init();
    virtual ~UDPComm();

    bool Start(const UdpParam& comm_param);
    bool Stop();

    bool Subscribe( const Char_t* channel,
                    void (*handlerMethod)(const Char_t *channel,Uint64_t timestamp,
                                         const void *buf, Int32_t buf_len,
                                         bool has_time, void *user),
                    void* user);

    bool StartReceiving();
    virtual bool Publish(const Char_t *channel, const void *buf, Int32_t buf_len) = 0;
    virtual bool Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len) = 0;

protected:
    virtual Int32_t SocketCommReceivePacket(void *handle,
                                            const void *buf_in,
                                            Int32_t buf_len,
                                            uint64_t from_addr, Uint64_t timestamp) = 0;
public:
    Int32_t SocketCommReceivePacketByMessageCommon(
            socket_comm_t *handle,
            Uint8_t *buf, Uint32_t buf_len,
            UDPP1_t *udpp_1,
            MSG_BUF_META *msg_buf_meta, Uint64_t timestamp);

private:
    void TheadReceivingMessages();
    void ThreadProcMessage();
    void ThreadDeliverPacket();


protected:
//  UdpParam d_ptr->param_; // 通信连接参数

//  bool d_ptr->receiving_flag_; // 是否已经开始接收消息
//  void* d_ptr->communication_handle_;

//  SocketReceivingChannelInfo d_ptr->receiving_channel_info_;// 接收相关的信息
//  SocketSendingChannelInfo d_ptr->sending_channel_info_; // 发送相关的信息

//  enum { MAX_SUBSCRIPTION_NUM = 40 };
//  Int32_t d_ptr->subscription_count_;
//  SocketSubscription d_ptr->subscription_list_[MAX_SUBSCRIPTION_NUM];

  common::os::Mutex lock_sending_channel_;

  common::os::Thread receiving_thread_;
  friend class ThreadFuncHelper<UDPComm>;
  ThreadFuncHelper<UDPComm> receiving_thread_func_helper_;

#if ENABLE_THREADPROCMESSAGE
//  bool proc_flag_;
 common::os::Thread proc_thread_;
 ThreadFuncHelper<UDPComm> proc_thread_func_helper_;
#endif

  std::mutex mtx_msg_buf_;
  std::condition_variable cv_msg_buf_;
  std::queue<MSG_BUF *> v_msg_buf_;
  std::thread thread_proc_;
  std::atomic_bool proc_flag_;

  Char_t *receiving_msg_buf_=nullptr;

  std::thread m_DeliverPacketThread;
  std::atomic_bool m_PacketFlag;
  std::mutex mtx_mpu_Packet_;
  std::condition_variable cv_mpu_Packet_;

  UDPP1_t udpp_1;
  MSG_BUF_META *msg_buf_meta = nullptr;

  std::mutex mtx_msg_send;

  std::vector<std::shared_ptr<St_DeliverPacket>> VecSt_DeliverPacketPtr;

protected:
  UDPCommData *d_ptr;
};



/**
 * @class UdpComNode
 * @brief 消息通信
 */
class UdpComNode: public UDPComm {
public:
  UdpComNode();
  UdpComNode(UdpNodePrivate &dd);
  void init();

  bool Publish(const Char_t *channel, const void *buf, Int32_t buf_len);
  bool Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len);

protected:
  virtual Int32_t SocketCommReceivePacket(void *handle,
                                          const void *buf_in,
                                          Int32_t buf_len,
                                          uint64_t from_addr, Uint64_t timestamp);
};

class UdpIDNode: public UDPComm
{
public:
    UdpIDNode();
    ~UdpIDNode(){}

  bool Publish(const Char_t *channel, const void *buf, Int32_t buf_len);
  bool Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len);

protected:
  virtual Int32_t SocketCommReceivePacket(void *handle,
                                          const void *buf_in,
                                          Int32_t buf_len,
                                          uint64_t from_addr, Uint64_t timestamp);
private:
  Int32_t SocketCommReceivePacketByMessageID(socket_comm_t *handle,
                                              const void *buf_in,
                                              Int32_t buf_len,
                                              uint64_t from_addr,Uint64_t timestamp);
  Int32_t SocketCommReceivePacketByMessageIDCommon(
      socket_comm_t *handle,
      Uint8_t *buf, Uint32_t buf_len,
      UDPP1_t *udpp_1,
      MSG_BUF_META *msg_buf_meta, Uint64_t timestamp);

  Uint16_t GetRequestResendErrorCode();


#if 0
private:
  void ThreadCycleReceiveMessage();

private:
  bool send_flag_=false;
  common::os::Thread send_thread_;
  friend class ThreadFuncHelper<UdpIDNode>;
 ThreadFuncHelper<UdpIDNode> send_thread_func_helper_;

#endif

private:
  bool map_first_frame;
  bool perception_first_frame;
  bool system_first_frame;
  bool spdpln_first_frame;
  bool pathpln_first_frame;

  MSG_BUF_META msg_meta_map;
  MSG_BUF_META msg_meta_perception;
  MSG_BUF_META msg_meta_system;
  MSG_BUF_META msg_meta_spdpln;
  MSG_BUF_META msg_meta_pathpln;
  MSG_BUF_META msg_meta_PartialMap;
};


class UdpMPUNode: public UDPComm
{
public:
    UdpMPUNode();
    ~UdpMPUNode(){}

  bool Publish(const Char_t *channel, const void *buf, Int32_t buf_len);
  bool Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len);

protected:
  virtual Int32_t SocketCommReceivePacket(void *handle,
                                          const void *buf_in,
                                          Int32_t buf_len,
                                          uint64_t from_addr, Uint64_t timestamp);
private:
  Int32_t SocketCommReceivePacketByMessageMPU(socket_comm_t *handle,
                                              const void *buf_in,
                                              Int32_t buf_len,
                                              uint64_t from_addr, Uint64_t timestamp);
  Int32_t SocketCommReceivePacketByMessageMPUCommon(
      socket_comm_t *handle,
      Uint8_t *buf, Uint32_t buf_len,
      UDPP1_t *udpp_1,
      MSG_BUF_META *msg_buf_meta);

private:
  Int32_t ReceiveLanePositionDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceiveMapDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceivePerceptionDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceiveSystemDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceiveControlDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceivePlanningSpdPlnBhvPlnDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);
  Int32_t ReceivePlanningPathPlnDisplayData(socket_comm_t *handle, Uint8_t *buf, Uint32_t buf_len, UDPP1_t *udpp_1);

#if 0
private:
  void ThreadCycleReceiveMessage();

private:
  bool send_flag_=false;
  common::os::Thread send_thread_;
  friend class ThreadFuncHelper<UdpIDNode>;
  ThreadFuncHelper<UdpIDNode> send_thread_func_helper_;

#endif

private:
  bool map_first_frame;
  bool perception_first_frame;
  bool system_first_frame;
  bool spdpln_first_frame;
  bool pathpln_first_frame;

  MSG_BUF_META msg_meta_map;
  MSG_BUF_META msg_meta_perception;
  MSG_BUF_META msg_meta_system;
  MSG_BUF_META msg_meta_spdpln;
  MSG_BUF_META msg_meta_pathpln;
  MSG_BUF_META msg_meta_FitSamplePoint;
  MSG_BUF_META msg_meta_VehicleCorrectState;
  MSG_BUF_META msg_meta_VehicleState;
  MSG_BUF_META msg_meta_CorrectPosition;
  MSG_BUF_META msg_meta_PartialMap;
  MSG_BUF_META msg_meta_PartialMapLineFit;
  MSG_BUF_META msg_meta_OrigCamera;
  MSG_BUF_META msg_meta_MapLocationEhr;
  MSG_BUF_META msg_meta_LocationPosition;
  MSG_BUF_META msg_meta_DecisionPosition;
  MSG_BUF_META msg_meta_CameraFitLine;
  MSG_BUF_META msg_meta_FitPointsMap;
  MSG_BUF_META msg_meta_MpuState;
};

/**
 * @brief UDP无通信协议通信版本，基于消息订阅方式
 * 
 */
class UDPNoProtocolComm : public UDPComm
{
public:
  UDPNoProtocolComm(const std::string& channel_name)
    :ChannelName(channel_name){}
  ~UDPNoProtocolComm(){}

protected:
  Int32_t SocketCommReceivePacket(void *handle,
                                  const void *buf_in,
                                  Int32_t buf_len,
                                  uint64_t from_addr,
                                  Uint64_t timestamp) override;

  bool Publish(const Char_t *channel, const void *buf, Int32_t buf_len) override;
  bool Publish(const Uint32_t message_id, const void *buf, Int32_t buf_len) {
    // not implement
    return false;
  }

private:
  std::string ChannelName;
};
  
}  // mpu
}  // framework
}  // phoenix

#endif  // PHOENIX_FRAMEWORK_UDP_NODE_H_


