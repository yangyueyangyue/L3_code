#ifndef PHOENIX_FRAMEWORK_TCP_CLIENT_H_
#define PHOENIX_FRAMEWORK_TCP_CLIENT_H_

#include "utils/macros.h"
#include <arpa/inet.h>
#include "os/thread.h"
#include "socket_communication.h"
#include <functional>
namespace phoenix {
namespace framework {

typedef std::function<void (const Char_t *buf, const Int32_t buf_len)> TcpClientRecvMsgCallback;

class TcpClient {
public:
  TcpClient();
  ~TcpClient();

  bool Start(const Char_t *server_addr, const Uint16_t server_port);
  bool Stop();
  bool StartReceiving();
  void TheadReceivingMessages();
  void SetRecvMsgCallback(TcpClientRecvMsgCallback callback){
    recv_msg_callback_ = std::move(callback);
  }
  Int32_t Send(const Char_t *buf, const Int32_t buf_len);
private:
  Char_t  server_addr_[20];
  Uint16_t  server_port_;
  Int32_t sock_fd_;
  Char_t receiving_msg_buf_[65536];
  bool receiving_flag_;
  SocketCommInstance_t socket_communication_;
  common::os::ThreadFuncHelper<TcpClient> receiving_thread_func_helper_;
  common::os::Thread receiving_thread_;
  TcpClientRecvMsgCallback recv_msg_callback_;
};


}  // framework
}  // phoenix

#endif  // PHOENIX_FRAMEWORK_TCP_CLIENT_H_
