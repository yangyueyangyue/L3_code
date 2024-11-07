#include "communication/tcp_client.h"
#include <sys/fcntl.h>

namespace phoenix {
namespace framework {

TcpClient::TcpClient() {
  common::com_memset(server_addr_, 0, sizeof(server_addr_));
  server_port_ = 0;
  receiving_flag_ = false;
  sock_fd_ = -1;
  recv_msg_callback_ = Nullptr_t;
}

TcpClient::~TcpClient() {
  Stop();
}

bool TcpClient::Start(const Char_t *server_addr, const Uint16_t server_port){
  if (sock_fd_ >= 0) {
    LOG_ERR << "TcpClient::Start Already start.";
    return false;
  }

  Int32_t server_addr_len = strlen(server_addr);
  common::com_memcpy(server_addr_, server_addr, server_addr_len);
  server_port_ = server_port;

  fd_set fd_set_read;
  fd_set fd_set_write;
  struct timeval timeout;
  Int32_t err = 0;
  Uint32_t errlen = sizeof(err);

  Int32_t socket_flag = 0;
  struct sockaddr_in serveraddr;
  common::com_memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = inet_addr(server_addr);
  serveraddr.sin_port = htons(server_port_);

  sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    LOG_ERR << "TcpClient::Start Failed to create sock fd.";
    return false;
  }
  socket_flag = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, socket_flag | O_NONBLOCK);
  Int32_t connect_ret = connect(sock_fd_,
                (struct sockaddr*)&serveraddr,
                sizeof(struct sockaddr));
  if (0 != connect_ret) {
    if (EINPROGRESS == errno) {
      LOG_INFO(3) << "TcpClient::Start Doing connection.";
      // 正在处理连接
      FD_ZERO(&fd_set_read);
      FD_ZERO(&fd_set_write);
      FD_SET(sock_fd_, &fd_set_read);
      FD_SET(sock_fd_, &fd_set_write);
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      Int32_t rc = select(sock_fd_+ 1,
                  &fd_set_read, &fd_set_write, NULL, &timeout);
      // select调用失败
      if (rc < 0) {
        LOG_ERR << "TcpClient::Start Failed to connect to tcp client, error="
                << strerror(errno);
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
      }
      // 连接超时
      if (0 == rc) {
        LOG_ERR << "TcpClient::Start Failed to connect to tcp client, timeout.";
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
      }
      // 当连接建立遇到错误时，描述符变为即可读，也可写，rc=2
      // 遇到这种情况，可调用getsockopt函数
      if (2 == rc) {
        if (-1 == getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR,
                             &err, &errlen)) {
          LOG_ERR << "TcpClient::Start Failed to getsockopt(SO_ERROR), error="
                  << strerror(errno);
          close(sock_fd_);
          sock_fd_ = -1;
          return false;
        }
        if (err) {
          LOG_ERR << "TcpClient::Start Failed to connect to tcp client, error="
                  << strerror(err);
          close(sock_fd_);
          sock_fd_ = -1;
          return false;
        }
      }
      // 当连接成功建立时，描述符变成可写,rc=1
      if ((1 == rc) && FD_ISSET(sock_fd_, &fd_set_write)) {
        connect_ret = rc;
        printf("TcpClient::Start Connect success\n");
        LOG_INFO(3) << "TcpClient::Start Succeeded to connecte to notifying channel.";
      }
    }
  }
  if (connect_ret < 0) {
    LOG_ERR << "TcpClient::Start Can't connect to server for tcp client.";
    close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  socket_flag = fcntl(sock_fd_, F_GETFL, 0);
  fcntl(sock_fd_, F_SETFL, socket_flag & (~O_NONBLOCK));

  return true;
}

bool TcpClient::Stop(){
  LOG_INFO(3) << "TcpClient::Stop Stop Tcp Client ... ";

  if (receiving_flag_) {
    LOG_INFO(3) << "TcpClient::Stop Waiting to stop tcp client receiving thread  ... ";
    receiving_flag_ = false;
    recv_msg_callback_ = Nullptr_t;
    receiving_thread_.Join();
  }

  if (sock_fd_ >= 0) {
    Int32_t ret = close(sock_fd_);
    if (ret < 0) {
      LOG_ERR << "TcpClient::Stop Failed to close sock for tcp client.";
    }
    sock_fd_ = -1;
  }
  LOG_INFO(3) << "TcpClient::Stop Stop Tcp Client ... [OK]";
  return true;
}

bool TcpClient::StartReceiving(){
  if (receiving_flag_) {
    LOG_WARN << "TcpClient::StartReceiving Already start.";
    return true;
  }

  receiving_flag_ = true;
  receiving_thread_func_helper_.SetThreadFunc(
        this, &TcpClient::TheadReceivingMessages);
  common::os::Thread::Parameter thread_param;
  bool ret = receiving_thread_.Create(thread_param,
                                      &receiving_thread_func_helper_);
  if (!ret) {
    receiving_flag_ = false;
    LOG_ERR << "TcpClient::StartReceiving Failed to create Tcp Client receiving thread.";
    return false;
  }

  return true;
}

void TcpClient::TheadReceivingMessages(){
  LOG_INFO(3) << "TcpClient::TheadReceivingMessages Receiving Thread ... [Started]";
  if (sock_fd_ < 0) {
    LOG_ERR << "TcpClient::TheadReceivingMessages Invalid receiving sock.";
    return;
  }
  while (receiving_flag_) {
    fd_set read_fds;  //读文件操作符
    FD_ZERO(&read_fds);
    // 每次调用select之前都要重新在read_fds中设置文件描述符，
    // 因为事件发生以后，文件描述符集合将被内核修改
    FD_SET(sock_fd_, &read_fds);
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
    Int32_t ret = select(sock_fd_+1,
                        &read_fds, Nullptr_t, Nullptr_t, &timeout);
    if (ret < 0) {
      LOG_ERR << "TcpClient::TheadReceivingMessages Failed to select fds.";
      return;
    }

    if(FD_ISSET(sock_fd_, &read_fds)) {
      Int32_t bytes = recv(sock_fd_, receiving_msg_buf_, sizeof(receiving_msg_buf_), 0);
      if (bytes < 0) {
        LOG_ERR << "TcpClient::TheadReceivingMessages Failed to receive tcp message.";
        return;
      }

      if(recv_msg_callback_) {
        recv_msg_callback_(receiving_msg_buf_, bytes);
      }
    }
  }
  
  LOG_INFO(3) << "TcpClient::TheadReceivingMessages Receiving Thread ... [Stopped]";
}

Int32_t TcpClient::Send(const Char_t *buf, const Int32_t buf_len){
  if (sock_fd_ < 0) {
    LOG_ERR << "TcpClient::Send Invalid send sock.";
    return -1;
  }

  if(buf == Nullptr_t || buf_len <= 0) {
    LOG_ERR << "TcpClient::Send buff data error.";
    return -1;
  }

  Int32_t bytes = send(sock_fd_, buf, buf_len, 0);
  if (bytes < 1) {
    LOG_ERR << "TcpClient::Send Failed to send message.";
    return -1;
  }

  return bytes; 
}


}  // framework
}  // phoenix
