//
#include "serial_dev/posix/serial_driver_posix.h"

#include <stdio.h>       //标准输入输出,如printf、scanf以及文件操作
#include <stdlib.h>      //标准库头文件，定义了五种类型、一些宏和通用工具函数
#include <unistd.h>      //定义 read write close lseek 等Unix标准函数
#include <sys/types.h>   //定义数据类型，如 ssiz e_t off_t 等
#include <sys/stat.h>    //文件状态
#include <fcntl.h>       //文件控制定义
#include <termios.h>     //终端I/O
#include <errno.h>       //与全局变量 errno 相关的定义
#include <getopt.h>      //处理命令行参数
#include <string.h>      //字符串操作
#include <time.h>        //时间
#include <sys/select.h>  //select函数

#include "utils/log.h"


namespace phoenix {
namespace serial_dev {

#if (ENABLE_SERIAL_DEV_POSIX)

SerialDriverPosix::SerialDriverPosix() {
  fd_serial_port_ = -1;
}

SerialDriverPosix::~SerialDriverPosix() {
  ClosePort();
}

bool SerialDriverPosix::OpenPort(const SerialPortParam& param) {
  if (fd_serial_port_ >= 0) {
    LOG_WARN << "This port has been opened.";
    return (true);
  }

  fd_serial_port_ = open(param.port_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_serial_port_ < 0) {
    LOG_ERR << "Failed to open serial port \"" << param.port_name << "\".";
    return false;
  }

  // 设置串口阻塞， 0：阻塞， FNDELAY：非阻塞
  // 阻塞，即使前面在open串口设备时设置的是非阻塞的
  Int32_t ret = fcntl(fd_serial_port_, F_SETFL, 0);
  if (ret < 0) {
    close(fd_serial_port_);
    fd_serial_port_ = -1;
    LOG_ERR << "Failed to set serial port to block mode.";
    return false;
  }

  if (0 == isatty(fd_serial_port_)) {
    close(fd_serial_port_);
    fd_serial_port_ = -1;
    LOG_ERR << "Standard input is not a terminal device.";
    return false;
  }

  struct termios oldtio;
  struct termios newtio;

  // 保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
  if (0 != tcgetattr(fd_serial_port_, &oldtio)) {
    close(fd_serial_port_);
    fd_serial_port_ = -1;
    LOG_ERR << "Failed to get attribute of the serial port.";
    return false;
  }

  // 新termios参数清零
  bzero(&newtio, sizeof(newtio));
  // CLOCAL -- 忽略modem控制线, 本地连线, 不具数据机控制功能,
  // CREAD --  打开接受者, 使能接收标志
  newtio.c_cflag |= CLOCAL | CREAD;
  // 设置数据位数
  // 清数据位标志
  newtio.c_cflag &= ~CSIZE;
  switch (param.data_bits) {
  case (SerialPortParam::DATA_BITS_7):
    newtio.c_cflag |= CS7;
    break;
  case (SerialPortParam::DATA_BITS_8):
    newtio.c_cflag |= CS8;
    break;
  default:
    newtio.c_cflag |= CS8;
    LOG_ERR << "Unsupported data size, set default (8).";
    break;
  }
  // 设置校验位
  switch (param.parity) {
  case (SerialPortParam::PARITY_ODD):
    // 奇校验
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case (SerialPortParam::PARITY_EVEN):
    // 偶校验
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case (SerialPortParam::PARITY_NO):
    // 无校验
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    newtio.c_cflag &= ~PARENB;
    LOG_ERR << "Unsupported parity, set default (No Parity).";
    break;
  }
  // 设置停止位
  switch (param.stop_bits) {
  case (SerialPortParam::STOP_BITS_ONE):
    newtio.c_cflag &= ~CSTOPB;
    break;
  case (SerialPortParam::STOP_BITS_TWO):
    newtio.c_cflag |= CSTOPB;
    break;
  default:
    newtio.c_cflag &= ~CSTOPB;
    LOG_ERR << "Unsupported stop bits, set default (1).";
    break;
  }
  // 设置波特率 1200/2400/4800/9600/19200/38400/57600/115200/230400
  switch (param.baud_rate) {
  case (SerialPortParam::BAUD_RATE_1200):
    cfsetispeed(&newtio, B1200);
    cfsetospeed(&newtio, B1200);
    break;
  case (SerialPortParam::BAUD_RATE_2400):
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case (SerialPortParam::BAUD_RATE_4800):
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case (SerialPortParam::BAUD_RATE_9600):
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case (SerialPortParam::BAUD_RATE_19200):
    cfsetispeed(&newtio, B19200);
    cfsetospeed(&newtio, B19200);
    break;
  case (SerialPortParam::BAUD_RATE_38400):
    cfsetispeed(&newtio, B38400);
    cfsetospeed(&newtio, B38400);
    break;
  case (SerialPortParam::BAUD_RATE_57600):
    cfsetispeed(&newtio, B57600);
    cfsetospeed(&newtio, B57600);
    break;
  case (SerialPortParam::BAUD_RATE_115200):
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case (SerialPortParam::BAUD_RATE_230400):
    cfsetispeed(&newtio, B230400);
    cfsetospeed(&newtio, B230400);
    break;
  default:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    LOG_ERR << "Unsupported baud rate, set default (115200).";
    break;
  }
  // 设置read读取最小字节数和超时时间
  newtio.c_cc[VTIME] = 1;  // 读取一个字符等待1*(1/10)s
  newtio.c_cc[VMIN] = 1;   // 读取字符的最少个数为1

  // 清空缓冲区
  tcflush(fd_serial_port_, TCIFLUSH);
  // 激活新设置
  if (0 != tcsetattr(fd_serial_port_, TCSANOW, &newtio)) {
    close(fd_serial_port_);
    fd_serial_port_ = -1;
    LOG_ERR << "Failed to set parameter to this serial port.";
    return false;
  }

  return true;
}

bool SerialDriverPosix::ClosePort() {
  if (fd_serial_port_ >= 0) {
    close(fd_serial_port_);
    fd_serial_port_ = -1;
  }

  return true;
}

Int32_t SerialDriverPosix::Send(const Uint8_t* data, Int32_t data_size) {
  if (fd_serial_port_ < 0) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  Int32_t bytes = write(fd_serial_port_, data, data_size);
  if (bytes != data_size) {
    LOG_ERR << "Failed to send data to serial port.";
    tcflush(fd_serial_port_, TCOFLUSH);
    return (-1);
  }

  return (bytes);
}

Int32_t SerialDriverPosix::ReadWait(
    Uint8_t* data, Int32_t max_data_size, Uint32_t timeout_ms) {
  if (fd_serial_port_ < 0) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  fd_set read_fds;  //读文件操作符
  FD_ZERO(&read_fds);
  // 每次调用select之前都要重新在read_fds中设置文件描述符，
  // 因为事件发生以后，文件描述符集合将被内核修改
  FD_SET(fd_serial_port_, &read_fds);
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
  Int32_t ret = select(fd_serial_port_+1, &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    return -1;
  }
  if(!FD_ISSET(fd_serial_port_, &read_fds)) {
    return -1;
  }

  Int32_t bytes = read(fd_serial_port_, data, max_data_size);

  return (bytes);
}

#endif  // #if (ENABLE_SERIAL_DEV_POSIX)

} // namespace serial_dev
} // namespace phoenix



