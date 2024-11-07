#ifndef PHOENIX_SERIAL_DEV_SERIAL_DRIVER_H_
#define PHOENIX_SERIAL_DEV_SERIAL_DRIVER_H_

#include "utils/macros.h"
#include "utils/com_utils.h"


namespace phoenix {
namespace serial_dev {


struct SerialPortParam {
  /// 定义波特率
  enum BaudRate {
    BAUD_RATE_INVALID = -1,
    BAUD_RATE_1200 = 1200,
    BAUD_RATE_2400 = 2400,
    BAUD_RATE_4800 = 4800,
    BAUD_RATE_9600 = 9600,
    BAUD_RATE_19200 = 19200,
    BAUD_RATE_38400 = 38400,
    BAUD_RATE_57600 = 57600,
    BAUD_RATE_115200 = 115200,
    BAUD_RATE_230400 = 230400
  };
  /// 定义数据位长度
  enum DataBits {
    DATA_BITS_INVALID = -1,
    DATA_BITS_7 = 7,
    DATA_BITS_8 = 8
  };
  /// 定义奇偶校验类型
  enum Parity {
    PARITY_INVALID = -1,
    PARITY_NO = 0,
    PARITY_EVEN = 2,
    PARITY_ODD = 3
  };
  /// 定义停止位长度
  enum StopBits {
    STOP_BITS_INVALID = -1,
    STOP_BITS_ONE = 1,
    STOP_BITS_TWO = 2
  };

  /// 端口名称
  Char_t port_name[128];
  /// 数据位长度
  Int32_t data_bits;
  /// 奇偶校验类型
  Int32_t parity;
  /// 停止位长度
  Int32_t stop_bits;
  /// 波特率
  Int32_t baud_rate;

  void Clear() {
    common::com_memset(port_name, 0, sizeof(port_name));
    data_bits = DATA_BITS_8;
    parity = PARITY_NO;
    stop_bits = STOP_BITS_ONE;
    baud_rate = BAUD_RATE_115200;
  }

  SerialPortParam() {
    Clear();
  }
};


class SerialDriver {
public:
  SerialDriver();
  virtual ~SerialDriver();

  virtual bool OpenPort(const SerialPortParam& param);
  virtual bool ClosePort();
  virtual Int32_t Send(const Uint8_t* data, Int32_t data_size);
  virtual Int32_t ReadWait(
      Uint8_t* data, Int32_t max_data_size = 1, Uint32_t timeout_ms = 50);
};


} // namespace serial_dev
} // namespace phoenix


#endif  // PHOENIX_SERIAL_DEV_SERIAL_DRIVER_H_
