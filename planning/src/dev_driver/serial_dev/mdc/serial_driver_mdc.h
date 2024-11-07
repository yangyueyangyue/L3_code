//
#ifndef PHOENIX_SERIAL_DEV_SERIAL_DRIVER_MDC_H_
#define PHOENIX_SERIAL_DEV_SERIAL_DRIVER_MDC_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "container/ring_buffer.h"
#include "container/string_ring_buffer.h"
#include "serial_dev/serial_driver.h"

#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <iostream>


#define MDC_UART_DEV_INVALID (0)
#define MDC_UART_DEV_16 (1)
#define MDC_UART_DEV_18 (2)
#define MDC_UART_DEV MDC_UART_DEV_18


#if (ENABLE_SERIAL_DEV_MDC)
#if (MDC_UART_DEV == MDC_UART_DEV_16)
#include "impl_type_uartdataparam.h"
#include "mdc/sensor/uartrxserviceinterface_proxy.h"
#elif (MDC_UART_DEV == MDC_UART_DEV_18)
#include "mdc/mdcuart/impl_type_uartdataparam.h"
#include "mdc/mdcuart/uartrxserviceinterface_proxy.h"
#include "ara/log/logging.h"
#else
  ERROR: Invalid MDC UART device.
#endif
#endif // #if (ENABLE_SERIAL_DEV_MDC)


namespace phoenix {
namespace serial_dev {


#if (ENABLE_SERIAL_DEV_MDC)

#if (MDC_UART_DEV == MDC_UART_DEV_16)

class SerialDriverMdc : public SerialDriver {
public:
  SerialDriverMdc();
  virtual ~SerialDriverMdc();

  virtual bool OpenPort(const SerialPortParam& param);
  virtual bool ClosePort();
  virtual Int32_t Send(const Uint8_t* data, Int32_t data_size);
  virtual Int32_t ReadWait(
      Uint8_t* data, Int32_t max_data_size = 1, Uint32_t timeout_ms = 50);

private:
  using UartRxProxy = mdc::sensor::proxy::UartRxServiceInterfaceProxy;
  // 服务发现回调
  void ServiceAvailabilityCallback(
      ara::com::ServiceHandleContainer<UartRxProxy::HandleType> handles, 
      ara::com::FindServiceHandle handler);
  // 接收数据回调
  void UartDataEventCallback();

private:
  enum { MAX_PORT_NUM = 2 };
  enum { MAX_RECV_DATA_BUFF_SIZE = 8*1024 };
  enum { UARTSEND_METHOD_PERIOD = 500 };
  enum { UART_DATA_TX_MAX_LEN = 256 };

  bool port_is_open_;
  Int32_t port_id_;
  Int32_t instance_id_;

  std::unique_ptr<UartRxProxy> uart_rx_proxy_;

  boost::mutex receiving_lock_;
  boost::mutex receiving_uart_data_lock_;
  boost::mutex sending_uart_data_lock_;
  boost::condition_variable receiving_uart_data_cond_;

  // 环形缓冲区
  common::StringRingBuf recv_ring_buff_;
  // 环形缓冲区中的字符串存储区
  Char_t data_of_recv_ring_buff_[MAX_RECV_DATA_BUFF_SIZE];
};

#elif (MDC_UART_DEV == MDC_UART_DEV_18)


class SerialDriverMdc : public SerialDriver {
public:
  SerialDriverMdc();
  virtual ~SerialDriverMdc();

  virtual bool OpenPort(const SerialPortParam& param);
  virtual bool ClosePort();
  virtual Int32_t Send(const Uint8_t* data, Int32_t data_size);
  virtual Int32_t ReadWait(
      Uint8_t* data, Int32_t max_data_size = 1, Uint32_t timeout_ms = 50);

private:
  using ProxyUart = mdc::mdcuart::proxy::UartRxServiceInterfaceProxy;

  // 服务发现回调
 void ServiceAvailabilityCallback(
       ara::com::ServiceHandleContainer<ProxyUart::HandleType> handles,
       ara::com::FindServiceHandle);
  // 接收数据回调
  void UartDataEventCallback();
  void UartSetData();

private:
  enum { MAX_PORT_NUM = 2 };
  enum { MAX_RECV_DATA_BUFF_SIZE = 8*1024 };
  enum { UARTSEND_METHOD_PERIOD = 500 };
  enum { UART_DATA_TX_MAX_LEN = 256 };

  bool port_is_open_;
  Int32_t port_id_;
  Int32_t instance_id_;

  std::unique_ptr<ProxyUart> uart_rx_proxy_;

  boost::mutex receiving_lock_;
  boost::mutex receiving_uart_data_lock_;
  boost::mutex sending_uart_data_lock_;
  boost::condition_variable receiving_uart_data_cond_;

  // 环形缓冲区
  common::StringRingBuf recv_ring_buff_;
  // 环形缓冲区中的字符串存储区
  Char_t data_of_recv_ring_buff_[MAX_RECV_DATA_BUFF_SIZE];
};

#else
  ERROR: Invalid MDC UART device.
#endif

#endif  // #if (ENABLE_SERIAL_DEV_MDC)


} // namespace serial_dev
} // namespace phoenix


#endif  // PHOENIX_SERIAL_DEV_SERIAL_DRIVER_MDC_H_
