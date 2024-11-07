//
#include "serial_dev/mdc/serial_driver_mdc.h"
#include <chrono>


namespace phoenix {
namespace serial_dev {

#if (ENABLE_SERIAL_DEV_MDC)

#if (MDC_UART_DEV == MDC_UART_DEV_16)

SerialDriverMdc::SerialDriverMdc() {
  port_is_open_ = false;
  port_id_ = -1;
  instance_id_ = -1;

  common::InitStringRingBuf(
        &recv_ring_buff_, data_of_recv_ring_buff_, MAX_RECV_DATA_BUFF_SIZE);
}

SerialDriverMdc::~SerialDriverMdc() {
  ClosePort();
}

bool SerialDriverMdc::OpenPort(const SerialPortParam& param) {
  if (port_is_open_) {
    LOG_WARN << "This port has been opened.";
    return (true);
  }
  Int32_t port = param.port_name[0];
  if ((port < 0) || (port >= MAX_PORT_NUM)) {
    LOG_ERR << "Invalid port id.";
    return (false);
  }
  //初始化
  port_id_ = port;
  instance_id_ = port_id_ + 1;

  UartRxProxy::StartFindService(
    [this](ara::com::ServiceHandleContainer<UartRxProxy::HandleType> handles, 
            ara::com::FindServiceHandle handler) {
        SerialDriverMdc::ServiceAvailabilityCallback(std::move(handles), handler);
    }, instance_id_);

  common::InitStringRingBuf(
        &recv_ring_buff_, data_of_recv_ring_buff_, MAX_RECV_DATA_BUFF_SIZE);

  port_is_open_ = true;

  return true;
}

bool SerialDriverMdc::ClosePort() {
  if (port_is_open_ >= 0) {
    receiving_uart_data_cond_.notify_one();
    port_is_open_ = false;
  }

  return true;
}

Int32_t SerialDriverMdc::Send(const Uint8_t* data, Int32_t data_size) {
  if (!port_is_open_) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  Int32_t bytes = 0;

  /// TODO: T.B.D.
#if 0
  std::vector<Uint8_t> vec_data(UART_DATA_TX_MAX_LEN, 0);
  UartDataParam param;
  param.data.swap(vec_data);
  for (Int32_t i = 0; i < data_size && i < UART_DATA_TX_MAX_LEN; i++) {
    param.data.push_back(data[i]);
    bytes++;
  }
  param.validLen = bytes;

  sending_uart_data_lock_.lock();
  
  auto handle = uart_rx_proxy_->UartDataSetMethod(param);
  if (handle.wait_for(
    std::chrono::milliseconds(UARTSEND_METHOD_PERIOD)) != 
    ara::core::FutureStatus::kReady) {
    LOG_ERR << "Failed to send data to serial port timeout.";
    return (-1);
  }
  auto output = handle.get();
  if (0 != static_cast<int>(output.result)) {
    LOG_ERR << "Failed to send data to serial port.";
    return (-1);
  }
  
  sending_uart_data_lock_.unlock();
#endif
  
  return (bytes);
}

Int32_t SerialDriverMdc::ReadWait(
    Uint8_t* data, Int32_t max_data_size, Uint32_t timeout_ms) {
  if (!port_is_open_) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  Int32_t bytes = 0;

  receiving_uart_data_lock_.lock();

  bytes = common::ReadFromStringRingBuf(
        &recv_ring_buff_, (Char_t*)data, max_data_size);

  receiving_uart_data_lock_.unlock();

  if (port_is_open_ && (bytes < 1)) {
    boost::unique_lock<boost::mutex> lock(receiving_lock_);
#if 0
    receiving_uart_data_cond_.wait(lock);
#else
    receiving_uart_data_cond_.wait_until(
          lock, boost::chrono::system_clock::now() +
                boost::chrono::milliseconds(timeout_ms));
    //receiving_uart_data_cond_.wait_for(
    //      lock, boost::chrono::milliseconds(timeout_ms));
#endif
  }
  
  return (bytes);
}

void SerialDriverMdc::ServiceAvailabilityCallback(
      ara::com::ServiceHandleContainer<UartRxProxy::HandleType> handles, 
      ara::com::FindServiceHandle handler) {
  for (Int32_t i = 0; i < handles.size(); i++) {
    Int32_t instance = static_cast<Uint16_t>(handles[i].GetInstanceId());
    if (instance != instance_id_) {
      continue;
    }
    if (Nullptr_t == uart_rx_proxy_) {
      uart_rx_proxy_ = std::make_unique<UartRxProxy>(handles[i]);
      uart_rx_proxy_->UartDataRxEvent.Subscribe(
            ara::com::EventCacheUpdatePolicy::kNewestN, 100);
      uart_rx_proxy_->UartDataRxEvent.SetReceiveHandler(
            [this]() {
        SerialDriverMdc::UartDataEventCallback();
      });
      break;
    }
  }
}

void SerialDriverMdc::UartDataEventCallback() {
  if (Nullptr_t == uart_rx_proxy_) {
    LOG_ERR << "Invalid UART RX proxy.";
    return;
  }

  // Lock
  receiving_uart_data_lock_.lock();

  uart_rx_proxy_->UartDataRxEvent.Update();
  const auto& uart_samples = uart_rx_proxy_->UartDataRxEvent.GetCachedSamples();
  for (auto sample : uart_samples) {
    // LOG_INFO(1) << "Receiving uart data from port(" << port_id_
    //             << "), seq=" << sample->seq
    //             << ", len=" << sample->validLen
    //             << ".";
    if (sample->validLen > 0) {
      common::WriteToStringRingBufOverride(
            &recv_ring_buff_, (const Char_t*)&(sample->data[0]),
            sample->validLen);
    }
  }
  uart_rx_proxy_->UartDataRxEvent.Cleanup();

  // Unlock
  receiving_uart_data_lock_.unlock();

  // Notify
  receiving_uart_data_cond_.notify_one();
}

#elif (MDC_UART_DEV == MDC_UART_DEV_18)

SerialDriverMdc::SerialDriverMdc() {
  port_is_open_ = false;
  port_id_ = -1;
  instance_id_ = -1;

  common::InitStringRingBuf(
        &recv_ring_buff_, data_of_recv_ring_buff_, MAX_RECV_DATA_BUFF_SIZE);
}

SerialDriverMdc::~SerialDriverMdc() {
  ClosePort();
}

bool SerialDriverMdc::OpenPort(const SerialPortParam& param) {
  if (port_is_open_) {
    LOG_WARN << "This port has been opened.";
    return (true);
  }
  Int32_t port = param.port_name[0];
  if ((port < 0) || (port >= MAX_PORT_NUM)) {
    LOG_ERR << "Invalid port id.";
    return (false);
  }
  //初始化
  port_id_ = port;
  instance_id_ = port_id_ + 1;
  std::string instance_str = std::to_string(instance_id_);
  ProxyUart::StartFindService(
        [this](ara::com::ServiceHandleContainer<ProxyUart::HandleType> handles,
        ara::com::FindServiceHandle handler) {
    SerialDriverMdc::ServiceAvailabilityCallback(std::move(handles), handler);
  }, ara::com::InstanceIdentifier(ara::core::StringView(instance_str.c_str())));

  common::InitStringRingBuf(
        &recv_ring_buff_, data_of_recv_ring_buff_, MAX_RECV_DATA_BUFF_SIZE);

  port_is_open_ = true;

  return true;
}

bool SerialDriverMdc::ClosePort() {
  if (port_is_open_ >= 0) {
    receiving_uart_data_cond_.notify_one();
    port_is_open_ = false;
  }

  return true;
}

Int32_t SerialDriverMdc::Send(const Uint8_t* data, Int32_t data_size) {
  if (!port_is_open_) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  Int32_t bytes = 0;

  /// TODO: T.B.D.

  return (bytes);
}

Int32_t SerialDriverMdc::ReadWait(
    Uint8_t* data, Int32_t max_data_size, Uint32_t timeout_ms) {
  if (!port_is_open_) {
    LOG_ERR << "This serial port has not been opened";
    return (-1);
  }

  Int32_t bytes = 0;

  receiving_uart_data_lock_.lock();

  bytes = common::ReadFromStringRingBuf(
        &recv_ring_buff_, (Char_t*)data, max_data_size);

  receiving_uart_data_lock_.unlock();

  if (port_is_open_ && (bytes < 1)) {
    boost::unique_lock<boost::mutex> lock(receiving_lock_);
#if 0
    receiving_uart_data_cond_.wait(lock);
#else
    receiving_uart_data_cond_.wait_until(
          lock, boost::chrono::system_clock::now() +
                boost::chrono::milliseconds(timeout_ms));
    //receiving_uart_data_cond_.wait_for(
    //      lock, boost::chrono::milliseconds(timeout_ms));
#endif
  }

  return (bytes);
}

void SerialDriverMdc::ServiceAvailabilityCallback(
    ara::com::ServiceHandleContainer<ProxyUart::HandleType> handles,
    ara::com::FindServiceHandle) {
  if(!port_is_open_) {
    return;
  }

#if 0
  for (auto it : handles) {
    m_logger.LogInfo() << "Uart Instance "
                       << it.GetInstanceId().ToString()
                       << " is available";
  }
#endif

  for (Int32_t i = 0; i < handles.size(); i++) {
    //ara::core::StringView instance_id_str = handles[i].GetInstanceId().ToString();
    std::cout << "    [" << i << "] instance_id = " << handles[i].GetInstanceId().ToString()
              << std::endl;

    std::stringstream instance_id_str;
    instance_id_str << handles[i].GetInstanceId().ToString();
    if (instance_id_str.str() != std::to_string(instance_id_)) {
      continue;
    }

    // ara::core::StringView instance_id_str = it.GetInstanceId().ToString();
    if (Nullptr_t == uart_rx_proxy_) {
      uart_rx_proxy_ = std::make_unique<ProxyUart>(handles[0]);
      // m_logger.LogInfo() << "Created uart proxy from handle with instance: " << instanceId;
      uart_rx_proxy_->mdcEvent.SetReceiveHandler([this]() {
        SerialDriverMdc::UartDataEventCallback();
      });
      uart_rx_proxy_->mdcEvent.Subscribe(100);
      // m_uartMethodThread = std::make_unique<std::thread>(&SerialDriverMdc18::UartSetData, this);
      break;
    }
  }
}

void SerialDriverMdc::UartDataEventCallback() {
  if (Nullptr_t == uart_rx_proxy_) {
    LOG_ERR << "Invalid UART RX proxy.";
    return;
  }

  // Lock
  receiving_uart_data_lock_.lock();

  // 接收UART通路数据
  uart_rx_proxy_->mdcEvent.GetNewSamples(
        [this](ara::com::SamplePtr<mdc::mdcuart::UartDataParam const> ptr) {
    auto sample = ptr.Get();
    if (sample->validLen > 0) {
      common::WriteToStringRingBufOverride(
            &recv_ring_buff_, (const Char_t*)&(sample->data[0]), sample->validLen);
    }

#if 0
    std::cout<<"receive raw data size == "<< std::dec <<sample->data.size()<<std::endl;
    for(int i = 0; i<sample->data.size();i++)
    {
      if(i == 0)
        std::cout<<"receive raw data is : "<<std::hex<<static_cast<int>(sample->data[i])<<"  ";
      else if(i <= (sample->data.size() - 1))
        printf(" %02X", sample->data[i]);
      else
        std::cout<<std::endl;
    }
#endif
  });

  // Unlock
  receiving_uart_data_lock_.unlock();

  // Notify
  receiving_uart_data_cond_.notify_one();
}

//void SerialDriverMdc18::UartSetData()
//{
//  unsigned int i = 0;
//  if (m_uartProxy == nullptr) {
//    m_logger.LogError() << "m_uartProxy not ready!";
//    return;
//  }

//  mdc::mdcuart::UartDataParam param;
//  param.seq = UART_MSG_TEST_SEQ;
//  param.validLen = UART_DATA_VALID_LEN;

//  for (i = 0; i < UART_DATA_MAX_LEN; i++) {
//    param.data.push_back(i);
//  }

//  while (1) {
//    param.seq++;
//    for (i = 0; i < UART_DATA_MAX_LEN; i++) {
//      param.data[i]++;
//    }
//    auto handle = m_uartProxy->UartDataSetMethod(param);
//    auto sendResult = handle.GetResult();
//    // 判断返回的是正常值还是错误码并做校验和打印
//    if (sendResult.HasValue()) {
//      // 获取值
//      auto value = sendResult.Value();
//      //            std::cout << "success send the value, reply value is: " << static_cast<int>(value.result) << std::endl;
//    } else {
//      // 获取错误码
//      auto error = sendResult.Error();
//      //            std::cout << "method send error, the error is: " << std::string(error.Message().data()) << " domain is: "
//      //                << error.Domain().Id() << " error code is " << error.Value() << std::endl;
//    }
//    std::this_thread::sleep_for(std::chrono::milliseconds(UARTSEND_METHOD_PERIOD));
//  }
//}

#else
  ERROR: Invalid MDC UART device.
#endif

#endif  // #if (ENABLE_SERIAL_DEV_MDC)


} // namespace serial_dev
} // namespace phoenix



