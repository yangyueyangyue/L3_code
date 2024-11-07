#ifndef PHOENIX_CAN_DEV_MDC_CAN_CAN_DRIVER_MDC_H_
#define PHOENIX_CAN_DEV_MDC_CAN_CAN_DRIVER_MDC_H_


#include "utils/macros.h"
#include "container/ring_buffer.h"
#include "can_dev/can_driver.h"

#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <iostream>

#define MDC_CAN_DEV_INVALID (0)
#define MDC_CAN_DEV_16 (1)
#define MDC_CAN_DEV_18 (2)
#define MDC_CAN_DEV MDC_CAN_DEV_18

#if (ENABLE_CAN_DEV_MDC)
#if (MDC_CAN_DEV == MDC_CAN_DEV_16)
#include "mdc/sensor/cantxserviceinterface_skeleton.h"
#include "mdc/sensor/canrxserviceinterface_common.h"
#include "mdc/sensor/cantxserviceinterface_common.h"
#include "mdc/sensor/canrxserviceinterface_proxy.h"
#include "impl_type_cansetdataresult.h"
#include "impl_type_canbusdataparam.h"
#elif (MDC_CAN_DEV == MDC_CAN_DEV_18)
#include "mdc/mdccanrx/canrxserviceinterface_common.h"
#include "mdc/mdccanrx/canrxserviceinterface_proxy.h"
#include "mdc/mdccan/impl_type_canfdsetdataresult.h"
#include "mdc/mdccan/impl_type_canfdbusdataparaml.h"
#include "mdc/mdccan/impl_type_canfdbusdataparams.h"
#include "ara/log/logging.h"
#else
  ERROR: Invalid MDC CAN device.
#endif
#endif // #if (ENABLE_CAN_DEV_MDC)


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_MDC)


#if (MDC_CAN_DEV == MDC_CAN_DEV_16)

class CanDriverMdc : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverMdc();
  ~CanDriverMdc();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  using CanRxProxy = mdc::sensor::proxy::CanRxServiceInterfaceProxy;
  using CanTxSkeleton = mdc::sensor::skeleton::CanTxServiceInterfaceSkeleton;
  // 服务发现回调
  void ServiceAvailabilityCallback(
      ara::com::ServiceHandleContainer<CanRxProxy::HandleType> handles,
      ara::com::FindServiceHandle handler);
  // 接收数据回调
  void CanDataEventCallback(Uint8_t channelID);

private:
  enum { MAX_CHANNEL_NUM = 12 };
  static bool device_is_open_;

  bool channel_is_open_;

  boost::mutex receiving_lock_;
  boost::mutex receiving_can_frames_lock_;
  boost::mutex sending_can_frames_lock_;
  boost::condition_variable receiving_can_frames_cond_;
  common::RingBuffer<CanFrame, 200> received_can_frames_;

  Int32_t channel_id_;
  Int32_t instance_id_;
  std::unique_ptr<CanRxProxy> can_rx_proxy_;
  std::unique_ptr<CanTxSkeleton> can_tx_skeleton_;
};

#elif (MDC_CAN_DEV == MDC_CAN_DEV_18)

class CanDriverMdc : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverMdc();
  ~CanDriverMdc();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
        CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  using CanFdRxProxy = mdc::mdccanrx::proxy::CanRxServiceInterfaceProxy;

  // 服务发现回调
  void ServiceAvailabilityCallback(
        ara::com::ServiceHandleContainer<CanFdRxProxy::HandleType> handles,
        ara::com::FindServiceHandle handler);
  // canFd 短包接收数据回调
  void CanFdDataSEventCallback(uint8_t channelId);
  // canFd 长包接收数据回调
  void CanFdDataLEventCallback(uint8_t channelId);

private:
  enum { MAX_CHANNEL_NUM = 12 };
  static bool device_is_open_;

  bool channel_is_open_;

  boost::mutex receiving_lock_;
  boost::mutex receiving_can_frames_lock_;
  boost::mutex sending_can_frames_lock_;
  boost::condition_variable receiving_can_frames_cond_;
  common::RingBuffer<CanFrame, 200> received_can_frames_;
  Int32_t channel_id_;
  Int32_t instance_id_;
  // 客户端接收
  std::unique_ptr<CanFdRxProxy> can_rx_proxy_;
};

#else
  ERROR: Invalid MDC CAN device.
#endif

#endif  // #if (ENABLE_CAN_DEV_MDC)


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_MDC_CAN_CAN_DRIVER_MDC_H_
