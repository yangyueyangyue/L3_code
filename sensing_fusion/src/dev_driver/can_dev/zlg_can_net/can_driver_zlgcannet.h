#ifndef PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_H_
#define PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_H_


#include "utils/macros.h"
#include "can_dev/can_driver.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_ZLGCANNET)

class CanDriverZlgCanNet : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverZlgCanNet();
  ~CanDriverZlgCanNet();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  static bool device_is_open_;

  bool channel_is_open_;
  Int32_t sockfd_notifying_;
  Int32_t sockfd_working_;
};

#endif  // #if (ENABLE_CAN_DEV_ZLGCANNET)


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_ZLG_CAN_NET_CAN_DRIVER_ZLGCANNET_H_
