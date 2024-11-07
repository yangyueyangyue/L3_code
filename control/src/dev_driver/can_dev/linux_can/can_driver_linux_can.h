#ifndef PHOENIX_CAN_DEV_LINUX_CAN_CAN_DRIVER_LINUX_CAN_H_
#define PHOENIX_CAN_DEV_LINUX_CAN_CAN_DRIVER_LINUX_CAN_H_


#include "utils/macros.h"
#include "can_dev/can_driver.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_LINUX_CAN)

class CanDriverLinuxCan : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverLinuxCan();
  ~CanDriverLinuxCan();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  static bool device_is_open_;

  bool channel_is_open_;
  Int32_t sockfd_can_;
};

#endif  // #if (ENABLE_CAN_DEV_LINUX_CAN)


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_LINUX_CAN_CAN_DRIVER_LINUX_CAN_H_
