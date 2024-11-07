#ifndef PHOENIX_CAN_DEV_KAVASER_CAN_DRIVER_KVASER_H_
#define PHOENIX_CAN_DEV_KAVASER_CAN_DRIVER_KVASER_H_


#include "utils/macros.h"
#include "can_dev/can_driver.h"
#if (ENABLE_CAN_DEV_KVASER)
#include "canlib.h"
#endif


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_KVASER)

class CanDriverKvaser : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverKvaser();
  ~CanDriverKvaser();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);

private:
  bool CreateCanHandle(const CanChannelParam& param, canHandle& handle);
  bool DestroyCanHandle(canHandle& handle);

private:
  static bool device_is_open_;

  canHandle handle_read_;
  canHandle handle_write_;
};

#endif  // #if (ENABLE_CAN_DEV_KVASER)


}  // namespace can_dev
}  // namespace phoenix

#endif // PHOENIX_CAN_DEV_KAVASER_CAN_DRIVER_KVASER_H_
