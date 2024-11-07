#ifndef PHOENIX_CAN_DEV_EMUC2_CAN_DRIVER_EMUC2_H_
#define PHOENIX_CAN_DEV_EMUC2_CAN_DRIVER_EMUC2_H_


#include "utils/macros.h"
#include "can_dev/can_driver.h"


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_EMUC2)

class CanDriverEmuc2 : public CanDriver {
public:
  static bool OpenDevice();
  static bool CloseDevice();

  CanDriverEmuc2();
  ~CanDriverEmuc2();

  bool OpenChannel(const CanChannelParam& param);
  bool CloseChannel();
  Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout = 50);

private:
  static bool device_is_open_;
  static bool channel_is_open_;

  static int device_port_;

  int channel_ = 0;
};

#endif  // #if (ENABLE_CAN_DEV_EMUC2)


}  // namespace can_dev
}  // namespace phoenix

#endif  // PHOENIX_CAN_DEV_EMUC2_CAN_DRIVER_EMUC2_H_
