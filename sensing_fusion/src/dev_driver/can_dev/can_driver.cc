//
#include "can_dev/can_driver.h"
#include "utils/log.h"


namespace phoenix {
namespace can_dev {


CanDriver::CanDriver() {
}

CanDriver::~CanDriver() {
}


bool CanDriver::OpenChannel(const CanChannelParam& param) {
  LOG_WARN << "You are accessing the base class now.";

  return false;
}

bool CanDriver::CloseChannel() {
  LOG_WARN << "You are accessing the base class now.";

  return false;
}


Int32_t CanDriver::Send(const CanFrame* frame, Int32_t frame_num) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}

Int32_t CanDriver::SendCanFd(const CanFdFrame* frame, Int32_t frame_num) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}


Int32_t CanDriver::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}

Int32_t CanDriver::ReadCanFdWait(
    CanFdFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}


}  // namespace can_dev
}  // namespace phoenix


