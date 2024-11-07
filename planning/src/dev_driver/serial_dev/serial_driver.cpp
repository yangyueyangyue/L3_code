//
#include "serial_dev/serial_driver.h"
#include "utils/log.h"

namespace phoenix {
namespace serial_dev {


SerialDriver::SerialDriver() {
}

SerialDriver::~SerialDriver() {
}

bool SerialDriver::OpenPort(const SerialPortParam& param) {
  LOG_WARN << "You are accessing the base class now.";

  return false;
}

bool SerialDriver::ClosePort() {
  LOG_WARN << "You are accessing the base class now.";

  return false;
}

Int32_t SerialDriver::Send(const Uint8_t* data, Int32_t data_size) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}

Int32_t SerialDriver::ReadWait(
    Uint8_t* data, Int32_t max_data_size, Uint32_t timeout_ms) {
  LOG_WARN << "You are accessing the base class now.";

  return (-1);
}


} // namespace serial_dev
} // namespace phoenix
