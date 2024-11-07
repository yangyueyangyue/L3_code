#ifndef PHOENIX_SERIAL_DEV_SERIAL_DRIVER_POSIX_H_
#define PHOENIX_SERIAL_DEV_SERIAL_DRIVER_POSIX_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "serial_dev/serial_driver.h"


namespace phoenix {
namespace serial_dev {


class SerialDriverPosix : public SerialDriver {
public:
  SerialDriverPosix();
  virtual ~SerialDriverPosix();

  virtual bool OpenPort(const SerialPortParam& param);
  virtual bool ClosePort();
  virtual Int32_t Send(const Uint8_t* data, Int32_t data_size);
  virtual Int32_t ReadWait(
      Uint8_t* data, Int32_t max_data_size = 1, Uint32_t timeout_ms = 50);

private:
  Int32_t fd_serial_port_;
};


} // namespace serial_dev
} // namespace phoenix


#endif  // PHOENIX_SERIAL_DEV_SERIAL_DRIVER_POSIX_H_
