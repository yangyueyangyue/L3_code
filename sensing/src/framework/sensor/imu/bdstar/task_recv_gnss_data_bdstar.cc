//
#include "sensor/imu/bdstar/task_recv_gnss_data_bdstar.h"

#include <iostream>

#include "utils/com_utils.h"
#include "utils/log.h"
#include "utils/gps_tools.h"

#include "communication/shared_data.h"
#if (ENABLE_SERIAL_DEV_POSIX)
#include "serial_dev/posix/serial_driver_posix.h"
#endif
#if (ENABLE_SERIAL_DEV_MDC)
#include "serial_dev/mdc/serial_driver_mdc.h"
#endif


#define ENABLE_RECV_GNSS_BDSTAR_TRACE (0)


namespace phoenix {
namespace sensor {
namespace imu {
namespace bdstar {


static inline void DecodeInt32Value(const void* buf, Int32_t offset, Int32_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0])) |
      (((Uint32_t) buffer[offset + 1]) << 8) |
      (((Uint32_t) buffer[offset + 2]) << 16) |
      (((Uint32_t) buffer[offset + 3]) << 24);
}

static inline void DecodeUint32Value(const void* buf, Int32_t offset, Uint32_t* p) {
  DecodeInt32Value(buf, offset, (Int32_t*)p);
}

static inline void DecodeInt64Value(const void* buf, Int32_t offset, Int64_t* p) {
  const Uint8_t* buffer = (const Uint8_t *) buf;
  Int32_t pos = offset;

  Uint64_t a =
      (((Uint32_t) buffer[pos + 0])) |
      (((Uint32_t) buffer[pos + 1]) << 8) |
      (((Uint32_t) buffer[pos + 2]) << 16) |
      (((Uint32_t) buffer[pos + 3]) << 24);
  pos += 4;
  Uint64_t b =
      (((Uint32_t) buffer[pos + 0])) |
      (((Uint32_t) buffer[pos + 1]) << 8) |
      (((Uint32_t) buffer[pos + 2]) << 16) |
      (((Uint32_t) buffer[pos + 3]) << 24);
  pos += 4;
  p[0] = (b << 32) + (a & 0xffffffff);
}

static inline void DecodeUint64Value(const void* buf, Int32_t offset, Uint64_t* p) {
  DecodeInt64Value(buf, offset, (Int64_t*)p);
}

static inline void DecodeFloat32Value(const void* buf, Int32_t offset, Float32_t* p) {
  DecodeInt32Value(buf, offset, (Int32_t*)p);
}

static inline void DecodeFloat64Value(const void* buf, Int32_t offset, Float64_t* p) {
  DecodeInt64Value(buf, offset, (Int64_t*)p);
}

#define CRC32_POLYNOMIAL 0xEDB88320L
// Calculate a CRC value to be used by CRC calculation functions.
Uint32_t CRC32Value(Int32_t i) {
  Int32_t j = 0;
  Uint32_t crc = i;
  for (j = 8 ; j > 0; j--) {
    if (crc & 1) {
      crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

// Calculates the CRC-32 of a block of data all at once
Uint32_t CalculateBlockCRC32(
    Uint32_t count, /* Number of bytes in the data block */
    const Uint8_t *buffer ) /* Data block */ {
  Uint32_t temp1;
  Uint32_t temp2;
  Uint32_t crc = 0;
  while ((count--) != 0) {
    temp1 = ( crc >> 8 ) & 0x00FFFFFFL;
    temp2 = CRC32Value( ((Int32_t) crc ^ *buffer++ ) & 0xff );
    crc = temp1 ^ temp2;
  }
  return (crc);
}


TaskRecvGnssDataBdstar::TaskRecvGnssDataBdstar(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_GNSS_DATA, "Recv Gnss Data", manager) {
  running_flag_recv_gnss_ = false;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  #if (ENABLE_SERIAL_DEV_MDC)
    serial_dev_ = new serial_dev::SerialDriverMdc();
  #endif
#else
  #if (ENABLE_SERIAL_DEV_POSIX)
    serial_dev_ = new serial_dev::SerialDriverPosix();
  #endif
#endif

  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));
}

TaskRecvGnssDataBdstar::~TaskRecvGnssDataBdstar() {
  Stop();

  if (Nullptr_t != serial_dev_) {
    delete serial_dev_;
  }
}

bool TaskRecvGnssDataBdstar::Start() {
  if (Nullptr_t == serial_dev_) {
    LOG_ERR << "Invalid serial device.";
    return false;
  }

  if (running_flag_recv_gnss_) {
    return (true);
  }

  serial_dev::SerialPortParam param;
  //com_snprintf(param.port_name, 64, "/dev/ttyS0");
  com_snprintf(param.port_name, 64, "/dev/ttyUSB0");
  param.data_bits = serial_dev::SerialPortParam::DATA_BITS_8;
  param.parity = serial_dev::SerialPortParam::PARITY_NO;
  param.stop_bits = serial_dev::SerialPortParam::STOP_BITS_ONE;
  param.baud_rate = serial_dev::SerialPortParam::BAUD_RATE_115200;
  if (!serial_dev_->OpenPort(param)) {
    LOG_ERR << "Failed to open serial device.";
    return false;
  }

  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));

  LOG_INFO(3) << "Create thread of receiving gnss data...";
  running_flag_recv_gnss_ = true;
  thread_recv_gnss_ = boost::thread(boost::bind(&TaskRecvGnssDataBdstar::ThreadReceivingGnss, this));

  LOG_INFO(3) << "Create thread of receiving gnss data... [OK]";

  return (true);
}

bool TaskRecvGnssDataBdstar::Stop() {
  if (running_flag_recv_gnss_) {
    running_flag_recv_gnss_ = false;

    LOG_INFO(3) << "Stop thread of receiving gnss data ...";
    bool ret = thread_recv_gnss_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving gnss data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving gnss data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving gnss data ... [NG]";
    }

    LOG_INFO(3) << "Close serial port.";
    if (Nullptr_t != serial_dev_) {
      serial_dev_->ClosePort();
    }
  }

  return (true);
}

void TaskRecvGnssDataBdstar::ThreadReceivingGnss() {
  LOG_INFO(3) << "Thread of receiving gnss data ... [Started]";

  Uint8_t data_buff[512];
  while (running_flag_recv_gnss_) {
    Int32_t bytes = serial_dev_->ReadWait(data_buff, 510);
    if (bytes > 0) {
      SpinSerialData(&data_buff[0], bytes);
    }
  }

  LOG_INFO(3) << "Thread of receiving gnss data ... [Stopped]";
}

void TaskRecvGnssDataBdstar::SpinSerialData(
    const Uint8_t* buffer, Int32_t length) {
  if (length < 1) {
    return;
  }

  Int32_t msg_len = 0;
  for (Int32_t i = 0; i < length; ++i) {
    if (0 == data_length_) {
      /* Synch */
      if (0xAA == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (1 == data_length_) {
      /* Synch */
      if (0x44 == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (2 == data_length_) {\
      /* Synch */
      if (0x13 == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (3 == data_length_) {
      /* message length, not including header or CRC*/
      if ((88 == buffer[i] /*inspvas*/) || (40 == buffer[i] /*rawimus*/)) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
        LOG_ERR << "Unknown message or invalid message length.";
      }
    } else {
      /* message length, not including header or CRC*/
      msg_len = data_buff_[3];

      if (data_length_ < 12 /* message header length*/) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        /* header:12, msg:variable, crc:4*/
        Int32_t msg_frame_len = 12 + msg_len + 4;
        if (data_length_ < (msg_frame_len-1)) {
          data_buff_[data_length_] = buffer[i];
          data_length_++;
        } else if (data_length_ == (msg_frame_len-1)) {
          data_buff_[data_length_] = buffer[i];
          data_length_++;

          // complete message frame
          ParseMessageFrame(data_buff_, data_length_);

          data_length_ = 0;
        } else {
          //
          LOG_ERR << "Unexpected case occurs.";

          data_length_ = 0;
        }
      }
    }
  }
}

void TaskRecvGnssDataBdstar::ParseMessageFrame(
    const Uint8_t* buffer, Int32_t length) {
  const static Float64_t acc_sf = (0.2 / 65535) * 0.001;
  const static Float64_t g_sf = (0.008 / 65535);

#if (ENABLE_RECV_GNSS_BDSTAR_TRACE)
  printf("Receiving gnss:");
  for (Int32_t i = 0; i < length; ++i) {
    printf(" %02X", buffer[i]);
  }
  printf("\n");
#endif

  // message header
  Int32_t msg_len = buffer[3];
  Int32_t msg_id = buffer[4] | (buffer[5] << 8);
  Int32_t week_number = buffer[6] | (buffer[7] << 8);
  Uint32_t milliseconds = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16) | (buffer[11] << 24);

  Int32_t index = 12;

  Uint32_t crc_recv = 0;
  DecodeUint32Value(buffer, length-4, &crc_recv);
  Uint32_t crc_check = CalculateBlockCRC32(length-4, buffer);
  if (crc_recv != crc_check) {
    LOG_ERR << "Failed to check CRC value, crc_recv=" << crc_recv
            << ", crc_check=" << crc_check << ".";
    return;
  }

#if (ENABLE_RECV_GNSS_BDSTAR_TRACE)
  printf("crc_recv=0x%08X, crc_check=0x%08X\n", crc_recv, crc_check);
#endif

  // Parse message
  if (325 == msg_id) {
    Uint32_t week = 0;
    DecodeUint32Value(buffer, index, &week);
    index += 4;

    Float64_t seconds = 0;
    DecodeFloat64Value(buffer, index, &seconds);
    index += 8;

    Uint32_t status = 0;
    DecodeUint32Value(buffer, index, &status);
    index += 4;

    Int32_t int32_value = 0;

    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t acc_z = int32_value * acc_sf;
    index += 4;

    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t acc_y = int32_value * acc_sf;
    index += 4;

    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t acc_x = int32_value * acc_sf;
    index += 4;

    // deg/s
    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t gryo_z = int32_value * g_sf;
    index += 4;

    // deg/s
    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t gryo_y = int32_value * g_sf;
    index += 4;

    // deg/s
    DecodeInt32Value(buffer, index, &int32_value);
    Float32_t gryo_x = int32_value * g_sf;
    index += 4;

    ad_msg::Imu imu;
    // 航向角的角速度
    imu.yaw_rate = -common::com_deg2rad(gryo_z);
    // 俯仰角的角速度
    imu.pitch_rate = common::com_deg2rad(gryo_x);
    // 横滚角的角速度
    imu.roll_rate = common::com_deg2rad(gryo_y);
    // 沿着车身x轴的加速度
    imu.accel_x = acc_y;
    // 沿着车身y轴的加速度
    imu.accel_y = acc_x;
    // 沿着车身z轴的加速度
    imu.accel_z = acc_z;
    // Notify
    framework::MessageRecvImuData imu_message(&imu);
    Notify(imu_message);

#if (ENABLE_RECV_GNSS_BDSTAR_TRACE)
    printf("IMU: week=%u, seconds=%f, status=0x%04x"
           ", acc_z=%f, acc_y=%f, acc_x=%f"
           ", gryo_z=%f, gryo_y=%f, gryo_x=%f"
           "\n",
           week, seconds, status,
           acc_z, acc_y, acc_x,
           gryo_z, gryo_y, gryo_x);
#endif
  } else if (508 == msg_id) {
    // GPS Time：GPS week number
    Uint32_t week = 0;
    DecodeUint32Value(buffer, index, &week);
    index += 4;

    Float64_t seconds = 0;
    DecodeFloat64Value(buffer, index, &seconds);
    index += 8;

    // 纬度 / deg
    Float64_t latitude = 0;
    DecodeFloat64Value(buffer, index, &latitude);
    index += 8;

    // 经度 / deg
    Float64_t longitude = 0;
    DecodeFloat64Value(buffer, index, &longitude);
    index += 8;

    // 高度 / m
    Float64_t altitude = 0;
    DecodeFloat64Value(buffer, index, &altitude);
    index += 8;

    // 北向速度 / m/s
    Float64_t v_n = 0;
    DecodeFloat64Value(buffer, index, &v_n);
    index += 8;

    // 东向速度 / m/s
    Float64_t v_e = 0;
    DecodeFloat64Value(buffer, index, &v_e);
    index += 8;

    // 地向速度 / m/s
    Float64_t v_u = 0;
    DecodeFloat64Value(buffer, index, &v_u);
    index += 8;

    // 横滚 / deg
    Float64_t roll = 0;
    DecodeFloat64Value(buffer, index, &roll);
    index += 8;

    // 俯仰 / deg
    Float64_t pitch = 0;
    DecodeFloat64Value(buffer, index, &pitch);
    index += 8;

    // 航向 / deg    （0～359.99）
    Float64_t heading = 0;
    DecodeFloat64Value(buffer, index, &heading);
    index += 8;

    Int32_t status = 0;
    DecodeInt32Value(buffer, index, &status);

    ad_msg::Gnss gnss;
    gnss.latitude = latitude;
    gnss.longitude = longitude;
    gnss.altitude = altitude;
    gnss.heading_gnss = heading;
    Float64_t heading_utm = 0.0;
    Float64_t utm_northing = 0.0;
    Float64_t utm_easting = 0.0;
    Float64_t yaw_offset_deg = 0.0;  //没用到
    Float64_t converge_angle = common::LLtoUTM(
          latitude, longitude, &utm_northing, &utm_easting);
    //调整为逆时针旋转
    heading_utm = 90.0 - heading + yaw_offset_deg + converge_angle;
    if (heading_utm < 0) {
      heading_utm += 360.0;
    }
    gnss.x_utm = utm_easting;
    gnss.y_utm = utm_northing;
    gnss.z_utm = altitude;
    gnss.heading_utm = common::com_deg2rad(heading_utm);
    gnss.pitch = common::com_deg2rad(pitch);
    gnss.roll = common::com_deg2rad(roll);
    gnss.v_e = v_e;
    gnss.v_n = v_n;
    gnss.v_u = v_u;
    gnss.v_x_utm = v_e;
    gnss.v_y_utm = v_n;
    gnss.v_z_utm = v_u;
    if (3 == status) {
      // 1 代表信号极佳
      gnss.gnss_status = ad_msg::Gnss::STATUS_GOOD;
    } else if (0 == status) {
      // 0 代表当前定位不可用
      gnss.gnss_status = ad_msg::Gnss::STATUS_BAD;
    } else {
      // 2 代表信号较差
      gnss.gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
    }
    gnss.utm_status = gnss.gnss_status;
    gnss.odom_status = ad_msg::Gnss::STATUS_INVALID;
    // Notify
    framework::MessageRecvGnssData gnss_message(&gnss);
    Notify(gnss_message);

#if (ENABLE_RECV_GNSS_BDSTAR_TRACE)
    printf("GNSS: week=%u, seconds=%f, status=0x%04x"
           ", latitude=%f, longitude=%f, altitude=%f"
           ", v_n=%f, v_e=%f, v_u=%f"
           ", roll=%f, pitch=%f, heading=%f"
           "\n",
           week, seconds, status,
           latitude, longitude, altitude,
           v_n, v_e, v_u,
           roll, pitch, heading);
#endif
  }
}


}  // namespace bdstar
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix
