//
#include "sensor/imu/mpsk/task_recv_gnss_data_mpsk.h"

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


#define ENABLE_RECV_GNSS_MPSK_TRACE (0)


namespace phoenix {
namespace sensor {
namespace imu {
namespace mpsk {


static inline void DecodeInt16Value(const void* buf, Int32_t offset, Int16_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0])) |
      (((Uint32_t) buffer[offset + 1]) << 8);
}

static inline void DecodeUint16Value(const void* buf, Int32_t offset, Uint16_t* p) {
  DecodeInt16Value(buf, offset, (Int16_t*)p);
}

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


TaskRecvGnssDataMpsk::TaskRecvGnssDataMpsk(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_GNSS_DATA, "Recv Gnss Data", manager) {
  running_flag_recv_gnss_ = false;

  serial_dev_ = Nullptr_t;
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

TaskRecvGnssDataMpsk::~TaskRecvGnssDataMpsk() {
  Stop();

  if (Nullptr_t != serial_dev_) {
    delete serial_dev_;
  }
}

bool TaskRecvGnssDataMpsk::Start() {
  if (Nullptr_t == serial_dev_) {
    LOG_ERR << "Invalid serial device.";
    return false;
  }

  if (running_flag_recv_gnss_) {
    return (true);
  }

  serial_dev::SerialPortParam param;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  param.port_name[0] = 0;
#else
  //com_snprintf(param.port_name, 64, "/dev/ttyS0");
  com_snprintf(param.port_name, 64, "/dev/ttyUSB0");
  param.data_bits = serial_dev::SerialPortParam::DATA_BITS_8;
  param.parity = serial_dev::SerialPortParam::PARITY_NO;
  param.stop_bits = serial_dev::SerialPortParam::STOP_BITS_ONE;
  param.baud_rate = serial_dev::SerialPortParam::BAUD_RATE_115200;
#endif
  if (!serial_dev_->OpenPort(param)) {
    LOG_ERR << "Failed to open serial device.";
    return false;
  }

  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));

  LOG_INFO(3) << "Create thread of receiving gnss data...";
  running_flag_recv_gnss_ = true;
  thread_recv_gnss_ = boost::thread(boost::bind(&TaskRecvGnssDataMpsk::ThreadReceivingGnss, this));

  LOG_INFO(3) << "Create thread of receiving gnss data... [OK]";

  return (true);
}

bool TaskRecvGnssDataMpsk::Stop() {
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

void TaskRecvGnssDataMpsk::ThreadReceivingGnss() {
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

void TaskRecvGnssDataMpsk::SpinSerialData(
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
      if (0x55 == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (2 == data_length_) {
      data_buff_[data_length_] = buffer[i];
      data_length_++;
    } else {
      /* message length, not including header or CRC*/
      msg_len = data_buff_[2];

      /* header:12, msg:variable, crc:4*/
      Int32_t msg_frame_len = 3 + msg_len + 1;
      // Int32_t msg_frame_len = 3 + msg_len;
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

void TaskRecvGnssDataMpsk::ParseMessageFrame(
    const Uint8_t* buffer, Int32_t length) {
#if (ENABLE_RECV_GNSS_MPSK_TRACE)
  printf("Receiving gnss:");
  for (Int32_t i = 0; i < length; ++i) {
    printf(" %02X", buffer[i]);
  }
  printf("\n");
#endif

  // message header
  Int32_t msg_len = buffer[2];
  Int32_t mode = buffer[3];
  Int32_t week_number = buffer[4] | (buffer[5] << 8);
  Uint32_t milliseconds = buffer[6] | (buffer[7] << 8) | (buffer[8] << 16) | (buffer[9] << 24);

  Int32_t index = 10;

  /// GNSS part
  // 观测卫星个数N
  Uint32_t star_num = buffer[index];
  index += 1;

  // 纬度/deg
  Float64_t latitude = 0;
  DecodeFloat64Value(buffer, index, &latitude);
  index += 8;

  // 经度/deg
  Float64_t longitude = 0;
  DecodeFloat64Value(buffer, index, &longitude);
  index += 8;

  // 高程/m
  Float32_t altitude = 0;
  DecodeFloat32Value(buffer, index, &altitude);
  index += 4;

  // 北向速度/m/s  /\  前向速度 / m/s
  Float32_t v_n = 0;
  DecodeFloat32Value(buffer, index, &v_n);
  index += 4;

  // 东向速度/m/s  /\  右向速度 / m/s
  Float32_t v_e = 0;
  DecodeFloat32Value(buffer, index, &v_e);
  index += 4;

  // 地向速度/m/s  /\  下向速度 / m/s
  Float32_t v_u = 0;
  DecodeFloat32Value(buffer, index, &v_u);
  index += 4;

  // 横滚/deg
  Float32_t roll = 0;
  DecodeFloat32Value(buffer, index, &roll);
  index += 4;

  // 俯仰/deg
  Float32_t pitch = 0;
  DecodeFloat32Value(buffer, index, &pitch);
  index += 4;

  // 航向/deg
  Float32_t heading = 0;
  DecodeFloat32Value(buffer, index, &heading);
  index += 4;


  /// IMU part
  // index = 47;
  // std::cout<<"index =11= "<<index<<std::endl;
  Int16_t int16_value = 0;

  // 北向加速度/m/s^2，10^-3 ? 10^2  /\  载体前向加速度 / m/s^2, 10^-2
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t acc_x = int16_value * 0.001F;
  Float32_t acc_x = int16_value * 0.01F;
  index += 2;

  // 东向加速度/m/s^2，10^-3 ? 10^-2 /\ 载体右向加速度 / m/s^2, 10^-3
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t acc_y = int16_value * 0.001F;
  Float32_t acc_y = int16_value * 0.01F;
  index += 2;

  // 地向加速度/m/s^2，10^-3 ? 10^-2 /\ 载体地向加速度 / m/s^2, 10^-2
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t acc_z = int16_value * 0.001F;
  Float32_t acc_z = int16_value * 0.01F;
  index += 2;

  // 横滚角速度/deg/s，10^-3 ? 10^-2
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t gryo_x = int16_value * 0.001F;
  Float32_t gryo_x = int16_value * 0.01F;
  index += 2;

  // 俯仰角速度/deg/s，10^-3 ? 10^-2
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t gryo_y = int16_value * 0.001F;
  Float32_t gryo_y = int16_value * 0.01F;
  index += 2;

  // 航向角速度/deg/s，10^-3 ? 10^-2
  DecodeInt16Value(buffer, index, &int16_value);
  // Float32_t gryo_z = int16_value * 0.001F;
  Float32_t gryo_z = -int16_value * 0.01F;
  index += 2;


  /// GNSS Status
  // Flag : 高2位
  //   0-机械编排；1-保持整秒时刻状态；2-组合更新计算开始；3-组合解算完成
  // N : 低6位，
  //   LC时，表示RTK状态，0-SPP，1-float，2-fixed，3-NG
  // Int32_t status = buffer[59] & 0x3F;
  Int32_t status = buffer[index] & 0x3F;
  index += 1;
#if (ENABLE_RECV_GNSS_MPSK_TRACE)
  printf("### status=%d\n", status);
#endif


  /// Send IMU message
  ad_msg::Imu imu;
  // 航向角的角速度
  imu.yaw_rate = common::com_deg2rad(gryo_z);
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

  if (com_isnan(imu.yaw_rate) || com_isinf(imu.yaw_rate)) {
    LOG_ERR << "Detected invalid imu value.";
    return;
  }

  // Notify
  framework::MessageRecvImuData imu_message(&imu);
  Notify(imu_message);

#if (ENABLE_RECV_GNSS_MPSK_TRACE)
  LOG_INFO(3)<<"****************IMU DATA**************************";
  LOG_INFO(3)<<"imu.accel_x:"<<imu.accel_x;
  LOG_INFO(3)<<"imu.accel_y:"<<imu.accel_y;
  LOG_INFO(3)<<"imu.accel_z:"<<imu.accel_z;
  LOG_INFO(3)<<"imu.roll_rate:"<<imu.roll_rate;
  LOG_INFO(3)<<"imu.pitch_rate:"<<imu.pitch_rate;
  LOG_INFO(3)<<"imu.yaw_rate:"<<imu.yaw_rate;
#endif


  /// Send GNSS message
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

  common::Vec2d unit_direction[2];
  unit_direction[0].set_x(common::com_cos(gnss.heading_utm));
  unit_direction[0].set_y(common::com_sin(gnss.heading_utm));
  unit_direction[1].set_x(-unit_direction[0].y());
  unit_direction[1].set_y(unit_direction[0].x());

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  phoenix::common::Vec2d offset_lon = 0.5F * unit_direction[0];
  phoenix::common::Vec2d offset_lat = 0.0F * unit_direction[1];
#else
  phoenix::common::Vec2d offset_lon = 0.0F * unit_direction[0];
  phoenix::common::Vec2d offset_lat = 0.0F * unit_direction[1];
#endif
  gnss.x_utm += offset_lon.x() + offset_lat.x();
  gnss.y_utm += offset_lon.y() + offset_lat.y();

  if (0x00 == status) {
    // SPP, single point position, 单点定位  （误差：米级）
    gnss.utm_status = ad_msg::Gnss::STATUS_BAD;
  } else if (0x01 == status) {
    // RTK float  （误差：10 cm ~ 20 cm）
    gnss.utm_status = ad_msg::Gnss::STATUS_CONVERGING;
  } else if (0x02 == status) {
    // RTK fixed  （误差：< 5 cm）
    gnss.utm_status = ad_msg::Gnss::STATUS_GOOD;
  } else if (0x03 == status) {
    // No GNSS    （误差：未知）
    gnss.utm_status = ad_msg::Gnss::STATUS_BAD;
  } else if (0x04 == status) {
    // 车辆静止且没有RTK    (误差：100 cm ~ 200 cm)
    gnss.utm_status = ad_msg::Gnss::STATUS_BAD;
  } else {
    // 其它状态    （误差：未知）
    gnss.utm_status = ad_msg::Gnss::STATUS_BAD;
  }
  gnss.gnss_status = gnss.utm_status;

  if (com_isnan(gnss.latitude) || com_isinf(gnss.latitude) ||
      com_isnan(gnss.longitude) || com_isinf(gnss.longitude) ||
      com_isnan(gnss.heading_gnss) || com_isinf(gnss.heading_gnss)) {
    LOG_ERR << "Detected invalid gnss value.";
    return;
  }

  if (com_isnan(gnss.x_utm) || com_isinf(gnss.x_utm) ||
      com_isnan(gnss.y_utm) || com_isinf(gnss.y_utm) ||
      com_isnan(gnss.heading_utm) || com_isinf(gnss.heading_utm)) {
    LOG_ERR << "Detected invalid utm value.";
    return;
  }

  // Notify
  framework::MessageRecvGnssData gnss_message(&gnss);
  Notify(gnss_message);

#if (ENABLE_RECV_GNSS_MPSK_TRACE)
  LOG_INFO(3)<<"****************GNSS DATA**************************";
  LOG_INFO(3)<<"gnss.gnss_status:"<<gnss.utm_status;
  LOG_INFO(3)<<"gnss.latitude:"<<gnss.latitude;
  LOG_INFO(3)<<"gnss.longitude:"<<gnss.longitude;
  LOG_INFO(3)<<"gnss.altitude:"<<gnss.altitude;
  LOG_INFO(3)<<"gnss.heading_gnss:"<<gnss.heading_gnss;
  LOG_INFO(3)<<"gnss.x_utm:"<<gnss.x_utm;
  LOG_INFO(3)<<"gnss.y_utm:"<<gnss.y_utm;
  LOG_INFO(3)<<"gnss.z_utm:"<<gnss.z_utm;
  LOG_INFO(3)<<"gnss.heading_utm:"<<gnss.heading_utm;
  LOG_INFO(3)<<"gnss.pitch:"<<gnss.pitch;
  LOG_INFO(3)<<"gnss.roll:"<<gnss.roll;
  LOG_INFO(3)<<"gnss.v_e:"<<gnss.v_e;
  LOG_INFO(3)<<"gnss.v_n:"<<gnss.v_n;
  LOG_INFO(3)<<"gnss.v_u:"<<gnss.v_u;
  LOG_INFO(3)<<"gnss.v_x_utm:"<<gnss.v_x_utm;
  LOG_INFO(3)<<"gnss.v_y_utm:"<<gnss.v_y_utm;
  LOG_INFO(3)<<"gnss.v_z_utm:"<<gnss.v_z_utm;
#endif
}


}  // namespace mpsk
}  // namespace imu
} // namespace sensor
} // namespace phoenix
