//
#include "sensor/imu/intertial_lab/task_recv_gnss_intertial_lab.h"

#include "utils/com_utils.h"
#include "utils/log.h"
#include "utils/gps_tools.h"

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
#endif

#define ENABLE_ESR_MSG_TRACE (0)


namespace phoenix {
namespace sensor {
namespace imu {
namespace intertial_lab {


static inline void DecodeInt16Value(const void* buf, Int32_t offset, Int16_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0]) << 8) |
      (((Uint32_t) buffer[offset + 1]));
}

static inline void DecodeUint16Value(const void* buf, Int32_t offset, Uint16_t* p) {
  DecodeInt16Value(buf, offset, (Int16_t*)p);
}

static inline void DecodeInt32Value(const void* buf, Int32_t offset, Int32_t* p) {
  const Uint8_t* buffer = (const Uint8_t*) buf;

  p[0] =
      (((Uint32_t) buffer[offset + 0]) << 24) |
      (((Uint32_t) buffer[offset + 1]) << 16) |
      (((Uint32_t) buffer[offset + 2]) << 8) |
      (((Uint32_t) buffer[offset + 3]));
}

static inline void DecodeUint32Value(const void* buf, Int32_t offset, Uint32_t* p) {
  DecodeInt32Value(buf, offset, (Int32_t*)p);
}

static inline void DecodeInt64Value(const void* buf, Int32_t offset, Int64_t* p) {
  const Uint8_t* buffer = (const Uint8_t *) buf;
  Int32_t pos = offset;

  Uint64_t a =
      (((Uint32_t) buffer[pos + 0]) << 24) |
      (((Uint32_t) buffer[pos + 1]) << 16) |
      (((Uint32_t) buffer[pos + 2]) << 8) |
      (((Uint32_t) buffer[pos + 3]));
  pos += 4;
  Uint64_t b =
      (((Uint32_t) buffer[pos + 0]) << 24) |
      (((Uint32_t) buffer[pos + 1]) << 16) |
      (((Uint32_t) buffer[pos + 2]) << 8) |
      (((Uint32_t) buffer[pos + 3]));
  pos += 4;
  p[0] = (a << 32) + (b & 0xffffffff);
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


TaskRecvGnssIntertialLab::TaskRecvGnssIntertialLab(framework::Task* manager)
     : framework::Task(framework::TASK_ID_RECV_GNSS_DATA,
                       "Recv Gnss Data (Intertial Lab)", manager) {
  running_flag_recv_gnss_ = false;

  can_channel_ = Nullptr_t;
#if (ENABLE_CAN_DEV_ZLGCANNET)
  can_channel_ = new can_dev::CanDriverZlgCanNet();
#endif
}

TaskRecvGnssIntertialLab::~TaskRecvGnssIntertialLab() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvGnssIntertialLab::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_gnss_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 1;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 4004;
  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel 1";
    return false;
  }

  LOG_INFO(3) << "Create thread of receiving GNSS (Intertial Lab) data...";
  running_flag_recv_gnss_ = true;
  thread_recv_gnss_ = boost::thread
      (boost::bind(&TaskRecvGnssIntertialLab::ThreadReceivingGnss, this));

  LOG_INFO(3) << "Create thread of receiving GNSS (Intertial Lab) data... [OK]";

  return (true);
}

bool TaskRecvGnssIntertialLab::Stop() {
  if (running_flag_recv_gnss_) {
    running_flag_recv_gnss_ = false;

    LOG_INFO(3) << "Stop thread of receiving GNSS (Intertial Lab) data ...";
    bool ret = thread_recv_gnss_.timed_join(
        boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving GNSS (Intertial Lab) data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving GNSS (Intertial Lab) data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving GNSS (Intertial Lab) data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvGnssIntertialLab::ThreadReceivingGnss() {
  LOG_INFO(3) << "Thread of receiving GNSS (Intertial Lab) data ... [Started]";

  gnss_info_.Clear();
  imu_info_.Clear();

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame[20];
  while (running_flag_recv_gnss_) {
    frame_num = can_channel_->ReadWait(&(can_frame[0]), 20);
    //std::cout << "Received " << frame_num << " data from GNSS (Intertial Lab)." << std::endl;

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFrame& fm = can_frame[i];\
        //printf("id[%08X] data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
        //       fm.id,
        //       fm.data[0], fm.data[1], fm.data[2], fm.data[3],
        //       fm.data[4], fm.data[5], fm.data[6], fm.data[7]);

        ParseCanFrame(fm);
      }
    }
  }

  LOG_INFO(3) << "Thread of receiving GNSS (Intertial Lab) data ... [Stopped]";
}

void TaskRecvGnssIntertialLab::ParseCanFrame(const can_dev::CanFrame& frame) {
  const static Float32_t KG = 1.0e-5;
  const static Float32_t KA = 1.0e-6;
  const static Float32_t G = 9.8106F;

  switch (frame.id) {
  case (0x000): {
    // Angular rates
    Int16_t angular_rate_x = 0;
    DecodeInt16Value(&frame.data[0], 0, &angular_rate_x);

    Int16_t angular_rate_y = 0;
    DecodeInt16Value(&frame.data[0], 2, &angular_rate_y);

    Int16_t angular_rate_z = 0;
    DecodeInt16Value(&frame.data[0], 4, &angular_rate_z);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

    //imu_info_.pitch_rate = common::com_deg2rad(angular_rate_x * KG);
    //imu_info_.roll_rate = common::com_deg2rad(angular_rate_y * KG);
    //imu_info_.yaw_rate = -common::com_deg2rad(angular_rate_z * KG);

    imu_info_.pitch_rate = common::com_deg2rad(angular_rate_x * 0.020F);
    imu_info_.roll_rate = common::com_deg2rad(angular_rate_y * 0.020F);
    imu_info_.yaw_rate = common::com_deg2rad(angular_rate_z * 0.020F);

    //std::cout << "raw = " << angular_rate_z * KG
    //          << ", yaw_rate=" << imu_info_.yaw_rate << std::endl;
  }
    break;

  case (0x001): {
    // Accelerations
    Int16_t acceleration_x = 0;
    DecodeInt16Value(&frame.data[0], 0, &acceleration_x);

    Int16_t acceleration_y = 0;
    DecodeInt16Value(&frame.data[0], 2, &acceleration_y);

    Int16_t acceleration_z = 0;
    DecodeInt16Value(&frame.data[0], 4, &acceleration_z);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

    imu_info_.accel_x = acceleration_x * KA * G;
    imu_info_.accel_y = acceleration_y * KA * G;
    imu_info_.accel_z = acceleration_z * KA * G;
  }
    break;

  case (0x002): {
    // Magnetic field
    Int16_t magn_field_X = 0;
    DecodeInt16Value(&frame.data[0], 0, &magn_field_X);

    Int16_t magn_field_Y = 0;
    DecodeInt16Value(&frame.data[0], 2, &magn_field_Y);

    Int16_t magn_field_Z = 0;
    DecodeInt16Value(&frame.data[0], 4, &magn_field_Z);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);
  }
    break;

  case (0x003): {
    // Orientation
    Uint16_t heading = 0;
    DecodeUint16Value(&frame.data[0], 0, &heading);

    Int16_t pitch = 0;
    DecodeInt16Value(&frame.data[0], 2, &pitch);

    Int16_t roll = 0;
    DecodeInt16Value(&frame.data[0], 4, &roll);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

    gnss_info_.heading_gnss = heading * 0.01F;
    gnss_info_.pitch = common::com_deg2rad(pitch * 0.01F);
    gnss_info_.roll = common::com_deg2rad(roll * 0.01F);
  }
    break;

  case (0x004): {
    // East velocity
    Int32_t east_velocity = 0;
    DecodeInt32Value(&frame.data[0], 0, &east_velocity);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 4, &time_stamp);

    gnss_info_.v_e = east_velocity * 0.01F;
    gnss_info_.v_x_utm = gnss_info_.v_e;
  }
    break;

  case (0x005): {
    // North velocity
    Int32_t north_velocity = 0;
    DecodeInt32Value(&frame.data[0], 0, &north_velocity);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 4, &time_stamp);

    gnss_info_.v_n = north_velocity * 0.01F;
    gnss_info_.v_y_utm = gnss_info_.v_n;
  }
    break;

  case (0x006): {
    // Vertical velocity
    Int32_t vertical_velocity = 0;
    DecodeInt32Value(&frame.data[0], 0, &vertical_velocity);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 4, &time_stamp);

    gnss_info_.v_u = vertical_velocity * 0.01F;
    gnss_info_.v_z_utm = gnss_info_.v_u;
  }
    break;

  case (0x007): {
    // Longitude
    Int64_t longitude = 0;
    Uint8_t lon_data[8] = { 0 };
    common::com_memcpy(&lon_data[2], frame.data, 6);
    DecodeInt64Value(lon_data, 0, &longitude);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

    gnss_info_.longitude = longitude * 1e-9;
  }
    break;

  case (0x008): {
    // Latitude
    Int64_t latitude = 0;
    Uint8_t lat_data[8] = { 0 };
    common::com_memcpy(&lat_data[2], frame.data, 6);
    DecodeInt64Value(lat_data, 0, &latitude);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

    gnss_info_.latitude = latitude * 1e-9;

    Float64_t heading_utm = 0.0;
    Float64_t utm_northing = 0.0;
    Float64_t utm_easting = 0.0;
    Float64_t yaw_offset_deg = -0.5;
    Float64_t converge_angle = common::LLtoUTM(
          gnss_info_.latitude, gnss_info_.longitude, &utm_northing, &utm_easting);
    //调整为逆时针旋转
    heading_utm = 90.0 - gnss_info_.heading_gnss + yaw_offset_deg + converge_angle;
    if (heading_utm < 0) {
      heading_utm += 360.0;
    }
    gnss_info_.x_utm = utm_easting;
    gnss_info_.y_utm = utm_northing;
    gnss_info_.heading_utm = common::NormalizeAngle(common::com_deg2rad(heading_utm));

    common::Vec2d unit_direction[2];
    unit_direction[0].set_x(common::com_cos(gnss_info_.heading_utm));
    unit_direction[0].set_y(common::com_sin(gnss_info_.heading_utm));
    unit_direction[1].set_x(-unit_direction[0].y());
    unit_direction[1].set_y(unit_direction[0].x());

    phoenix::common::Vec2d offset_lon = -0.6F * unit_direction[0];
    phoenix::common::Vec2d offset_lat = 0.0F * unit_direction[1];
    gnss_info_.x_utm += offset_lon.x() + offset_lat.x();
    gnss_info_.y_utm += offset_lon.y() + offset_lat.y();
  }
    break;

  case (0x009): {
    // Altitude
    Int32_t altitude = 0;
    DecodeInt32Value(&frame.data[0], 0, &altitude);

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 4, &time_stamp);

    gnss_info_.altitude = altitude * 0.001F;
    gnss_info_.z_utm = gnss_info_.altitude;
  }
    break;

  case (0x00A): {
    // Time information
    // GPS time (integer part)
    Uint32_t gps_time_integer_part = 0;
    DecodeUint32Value(&frame.data[0], 0, &gps_time_integer_part);

    // GPS time (milliseconds)
    Uint16_t gps_time_milliseconds = 0;
    DecodeUint16Value(&frame.data[0], 4, &gps_time_milliseconds);
  }
    break;

  case (0x00B): {
    // INS solution accuracy
    // INS position covariance
    Uint16_t ins_position_covariance = 0;
    DecodeUint16Value(&frame.data[0], 0, &ins_position_covariance);

    // INS velocity covariance
    Uint16_t ins_velocity_covariance = 0;
    DecodeUint16Value(&frame.data[0], 2, &ins_velocity_covariance);

    // INS heading covariance
    Int16_t ins_heading_covariance = 0;
    DecodeInt16Value(&frame.data[0], 4, &ins_heading_covariance);

    // INS solution status
    Uint8_t ins_solution_status = frame.data[5];

    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 6, &time_stamp);

  }
    break;

  case (0x00C): {
    // GNSS info
    //#SolnSVs
    Int8_t solnSVs = frame.data[0];

    // GNSS position type
    Uint8_t gnss_position_type  = frame.data[1];
    switch (gnss_position_type) {
    case (0):
      // No solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_INVALID;
      break;
    case (8):
      // Velocity computed using instantaneous Doppler
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (16):
      // Single point position
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (17):
      // Pseudorange differential solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (18):
      // Solution calculated using corrections from an WAAS
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (19):
      // Propagated by a Kalman filter without new observations
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (20):
      // OmniSTAR VBS position
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (32):
      // Floating L1 ambiguity solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case (33):
      // Floating ionospheric-free ambiguity solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case (34):
      // Floating narrow-lane ambiguity solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case (48):
      // Integer L1 ambiguity solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    case (50):
      // Integer narrow-lane ambiguity solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    case (64):
      // OmniSTAR HP position
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (65):
      // OmniSTAR XP or G2 position
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (68):
      // Converging TerraStar-C, TerraStar-C PRO or TerraStar-X solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (69):
      // Converged TerraStar-C, TerraStar-C PRO or TerraStar-X solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (77):
      // Converging TerraStar-L solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (78):
      // Converged TerraStar-L solution
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    default:
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_INVALID;
      break;
    }
    gnss_info_.utm_status = gnss_info_.gnss_status;

    // GPS time status
    Uint8_t gps_time_status = frame.data[2];

    // GNSS solution status
    Uint8_t gnss_solution_status = frame.data[3];
    switch (gnss_solution_status) {
    case (0):
      // Solution computed
      break;
    case (1):
      // Insufficient observations
      break;
    case (2):
      // No convergence
      break;
    case (3):
      // Singularity at parameters matrix
      break;
    case (4):
      // Covariance trace exceeds maximum (trace > 1000 m)
      break;
    case (5):
      // Test distance exceeded (maximum of 3 rejections if distance >10 km)
      break;
    case (6):
      // Not yet converged from cold start
      break;
    case (7):
      // Height or velocity limits exceeded (in accordance with export licensing restrictions)
      break;
    case (8):
      // Variance exceeds limits
      break;
    case (9):
      // Residuals are too large
      break;
    case (13):
      // Large residuals make position unreliable
      break;
    case (18):
      // When a FIX POSITION command is entered, the receiver computes its own position and
      // determines if the fixed position is valid
      break;
    case (19):
      // The fixed position, entered using the FIX POSITION command, is not valid
      break;
    case (20):
      // Position type is unauthorized - HP or XP on a receiver not authorized
      break;
    default:
      break;
    }
    printf("gnss_position_type = %d\n",gnss_position_type);
    printf("gnss_solution_status = %d\n",gnss_solution_status);


    Uint16_t time_stamp = 0;
    DecodeUint16Value(&frame.data[0], 4, &time_stamp);
  }
    break;

  default:
    break;
  }

  // Notify
  if (0x00C == frame.id) {
    framework::MessageRecvImuData imu_message(&imu_info_);
    Notify(imu_message);
    framework::MessageRecvGnssData gnss_message(&gnss_info_);
    Notify(gnss_message);
  }
}


}  // namespace intertial_lab
}  // namespace imu
}  // namespace senser
}  // namespace phoenix
