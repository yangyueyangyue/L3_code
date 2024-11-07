//武汉迈普时空GPS协议解析

#ifndef GNSS_PARSER_MPST_MPST_MESSAGE_H_
#define GNSS_PARSER_MPST_MPST_MESSAGE_H_

#include <stdint.h>

namespace Localization{
namespace MPST{

enum SyncByte : uint8_t {
  SYNC_0 = 0xAA,        //字头, 0xAA
  SYNC_1 = 0x55,        //字头, 0x55
};

enum CompositeMode{
    TC_MODE = 0,        //紧组合(TC)
    LC_MODE = 1,        //松组合(LC)
};

enum GnssMode{
    SPP_MODE = 0,
    RTK_FLOAT_MODE = 1,
    RTK_FIXED_MODE = 2,
    NO_GNSS_MODE = 3,
    USELESS_RTK_MODE = 4,
};

#pragma pack(push, 1)  //turn off struct padding.
//头信息
struct HeaderInfo {
    SyncByte sync[2];                             //字头
    uint8_t message_length;                 //数据长度
    uint8_t composite_mode;               //组合模式
    uint16_t gps_week;                          // GPS Week number.
    uint32_t gps_millisecs;                    //GPS millisecond of week
    uint8_t satelitie_num;                      //观测卫星个数 N
};
//static_assert(sizeof(HeaderInfo) ==16, "Incorrect size of HeaderInfo");     //字节对齐

struct Ins {
  double latitude;        // 纬度 /deg
  double longitude;       // 经度 /deg
  float height;                 // 高程 /m
  float north_velocity;  // 北向速度 /m/s
  float east_velocity;   // 东向速度 /m/s
  float up_velocity;     // 地向速度 /m/s
  float roll;                  // 横滚 /deg
  float pitch;                 // 俯仰 /deg
  float yaw;                   // 航向 /deg
  int16_t north_acceleration;     //北向加速度 /m/s^2 , 10^-2
  int16_t east_acceleration;      //东向加速度 /m/s^2 , 10^-2
  int16_t up_acceleration;        //地向加速度 /m/s^2 , 10^-2
  int16_t roll_velocity;                  // 横滚角速度 /deg/s , 10^-2
  int16_t pitch_velocity;                 // 俯仰角速度 /deg/s , 10^-2
  int16_t yaw_velocity;                   // 航向角速度 /deg/s , 10^-2
  uint8_t status;
};
//static_assert(sizeof(Ins) == 58, "Incorrect size of Ins");

#pragma pack(pop)  //back to whatever the previous packing mode was.

}   //MPST
}  // Localization

#endif  // GNSS_PARSER_MPST_MPST_MESSAGE_H_
