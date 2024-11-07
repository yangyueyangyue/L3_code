//
#ifndef PHOENIX_SENSOR_PARSE_SENSOR_DATA_COMMON_H_
#define PHOENIX_SENSOR_PARSE_SENSOR_DATA_COMMON_H_


#include "ad_msg.h"
#include "math/math_utils.h"
#include "container/static_vector.h"
#include "can_dev/can_driver.h"


namespace phoenix {
namespace sensor {

uint8_t Offset_DecToHex(uint8_t Length_bit);

uint32_t parse_sensor_data(const uint8_t *CANData, uint8_t StartByte, uint16_t StartBit, uint8_t LengthBit, bool CANFormat = 0);

}  // namespace sensor
}  // namespace phoenix


#endif // PHOENIX_SENSOR_PARSE_SENSOR_DATA_COMMON_H_

