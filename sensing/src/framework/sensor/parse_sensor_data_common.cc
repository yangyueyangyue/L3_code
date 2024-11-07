//
#include "sensor/parse_sensor_data_common.h"

namespace phoenix {
namespace sensor {

uint8_t Offset_DecToHex(uint8_t Length_bit)
{
    uint8_t offset_hex = 0;
    switch (Length_bit)
    {
        case 1:
            offset_hex = 0x01;
        break;

        case 2:
            offset_hex = 0x03;
        break;
        
        case 3:
            offset_hex = 0x07;
        break;
        
        case 4:
            offset_hex = 0x0F;
        break;
        
        case 5:
            offset_hex = 0x1F;
        break;
        
        case 6:
            offset_hex = 0x3F;
        break;
        
        case 7:
            offset_hex = 0x7F;
        break;
        
        case 8:
            offset_hex = 0xFF;
        break;

        default:
        break;
    }

    return offset_hex;
}

/*
 *  功能：根据需要解析的CAN参数对数据进行切片
 *  参数：
 *      uint8_t *CANData: 需要解析的数据
 *      uint8_t StartByte:  起始字节
 *      uint8_t StartBit:   起始位
 *      uint8_t LengthBit:  长度(bit)
 *      bool CANFormat: 小端-Inter(false), 大端-Motola(true)
 *  返回值：切片后的数据
 */
uint32_t parse_sensor_data(const uint8_t *CANData, uint8_t StartByte, uint16_t StartBit, uint8_t LengthBit, bool CANFormat)
{
    uint32_t SliceData = 0;

    // 起始位在起始字节中所占的位置
    uint8_t StartByteIndex = StartBit - (StartByte * 8);

    // Inter格式
    if(0 == CANFormat)
    {
        // 所占长度为8的倍数 且 从起始字节0位开始
        if(0 == (LengthBit % 8) && 0 == StartByteIndex)
        {
            uint8_t LengthByte = LengthBit / 8;
            for(int i = 1; i <= LengthByte; i++)
            {
                SliceData |= CANData[StartByte + (i - 1)] << ((i - 1) * 8);
            }
        }
        // 全部数据都在起始字节内
        else if((StartByteIndex + LengthBit) <= 8)
        {
            uint8_t offset_hex = Offset_DecToHex(LengthBit);

            SliceData = (CANData[StartByte] >> StartByteIndex) & offset_hex;
        }
        // 一部分数据在起始字节，一部分数据在下一字节内
        else if((StartByteIndex + LengthBit) > 8 && (LengthBit - (8 - StartByteIndex)) <= 8)
        {
            uint8_t current_byte_length = 8 - StartByteIndex;
            uint8_t next_byte_length = LengthBit - current_byte_length;
            
            uint8_t current_byte_length_hex = Offset_DecToHex(current_byte_length);
            uint8_t next_byte_length_hex = Offset_DecToHex(next_byte_length);

            SliceData = ((CANData[StartByte] >> StartByteIndex) & current_byte_length_hex) | ((CANData[StartByte + 1] & next_byte_length_hex) << current_byte_length);
        }
        else if((StartByteIndex + LengthBit) > 8 && (LengthBit - (8 - StartByteIndex)) > 8 && (LengthBit - (8 - StartByteIndex)) <= 16)
        {
            uint8_t current_byte_length = 8 - StartByteIndex;
            uint8_t mid_byte_length = 8;
            uint8_t end_byte_length = LengthBit - mid_byte_length - current_byte_length;

            uint8_t current_byte_length_hex = Offset_DecToHex(current_byte_length);
            uint8_t mid_byte_length_hex = Offset_DecToHex(mid_byte_length);
            uint8_t end_byte_length_hex = Offset_DecToHex(end_byte_length);

            SliceData = ((CANData[StartByte] >> StartByteIndex) & current_byte_length_hex) | ((CANData[StartByte + 1] & mid_byte_length_hex) << current_byte_length) | ((CANData[StartByte + 2] & end_byte_length_hex) << (current_byte_length + mid_byte_length));
        }
    }
    // Motola格式
    else
    {
        // 所占长度为8的倍数 且 从起始字节0位开始
        if(LengthBit % 8 == 0 && StartBit == (StartByte * 8))
        {
            uint8_t LengthByte = LengthBit / 8;
            for(int i = LengthByte; i >= 1; i--)
            {
                SliceData |= CANData[StartByte + (i - 1)] << ((LengthByte - i) * 8);
            }
        }
        //  所占长度小于8 且 全部数据都在起始字节内
        else if((StartByteIndex + LengthBit) < 8)
        {
            uint8_t offset_hex = Offset_DecToHex(LengthBit);

            SliceData = CANData[StartByte] >> StartByteIndex & offset_hex;
        }
        // 一部分数据在起始字节，一部分数据在下一字节内
        else if((StartByteIndex + LengthBit) > 8 && (LengthBit - (8 - StartByteIndex)) <= 8)
        {
            uint8_t current_byte_length = 8 - StartByteIndex;
            uint8_t next_byte_length = LengthBit - current_byte_length;
            
            uint8_t current_byte_length_hex = Offset_DecToHex(current_byte_length);
            uint8_t next_byte_length_hex = Offset_DecToHex(next_byte_length);

            SliceData = (CANData[StartByte + 1] & next_byte_length_hex) | (((CANData[StartByte] >> StartByteIndex) & current_byte_length_hex) << next_byte_length);
        }
        else if((StartByteIndex + LengthBit) > 8 && (LengthBit - (8 - StartByteIndex)) > 8 && (LengthBit - (8 - StartByteIndex)) <= 16)
        {
            uint8_t current_byte_length = 8 - StartByteIndex;
            uint8_t mid_byte_length = 8;
            uint8_t end_byte_length = LengthBit - mid_byte_length - current_byte_length;

            uint8_t current_byte_length_hex = Offset_DecToHex(current_byte_length);
            uint8_t mid_byte_length_hex = Offset_DecToHex(mid_byte_length);
            uint8_t end_byte_length_hex = Offset_DecToHex(end_byte_length);

            SliceData = (CANData[StartByte + 2] & end_byte_length_hex) | ((CANData[StartByte + 1] & mid_byte_length_hex) << end_byte_length) | (((CANData[StartByte] >> StartByteIndex) & current_byte_length_hex) << (end_byte_length + mid_byte_length));
        }
    }

    return SliceData;
}


}  // namespace sensor
}  // namespace phoenix
