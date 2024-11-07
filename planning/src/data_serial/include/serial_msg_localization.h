/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_serialization.h
 * @brief      通讯用的数据序列化
 * @details    实现了基通讯用的数据序列化的操作函数
 *
 * @author     longjiaoy
 * @date       2021.02.07
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/02/07  <td>1.0      <td>longjiaoy     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_H_
#define PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_H_


#include "ad_msg.h"
#include "utils/macros.h"
#include "utils/com_utils.h"


namespace phoenix {
namespace data_serial {


/**
 * @brief 序列化 Gnss 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeGnssArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Gnss* p, Int32_t elements);

/**
 * @brief 反序列化 Gnss 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeGnssArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Gnss* p, Int32_t elements);


/**
 * @brief 序列化 Imu 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeImuArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Imu* p, Int32_t elements);

/**
 * @brief 反序列化 Imu 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeImuArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Imu* p, Int32_t elements);

/**
 * @brief 序列化 RelativePos 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeRelativePosArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::RelativePos* p, Int32_t elements);

/**
 * @brief 反序列化 RelativePos 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeRelativePosArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::RelativePos* p, Int32_t elements);


/**
 * @brief 序列化 RelativePosList 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeRelativePosListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::RelativePosList* p, Int32_t elements);

/**
 * @brief 反序列化 RelativePosList 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeRelativePosListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::RelativePosList* p, Int32_t elements);


}  // namespace phoenix
}  // namespace data_serial


#endif // PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_H_
