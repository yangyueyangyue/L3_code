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
#ifndef PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_C_H_
#define PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_C_H_


#include "ad_msg_c.h"
#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 序列化 Imu 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeImuArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Imu_t* p, Int32_t elements);

/**
 * @brief 反序列化 Imu 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeImuArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Imu_t* p, Int32_t elements);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_DATA_SERIAL_SERIAL_MSG_LOCALIZATION_C_H_
