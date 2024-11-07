/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       string_ring_buffer.h
 * @brief      字符串环形缓冲区
 * @details    定义字符串环形缓冲区
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_STRING_RING_BUFFER_C_H_
#define PHOENIX_COMMON_STRING_RING_BUFFER_C_H_


#include "utils/macros.h"


/**
 * @struct StringRingBuf
 * @brief 字符串环形缓冲区
 */
typedef struct _StringRingBuf_t StringRingBuf_t;
struct _StringRingBuf_t{
  /// 未读区域的起始索引
  Int32_t read_index;
  /// 未写区域的起始索引
  Int32_t write_index;
  /// 字符串数据存储区的最大字节数
  Int32_t max_bytes;
  /// 字符串数据存储区首地址
  Char_t* buff;
  /// 已保存的数据字节
  Int32_t data_count;
};

/**
 * @enum
 * @brief 字符串环形缓冲区操作函数的返回值类型
 */
enum {
  RET_STR_RING_BUF_NO_DATA = -3,         /// 缓冲区中没有数据
  RET_STR_RING_BUF_NO_SPACE = -2,        /// 缓冲区已经满了
  RET_STR_RING_BUF_NG = -1,              /// 缓冲区操作失败
  RET_STR_RING_BUF_OK                    /// 缓冲区操作成功
};


/**
 * @brief 初始化字符串环形缓冲区
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @param[in] buff 字符串数据存储区首地址
 * @param[in] max_bytes 字符串数据存储区大小
 * @return STR_RING_BUF_OK 成功 \n
 *         STR_RING_BUF_NG 失败
 */
Int32_t Phoenix_Com_InitStringRingBuf(
    StringRingBuf_t* ring_buff, Char_t* buff, Int32_t max_bytes);

/**
 * @brief 销毁字符串环形缓冲区
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return STR_RING_BUF_OK 成功 \n
 *         STR_RING_BUF_NG 失败
 */
Int32_t Phoenix_Com_ReleaseStringRingBuf(StringRingBuf_t* ring_buff);

/**
 * @brief 获取字符串环形缓冲区中未读区域的起始索引
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 未读区域的起始索引
 */
Int32_t Phoenix_Com_GetStringRingBufReadIndex(StringRingBuf_t* ring_buff);

/**
 * @brief 获取字符串环形缓冲区中未写区域的起始索引
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 未写区域的起始索引
 */
Int32_t Phoenix_Com_GetStringRingBufWriteIndex(StringRingBuf_t* ring_buff);

/**
 * @brief 获取字符串环形缓冲区中未使用的空间的大小
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 未使用的空间的大小
 */
Int32_t Phoenix_Com_GetStringRingBufUnusedSpace(StringRingBuf_t* ring_buff);

/**
 * @brief 获取字符串环形缓冲区中已使用的空间的大小
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 已使用的空间的大小
 */
Int32_t Phoenix_Com_GetStringRingBufUsedSpace(StringRingBuf_t* ring_buff);

/**
 * @brief 向字符串环形缓冲区中写入数据(如果缓冲区中空间不足，则不写入)
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @param[in] src_addr 需要写入的数据的地址
 * @param[in] write_size 需要写入的数据的大小
 * @return STR_RING_BUF_OK 成功 \n
 *         STR_RING_BUF_NG 失败 \n
 *         STR_RING_BUF_NO_SPACE 空间不足
 */
Int32_t Phoenix_Com_WriteToStringRingBuf(
    StringRingBuf_t* ring_buff, Char_t* src_addr, Int32_t write_size);

/**
 * @brief 从字符串环形缓冲区中读取指定大小的数据(如果缓冲区中的数据小于指定的大小，则不读取)
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @param[in] dest_addr 存储需要读取的数据的地址
 * @param[in] read_size 需要读取的数据的大小
 * @return STR_RING_BUF_OK 成功 \n
 *         STR_RING_BUF_NG 失败 \n
 *         STR_RING_BUF_NO_DATA 数据不足
 */
Int32_t Phoenix_Com_ReadFromStringRingBufWithSpecialSize(
    StringRingBuf_t* ring_buff, Char_t* dest_addr, Int32_t read_size);

/**
 * @brief 向字符串环形缓冲区中写入数据(如果缓冲区中空间不足，则覆盖之前写入的数据)
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @param[in] src_addr 需要写入的数据的地址
 * @param[in] write_size 需要写入的数据的大小
 * @return 实际写入的数据的大小 \n
 *         STR_RING_BUF_NG 失败 \n
 *         STR_RING_BUF_NO_SPACE 空间不足(需要写入的数据太大了，大于整个缓冲区的大小)
 */
Int32_t Phoenix_Com_WriteToStringRingBufOverride(
    StringRingBuf_t* ring_buff, const Char_t* src_addr, Int32_t write_size);

/**
 * @brief 从字符串环形缓冲区中读取数据(如果缓冲区中的数据小于指定的大小
 *        则读取缓冲区已有的数据)
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @param[in] dest_addr 存储需要读取的数据的地址
 * @param[in] read_size 需要读取的数据的大小
 * @return 实际读取的数据的大小 成功 \n
 *         STR_RING_BUF_NG 失败
 */
Int32_t Phoenix_Com_ReadFromStringRingBuf(
    StringRingBuf_t* ring_buff, Char_t* dest_addr, Int32_t read_size);


#endif  // PHOENIX_COMMON_STRING_RING_BUFFER_C_H_
