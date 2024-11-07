/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       serialization_utils.h
 * @brief      基本数据类型序列化
 * @details    实现了基本数据类型序列化的操作函数
 *
 * @author     pengc
 * @date       2020.09.01
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_COMMON_SERIALIZATION_UTILS_C_H_
#define PHOENIX_COMMON_SERIALIZATION_UTILS_C_H_


#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 获取序列化 Uint8_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedUint8Array(
    const Uint8_t* p, Int32_t elements);

/**
 * @brief 序列化 Uint8_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeUint8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint8_t* p, Int32_t elements);

/**
 * @brief 反序列化 Uint8_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeUint8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint8_t* p, Int32_t elements);

/**
 * @brief 拷贝 Uint8_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneUint8Array(
    const Uint8_t* p, Uint8_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Int8_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedInt8Array(
    const Int8_t* p, Int32_t elements);

/**
 * @brief 序列化 Int8_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeInt8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int8_t* p, Int32_t elements);

/**
 * @brief 反序列化 Int8_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeInt8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int8_t* p, Int32_t elements);

/**
 * @brief 拷贝 Int8_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneInt8Array(
    const Int8_t* p, Int8_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Int16_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedInt16Array(
    const Int16_t* p, Int32_t elements);

/**
 * @brief 序列化 Int16_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeInt16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int16_t* p, Int32_t elements);

/**
 * @brief 反序列化 Int16_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeInt16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int16_t* p, Int32_t elements);

/**
 * @brief 拷贝 Int16_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneInt16Array(
    const Int16_t* p, Int16_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Uint16_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedUint16Array(
    const Uint16_t* p, Int32_t elements);

/**
 * @brief 序列化 Uint16_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeUint16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint16_t* p, Int32_t elements);

/**
 * @brief 反序列化 Uint16_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeUint16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint16_t* p, Int32_t elements);

/**
 * @brief 拷贝 Uint16_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneUint16Array(
    const Uint16_t* p, Uint16_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Int32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedInt32Array(
    const Int32_t* p, Int32_t elements);

/**
 * @brief 序列化 Int32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeInt32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int32_t* p, Int32_t elements);

/**
 * @brief 反序列化 Int32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeInt32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int32_t* p, Int32_t elements);

/**
 * @brief 拷贝 Int32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneInt32Array(
    const Int32_t* p, Int32_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Uint32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedUint32Array(
    const Uint32_t* p, Int32_t elements);

/**
 * @brief 序列化 Uint32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeUint32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint32_t* p, Int32_t elements);

/**
 * @brief 反序列化 Uint32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeUint32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint32_t* p, Int32_t elements);

/**
 * @brief 拷贝 Uint32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneUint32Array(
    const Uint32_t* p, Uint32_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Int64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedInt64Array(
    const Int64_t* p, Int32_t elements);

/**
 * @brief 序列化 Int64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeInt64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int64_t* p, Int32_t elements);

/**
 * @brief 反序列化 Int64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeInt64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int64_t* p, Int32_t elements);

/**
 * @brief 拷贝 Int64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneInt64Array(
    const Int64_t* p, Int64_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Uint64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedUint64Array(
    const Uint64_t* p, Int32_t elements);

/**
 * @brief 序列化 Uint64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeUint64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint64_t* p, Int32_t elements);

/**
 * @brief 反序列化 Uint64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeUint64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint64_t* p, Int32_t elements);

/**
 * @brief 拷贝 Uint64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneUint64Array(
    const Uint64_t* p, Uint64_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Float32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedFloat32Array(
    const Float32_t* p, Int32_t elements);

/**
 * @brief 序列化 Float32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeFloat32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float32_t* p, Int32_t elements);

/**
 * @brief 反序列化 Float32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeFloat32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float32_t* p, Int32_t elements);

/**
 * @brief 拷贝 Float32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneFloat32Array(
    const Float32_t* p, Float32_t* q, Int32_t elements);


/**
 * @brief 获取序列化 Float64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
Int32_t Phoenix_Common_GetSizeOfEncodedFloat64Array(
    const Float64_t *p, Int32_t elements);

/**
 * @brief 序列化 Float64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t Phoenix_Common_EncodeFloat64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float64_t* p, Int32_t elements);

/**
 * @brief 反序列化 Float64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t Phoenix_Common_DecodeFloat64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float64_t* p, Int32_t elements);

/**
 * @brief 拷贝 Float64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
void Phoenix_Common_CloneFloat64Array(
    const Float64_t* p, Float64_t* q, Int32_t elements);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_COMMON_SERIALIZATION_UTILS_C_H_


