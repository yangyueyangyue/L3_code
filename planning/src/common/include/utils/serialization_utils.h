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
#ifndef PHOENIX_COMMON_SERIALIZATION_UTILS_H_
#define PHOENIX_COMMON_SERIALIZATION_UTILS_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"


namespace phoenix {
namespace common {


/**
 * @brief 获取序列化 Uint8_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedUint8Array(
    const Uint8_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint8_t) * elements);
}

/**
 * @brief 序列化 Uint8_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeUint8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR << "(max_len=" << maxlen << ") < (elements=" << elements << ")";
    return -1;
  }

  Uint8_t* buffer = (Uint8_t*) buf;
  com_memcpy(&buffer[offset], p, elements);

  return (elements);
}

/**
 * @brief 反序列化 Uint8_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeUint8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR << "(max_len=" << maxlen << ") < (elements=" << elements << ")";
    return -1;
  }

  const Uint8_t* buffer = (const Uint8_t*) buf;
  com_memcpy(p, &buffer[offset], elements);

  return (elements);
}

/**
 * @brief 拷贝 Uint8_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneUint8Array(
    const Uint8_t* p, Uint8_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Uint8_t));
}


/**
 * @brief 获取序列化 Int8_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedInt8Array(
    const Int8_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int8_t) * elements);
}

/**
 * @brief 序列化 Int8_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeInt8Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR << "(max_len=" << maxlen << ") < (elements=" << elements << ")";
    return -1;
  }

  Int8_t* buffer = (Int8_t*) buf;
  com_memcpy(&buffer[offset], p, elements);

  return (elements);
}

/**
 * @brief 反序列化 Int8_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeInt8Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int8_t* p, Int32_t elements) {
  if (maxlen < elements) {
    LOG_ERR << "(max_len=" << maxlen << ") < (elements=" << elements << ")";
    return -1;
  }

  const Int8_t* buffer = (const Int8_t*) buf;
  com_memcpy(p, &buffer[offset], elements);

  return (elements);
}

/**
 * @brief 拷贝 Int8_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneInt8Array(
    const Int8_t* p, Int8_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Int8_t));
}


/**
 * @brief 获取序列化 Int16_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedInt16Array(
    const Int16_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int16_t) * elements);
}

/**
 * @brief 序列化 Int16_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeInt16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int16_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int16_t) * elements;
  Uint8_t* buffer = (Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint16_t* unsigned_p = (const Uint16_t*) p;
  for (index = 0; index < elements; index++) {
    Uint16_t v = unsigned_p[index];
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

/**
 * @brief 反序列化 Int16_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeInt16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int16_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int16_t) * elements;
  const Uint8_t* buffer = (const Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  for (index = 0; index < elements; index++) {
    p[index] = (buffer[pos] << 8) + buffer[pos + 1];
    pos += 2;
  }

  return (total_size);
}

/**
 * @brief 拷贝 Int16_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneInt16Array(
    const Int16_t* p, Int16_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Int16_t));
}


/**
 * @brief 获取序列化 Uint16_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedUint16Array(
    const Uint16_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint16_t) * elements);
}

/**
 * @brief 序列化 Uint16_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeUint16Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint16_t* p, Int32_t elements) {
  return EncodeInt16Array(buf, offset, maxlen, (const Int16_t*) p, elements);
}

/**
 * @brief 反序列化 Uint16_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeUint16Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint16_t* p, Int32_t elements) {
  return DecodeInt16Array(buf, offset, maxlen, (Int16_t*) p, elements);
}

/**
 * @brief 拷贝 Uint16_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneUint16Array(
    const Uint16_t* p, Uint16_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Uint16_t));
}


/**
 * @brief 获取序列化 Int32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedInt32Array(
    const Int32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int32_t) * elements);
}

/**
 * @brief 序列化 Int32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeInt32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int32_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int32_t) * elements;
  Uint8_t* buffer = (Uint8_t *) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint32_t* unsigned_p = (const Uint32_t*) p;
  for (index = 0; index < elements; index++) {
    const Uint32_t v = unsigned_p[index];
    buffer[pos++] = (v >> 24) & 0xff;
    buffer[pos++] = (v >> 16) & 0xff;
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

/**
 * @brief 反序列化 Int32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeInt32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int32_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int32_t) * elements;
  const Uint8_t* buffer = (const Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  for (index = 0; index < elements; index++) {
    p[index] =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
  }

  return (total_size);
}

/**
 * @brief 拷贝 Int32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneInt32Array(
    const Int32_t* p, Int32_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Int32_t));
}


/**
 * @brief 获取序列化 Uint32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedUint32Array(
    const Uint32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint32_t) * elements);
}

/**
 * @brief 序列化 Uint32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeUint32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint32_t* p, Int32_t elements) {
  return EncodeInt32Array(buf, offset, maxlen, (const Int32_t*) p, elements);
}

/**
 * @brief 反序列化 Uint32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeUint32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint32_t* p, Int32_t elements) {
  return DecodeInt32Array(buf, offset, maxlen, (Int32_t*) p, elements);
}

/**
 * @brief 拷贝 Uint32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneUint32Array(
    const Uint32_t* p, Uint32_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Uint32_t));
}


/**
 * @brief 获取序列化 Int64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedInt64Array(
    const Int64_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Int64_t) * elements);
}

/**
 * @brief 序列化 Int64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeInt64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int64_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int64_t) * elements;
  Uint8_t* buffer = (Uint8_t*) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  const Uint64_t* unsigned_p = (const Uint64_t*) p;
  for (index = 0; index < elements; index++) {
    const Uint64_t v = unsigned_p[index];
    buffer[pos++] = (v >> 56) & 0xff;
    buffer[pos++] = (v >> 48) & 0xff;
    buffer[pos++] = (v >> 40) & 0xff;
    buffer[pos++] = (v >> 32) & 0xff;
    buffer[pos++] = (v >> 24) & 0xff;
    buffer[pos++] = (v >> 16) & 0xff;
    buffer[pos++] = (v >> 8) & 0xff;
    buffer[pos++] = (v & 0xff);
  }

  return (total_size);
}

/**
 * @brief 反序列化 Int64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeInt64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int64_t* p, Int32_t elements) {
  Int32_t total_size = sizeof(Int64_t) * elements;
  const Uint8_t* buffer = (const Uint8_t *) buf;
  Int32_t pos = offset;
  Int32_t index = 0;

  if (maxlen < total_size) {
    LOG_ERR << "(max_len=" << maxlen << ") < (total_size=" << total_size << ")";
    return -1;
  }

  //  See Section 5.8 paragraph 3 of the standard
  //  http://open-std.org/JTC1/SC22/WG21/docs/papers/2015/n4527.pdf
  //  use uint for shifting instead if int
  for (index = 0; index < elements; index++) {
    Uint64_t a =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
    Uint64_t b =
        (((Uint32_t) buffer[pos + 0]) << 24) +
        (((Uint32_t) buffer[pos + 1]) << 16) +
        (((Uint32_t) buffer[pos + 2]) << 8) +
        ((Uint32_t)  buffer[pos + 3]);
    pos += 4;
    p[index] = (a << 32) + (b & 0xffffffff);
  }

  return total_size;
}

/**
 * @brief 拷贝 Int64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneInt64Array(
    const Int64_t* p, Int64_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Int64_t));
}


/**
 * @brief 获取序列化 Uint64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedUint64Array(
    const Uint64_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Uint64_t) * elements);
}

/**
 * @brief 序列化 Uint64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeUint64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint64_t* p, Int32_t elements) {
  return EncodeInt64Array(buf, offset, maxlen, (const Int64_t*) p, elements);
}

/**
 * @brief 反序列化 Uint64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeUint64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint64_t* p, Int32_t elements) {
  return DecodeInt64Array(buf, offset, maxlen, (Int64_t*) p, elements);
}

/**
 * @brief 拷贝 Uint64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneUint64Array(
    const Uint64_t* p, Uint64_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Uint64_t));
}


/**
 * @brief 获取序列化 Float32_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedFloat32Array(
    const Float32_t* p, Int32_t elements) {
  (void) p;
  return (sizeof(Float32_t) * elements);
}

/**
 * @brief 序列化 Float32_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeFloat32Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float32_t* p, Int32_t elements) {
  return EncodeInt32Array(buf, offset, maxlen, (const Int32_t*) p, elements);
}

/**
 * @brief 反序列化 Float32_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeFloat32Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float32_t* p, Int32_t elements) {
  return DecodeInt32Array(buf, offset, maxlen, (Int32_t*) p, elements);
}

/**
 * @brief 拷贝 Float32_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneFloat32Array(
    const Float32_t* p, Float32_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Float32_t));
}


/**
 * @brief 获取序列化 Float64_t 数组所需要的内存大小
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化所需要的内存大小
 */
static inline Int32_t GetSizeOfEncodedFloat64Array(
    const Float64_t *p, Int32_t elements) {
  (void) p;
  return (sizeof(Float64_t) * elements);
}

/**
 * @brief 序列化 Float64_t 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
static inline Int32_t EncodeFloat64Array(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float64_t* p, Int32_t elements) {
  return EncodeInt64Array(buf, offset, maxlen, (const Int64_t*) p, elements);
}

/**
 * @brief 反序列化 Float64_t 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
static inline Int32_t DecodeFloat64Array(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float64_t* p, Int32_t elements) {
  return DecodeInt64Array(buf, offset, maxlen, (Int64_t*) p, elements);
}

/**
 * @brief 拷贝 Float64_t 数组
 * @param p[in] 数据的源地址
 * @param p[in] 需要拷贝数据的目标地址
 * @param elements[in] 需要拷贝的数组元素的个数
 */
static inline void CloneFloat64Array(
    const Float64_t* p, Float64_t* q, Int32_t elements) {
  com_memcpy(q, p, elements * sizeof(Float64_t));
}


#if 0

template <typename T>
inline Int32_t GetSizeOfEncodedValueArray(
    const T *p, Int32_t elements) {
  LOG_ERR << "Unsupported value type.";
  return (-1);
}

template <typename T>
inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const T* p, Int32_t elements) {
  LOG_ERR << "Unsupported value type.";
  return (-1);
}

template <typename T>
inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    T* p, Int32_t elements) {
  LOG_ERR << "Unsupported value type.";
  return (-1);
}

template <typename T>
inline void CloneValueArray(
    const T* p, T* q, Int32_t elements) {
  LOG_ERR << "Unsupported value type.";
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Int8_t>(
    const Int8_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt8Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Int8_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int8_t* p, Int32_t elements) {
  return EncodeInt8Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Int8_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int8_t* p, Int32_t elements) {
  return DecodeInt8Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Int8_t>(
    const Int8_t* p, Int8_t* q, Int32_t elements) {
  CloneInt8Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Uint8_t>(
    const Uint8_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint8Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Uint8_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint8_t* p, Int32_t elements) {
  return EncodeUint8Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Uint8_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint8_t* p, Int32_t elements) {
  return DecodeUint8Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Uint8_t>(
    const Uint8_t* p, Uint8_t* q, Int32_t elements) {
  CloneUint8Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Int16_t>(
    const Int16_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt16Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Int16_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int16_t* p, Int32_t elements) {
  return EncodeInt16Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Int16_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int16_t* p, Int32_t elements) {
  return DecodeInt16Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Int16_t>(
    const Int16_t* p, Int16_t* q, Int32_t elements) {
  CloneInt16Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Uint16_t>(
    const Uint16_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint16Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Uint16_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint16_t* p, Int32_t elements) {
  return EncodeUint16Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Uint16_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint16_t* p, Int32_t elements) {
  return DecodeUint16Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Uint16_t>(
    const Uint16_t* p, Uint16_t* q, Int32_t elements) {
  CloneUint16Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Int32_t>(
    const Int32_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt32Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Int32_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int32_t* p, Int32_t elements) {
  return EncodeInt32Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Int32_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int32_t* p, Int32_t elements) {
  return DecodeInt32Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Int32_t>(
    const Int32_t* p, Int32_t* q, Int32_t elements) {
  CloneInt32Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Uint32_t>(
    const Uint32_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint32Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Uint32_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint32_t* p, Int32_t elements) {
  return EncodeUint32Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Uint32_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint32_t* p, Int32_t elements) {
  return DecodeUint32Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Uint32_t>(
    const Uint32_t* p, Uint32_t* q, Int32_t elements) {
  CloneUint32Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Int64_t>(
    const Int64_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt64Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Int64_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int64_t* p, Int32_t elements) {
  return EncodeInt64Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Int64_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int64_t* p, Int32_t elements) {
  return DecodeInt64Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Int64_t>(
    const Int64_t* p, Int64_t* q, Int32_t elements) {
  CloneInt64Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Uint64_t>(
    const Uint64_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint64Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Uint64_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint64_t* p, Int32_t elements) {
  return EncodeUint64Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Uint64_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint64_t* p, Int32_t elements) {
  return DecodeUint64Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Uint64_t>(
    const Uint64_t* p, Uint64_t* q, Int32_t elements) {
  CloneUint64Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Float32_t>(
    const Float32_t *p, Int32_t elements) {
  return GetSizeOfEncodedFloat32Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Float32_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float32_t* p, Int32_t elements) {
  return EncodeFloat32Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Float32_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float32_t* p, Int32_t elements) {
  return DecodeFloat32Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Float32_t>(
    const Float32_t* p, Float32_t* q, Int32_t elements) {
  CloneFloat32Array(p, q, elements);
}



template <>
inline Int32_t GetSizeOfEncodedValueArray<Float64_t>(
    const Float64_t *p, Int32_t elements) {
  return GetSizeOfEncodedFloat64Array(p, elements);
}

template <>
inline Int32_t EncodeValueArray<Float64_t>(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float64_t* p, Int32_t elements) {
  return EncodeFloat64Array(buf, offset, maxlen, p, elements);
}

template <>
inline Int32_t DecodeValueArray<Float64_t>(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float64_t* p, Int32_t elements) {
  return DecodeFloat64Array(buf, offset, maxlen, p, elements);
}

template <>
inline void CloneValueArray<Float64_t>(
    const Float64_t* p, Float64_t* q, Int32_t elements) {
  CloneFloat64Array(p, q, elements);
}

#else

static inline Int32_t GetSizeOfEncodedValueArray(
    const Int8_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt8Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int8_t* p, Int32_t elements) {
  return EncodeInt8Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int8_t* p, Int32_t elements) {
  return DecodeInt8Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Int8_t* p, Int8_t* q, Int32_t elements) {
  CloneInt8Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Uint8_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint8Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint8_t* p, Int32_t elements) {
  return EncodeUint8Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint8_t* p, Int32_t elements) {
  return DecodeUint8Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Uint8_t* p, Uint8_t* q, Int32_t elements) {
  CloneUint8Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Int16_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt16Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int16_t* p, Int32_t elements) {
  return EncodeInt16Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int16_t* p, Int32_t elements) {
  return DecodeInt16Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Int16_t* p, Int16_t* q, Int32_t elements) {
  CloneInt16Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Uint16_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint16Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint16_t* p, Int32_t elements) {
  return EncodeUint16Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint16_t* p, Int32_t elements) {
  return DecodeUint16Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Uint16_t* p, Uint16_t* q, Int32_t elements) {
  CloneUint16Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Int32_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt32Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int32_t* p, Int32_t elements) {
  return EncodeInt32Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int32_t* p, Int32_t elements) {
  return DecodeInt32Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Int32_t* p, Int32_t* q, Int32_t elements) {
  CloneInt32Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Uint32_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint32Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint32_t* p, Int32_t elements) {
  return EncodeUint32Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint32_t* p, Int32_t elements) {
  return DecodeUint32Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Uint32_t* p, Uint32_t* q, Int32_t elements) {
  CloneUint32Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Int64_t *p, Int32_t elements) {
  return GetSizeOfEncodedInt64Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Int64_t* p, Int32_t elements) {
  return EncodeInt64Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Int64_t* p, Int32_t elements) {
  return DecodeInt64Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Int64_t* p, Int64_t* q, Int32_t elements) {
  CloneInt64Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Uint64_t *p, Int32_t elements) {
  return GetSizeOfEncodedUint64Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Uint64_t* p, Int32_t elements) {
  return EncodeUint64Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Uint64_t* p, Int32_t elements) {
  return DecodeUint64Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Uint64_t* p, Uint64_t* q, Int32_t elements) {
  CloneUint64Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Float32_t *p, Int32_t elements) {
  return GetSizeOfEncodedFloat32Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float32_t* p, Int32_t elements) {
  return EncodeFloat32Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float32_t* p, Int32_t elements) {
  return DecodeFloat32Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Float32_t* p, Float32_t* q, Int32_t elements) {
  CloneFloat32Array(p, q, elements);
}



static inline Int32_t GetSizeOfEncodedValueArray(
    const Float64_t *p, Int32_t elements) {
  return GetSizeOfEncodedFloat64Array(p, elements);
}

static inline Int32_t EncodeValueArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Float64_t* p, Int32_t elements) {
  return EncodeFloat64Array(buf, offset, maxlen, p, elements);
}

static inline Int32_t DecodeValueArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Float64_t* p, Int32_t elements) {
  return DecodeFloat64Array(buf, offset, maxlen, p, elements);
}

static inline void CloneValueArray(
    const Float64_t* p, Float64_t* q, Int32_t elements) {
  CloneFloat64Array(p, q, elements);
}

#endif


}  // namespace common
}  // namespace phoneix


#endif // PHOENIX_COMMON_SERIALIZATION_UTILS_H_


