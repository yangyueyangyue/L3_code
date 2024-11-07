/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       string_ring_buffer.cc
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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "utils/com_utils_c.h"
#include "container/string_ring_buffer_c.h"


#define MIN(a, b) (a) > (b) ? (b) : (a)


/*
 * @brief 获取写指针到环形缓冲区尾部的距离
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 写指针到环形缓冲区尾部的距离
 */
static Int32_t DistanceToEnd_W(StringRingBuf_t* ring_buff) {
  return (ring_buff->max_bytes - ring_buff->write_index);
}

/*
 * @brief 获取读指针到环形缓冲区尾部的距离
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 读指针到环形缓冲区尾部的距离
 */
static Int32_t DistanceToEnd_R(StringRingBuf_t* ring_buff) {
  return (ring_buff->max_bytes - ring_buff->read_index);
}

/*
 * @brief 获取环形缓冲区中未使用的空间大小
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 未使用的空间大小
 */
static Int32_t UnusedSpace(StringRingBuf_t* ring_buff) {
  return (ring_buff->max_bytes - ring_buff->data_count);
}

/*
 * @brief 获取环形缓冲区中已使用的空间大小
 * @param[in] ring_buff 指向字符串环形缓冲区的实例的地址
 * @return 已使用的空间大小
 */
static Int32_t UsedSpace(StringRingBuf_t* ring_buff) {
  return (ring_buff->data_count);
}

// 初始化字符串环形缓冲区
Int32_t Phoenix_Com_InitStringRingBuf(StringRingBuf_t* ring_buff, Char_t* buff,
    Int32_t max_bytes) {
  if ((Nullptr_t == ring_buff) || (Nullptr_t == buff) || (max_bytes <= 0)) {
    return (RET_STR_RING_BUF_NG);
  }

  ring_buff->read_index = 0;
  ring_buff->write_index = 0;
  ring_buff->max_bytes = max_bytes;
  ring_buff->buff = buff;
  ring_buff->data_count = 0;

  return (RET_STR_RING_BUF_OK);
}

// 销毁字符串环形缓冲区
Int32_t Phoenix_Com_ReleaseStringRingBuf(StringRingBuf_t* ring_buff) {
  if (Nullptr_t == ring_buff) {
    return (RET_STR_RING_BUF_NG);
  }

  ring_buff->read_index = 0;
  ring_buff->write_index = 0;
  ring_buff->max_bytes = 0;
  ring_buff->buff = Nullptr_t;
  ring_buff->data_count = 0;
  return (RET_STR_RING_BUF_OK);
}

// 获取字符串环形缓冲区中未读区域的起始索引
Int32_t Phoenix_Com_GetStringRingBufReadIndex(StringRingBuf_t* ring_buff) {
  if (Nullptr_t == ring_buff) {
    return (RET_STR_RING_BUF_NG);
  }
  return (ring_buff->read_index);
}

// 获取字符串环形缓冲区中未写区域的起始索引
Int32_t Phoenix_Com_GetStringRingBufWriteIndex(StringRingBuf_t* ring_buff) {
  if (Nullptr_t == ring_buff) {
    return (RET_STR_RING_BUF_NG);
  }
  return (ring_buff->write_index);
}

// 获取字符串环形缓冲区中未使用的空间的大小
Int32_t Phoenix_Com_GetStringRingBufUnusedSpace(StringRingBuf_t* ring_buff) {
  Int32_t size = 0;

  if (Nullptr_t == ring_buff) {
    return (RET_STR_RING_BUF_NG);
  }

  size = UnusedSpace(ring_buff);

  return (size);
}

// 获取字符串环形缓冲区中已使用的空间的大小
Int32_t Phoenix_Com_GetStringRingBufUsedSpace(StringRingBuf_t* ring_buff) {
  Int32_t size = 0;

  if (Nullptr_t == ring_buff) {
    return (RET_STR_RING_BUF_NG);
  }

  size = UsedSpace(ring_buff);

  return (size);
}

// 向字符串环形缓冲区中写入数据(如果缓冲区中空间不足，则不写入)
Int32_t Phoenix_Com_WriteToStringRingBuf(
    StringRingBuf_t* ring_buff, Char_t* src_addr, Int32_t write_size) {
  Int32_t unused_size = 0;
  Int32_t distance_to_end = 0;

  if ((Nullptr_t == ring_buff) || (Nullptr_t == src_addr) ||
      (write_size <= 0)) {
    return (RET_STR_RING_BUF_NG);
  }

  unused_size = UnusedSpace(ring_buff);
  distance_to_end = DistanceToEnd_W(ring_buff);

  if (write_size > unused_size) {
    return (RET_STR_RING_BUF_NO_SPACE);
  } else {
    if (write_size > distance_to_end) {
      phoenix_com_memcpy(&(ring_buff->buff[ring_buff->write_index]),
             src_addr, distance_to_end);
      phoenix_com_memcpy(&(ring_buff->buff[0]), (src_addr + distance_to_end),
             write_size - distance_to_end);
      ring_buff->write_index = write_size - distance_to_end;
    } else {
      phoenix_com_memcpy(&(ring_buff->buff[ring_buff->write_index]),
          src_addr, write_size);
      ring_buff->write_index += write_size;
    }

    ring_buff->data_count += write_size;
    if (ring_buff->data_count > ring_buff->max_bytes) {
      ring_buff->data_count = ring_buff->max_bytes;
    }

    if (ring_buff->write_index > ring_buff->max_bytes) {
      ring_buff->write_index -= ring_buff->max_bytes;
    }
  }

  return (RET_STR_RING_BUF_OK);
}

// 从字符串环形缓冲区中读取指定大小的数据(如果缓冲区中的数据小于指定的大小，则不读取)
Int32_t Phoenix_Com_ReadFromStringRingBufWithSpecialSize(
    StringRingBuf_t* ring_buff, Char_t* dest_addr, Int32_t read_size) {
  Int32_t used_size = 0;
  Int32_t distance_to_end = 0;

  if ((Nullptr_t == ring_buff) || (Nullptr_t == dest_addr) ||
      (read_size <= 0)) {
    return (RET_STR_RING_BUF_NG);
  }

  used_size = UsedSpace(ring_buff);
  distance_to_end = DistanceToEnd_R(ring_buff);

  if (read_size > used_size) {
    return (RET_STR_RING_BUF_NO_DATA);
  } else {
    if (read_size > distance_to_end) {
      phoenix_com_memcpy(dest_addr, &(ring_buff->buff[ring_buff->read_index]),
          distance_to_end);
      phoenix_com_memcpy((dest_addr + distance_to_end), &(ring_buff->buff[0]),
          read_size - distance_to_end);
      ring_buff->read_index = read_size - distance_to_end;
    } else {
      phoenix_com_memcpy(dest_addr, &(ring_buff->buff[ring_buff->read_index]),
          read_size);
      ring_buff->read_index += read_size;
    }

    if (ring_buff->read_index > ring_buff->max_bytes) {
      ring_buff->read_index -= ring_buff->max_bytes;
    }
  }

  return (RET_STR_RING_BUF_OK);
}

// 向字符串环形缓冲区中写入数据(如果缓冲区中空间不足，则覆盖之前写入的数据)
Int32_t Phoenix_Com_WriteToStringRingBufOverride(StringRingBuf_t* ring_buff,
    const Char_t* src_addr, Int32_t write_size) {
  Int32_t unused_size = 0;
  Int32_t distance_to_end = 0;

  if ((Nullptr_t == ring_buff) || (Nullptr_t == src_addr) ||
      (write_size <= 0)) {
    return (RET_STR_RING_BUF_NG);
  }
  if (write_size > ring_buff->max_bytes) {
    return (RET_STR_RING_BUF_NO_SPACE);
  }

  unused_size = UnusedSpace(ring_buff);
  distance_to_end = DistanceToEnd_W(ring_buff);

  if (write_size > distance_to_end) {
    phoenix_com_memcpy(&(ring_buff->buff[ring_buff->write_index]),
        src_addr, distance_to_end);
    phoenix_com_memcpy(&(ring_buff->buff[0]), (src_addr + distance_to_end),
        write_size - distance_to_end);
    ring_buff->write_index = write_size - distance_to_end;
    if (ring_buff->write_index > ring_buff->read_index) {
       ring_buff->read_index = ring_buff->write_index;
    }
  } else {
    phoenix_com_memcpy(&(ring_buff->buff[ring_buff->write_index]), src_addr, write_size);
    ring_buff->write_index += write_size;
  }

  if (ring_buff->write_index >= ring_buff->max_bytes) {
    ring_buff->write_index -= ring_buff->max_bytes;
  }

  if (ring_buff->read_index >= ring_buff->max_bytes) {
    ring_buff->read_index -= ring_buff->max_bytes;
  }
  ring_buff->data_count += write_size;
  if (ring_buff->data_count > ring_buff->max_bytes) {
    ring_buff->data_count = ring_buff->max_bytes;
  }
  return (write_size);
}

// 从字符串环形缓冲区中读取数据(如果缓冲区中的数据小于指定的大小则读取缓冲区已有的数据)
Int32_t Phoenix_Com_ReadFromStringRingBuf(StringRingBuf_t* ring_buff,
    Char_t* dest_addr, Int32_t read_size) {
  Int32_t used_size = 0;
  Int32_t distance_to_end = 0;

  if ((Nullptr_t == ring_buff) || (Nullptr_t == dest_addr) ||
      (read_size <= 0)) {
    return (RET_STR_RING_BUF_NG);
  }

  used_size = UsedSpace(ring_buff);
  distance_to_end = DistanceToEnd_R(ring_buff);

  if (read_size > used_size) {
    read_size = used_size;
  }

  if (read_size > 0) {
    if (read_size > distance_to_end) {
      phoenix_com_memcpy(dest_addr, &(ring_buff->buff[ring_buff->read_index]),
          distance_to_end);
      phoenix_com_memcpy((dest_addr + distance_to_end), &(ring_buff->buff[0]),
          read_size - distance_to_end);
      ring_buff->read_index = read_size - distance_to_end;
    } else {
      phoenix_com_memcpy(dest_addr, &(ring_buff->buff[ring_buff->read_index]), read_size);
      ring_buff->read_index += read_size;
    }
    if (ring_buff->read_index >= ring_buff->max_bytes) {
      ring_buff->read_index -= ring_buff->max_bytes;
    }
  }

  return (read_size);
}

