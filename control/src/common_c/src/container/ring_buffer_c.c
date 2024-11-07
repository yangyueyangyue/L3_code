/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       ring_buffer_c.c
 * @brief      环形数据缓冲区
 * @details    定义环形数据缓冲区
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "container/ring_buffer_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"


/*
 * @brief Limit the ring buffer index
 * @param[in] Instance of Ring Buffer
 * @param[in] start_real_index start real index
 * @param[in] offset offset to the start real index
 * @return real ring buffer index
 *
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_RingBuffer_BoundIndex(
    const RingBuffer_t* ins, Int32_t start_real_index, Int32_t offset) {
  Int32_t real_index = start_real_index;

  offset %= ins->max_data_num_;
  if (0 == offset || Phoenix_Com_RingBuffer_IsEmpty(ins)) {
    return (real_index);
  }

  if (offset > 0) {
    if (ins->write_position_ > ins->read_position_) {
      if ((start_real_index+offset) >= ins->write_position_) {
        real_index = ins->write_position_;
      } else {
        real_index += offset;
      }
    } else {
      if (start_real_index >= ins->read_position_) {
        if ((start_real_index+offset) >= ins->max_data_num_) {
          real_index = start_real_index + offset - ins->max_data_num_;
          if (real_index >= ins->write_position_) {
            real_index = ins->write_position_;
          }
        } else {
          real_index = start_real_index + offset;
        }
      } else {
        real_index = start_real_index + offset;
        if (real_index >= ins->write_position_) {
          real_index = ins->write_position_;
        }
      }
    }
  } else {
    if (ins->write_position_ > ins->read_position_) {
      if ((start_real_index+offset) <= ins->read_position_) {
        real_index = ins->read_position_;
      } else {
        real_index += offset;
      }
    } else {
      if (start_real_index > ins->read_position_) {
        real_index = start_real_index + offset;
        if (real_index <= ins->read_position_) {
          real_index = ins->read_position_;
        }
      } else if (start_real_index == ins->read_position_) {
        if (ins->read_position_ == ins->write_position_) {
          if ((start_real_index+offset) < 0) {
            real_index = start_real_index + offset + ins->max_data_num_;
            if (real_index <= ins->read_position_) {
              real_index = ins->read_position_;
            }
          } else {
            real_index = start_real_index + offset;
          }
        } else {
          real_index = ins->read_position_;
        }
      } else {
        if ((start_real_index+offset) < 0) {
          real_index = start_real_index + offset + ins->max_data_num_;
          if (real_index <= ins->read_position_) {
            real_index = ins->read_position_;
          }
        } else {
          real_index = start_real_index + offset;
        }
      }
    }
  }

  return (real_index);
}


/*
 * @brief 初始化RingBuffer实例
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_Init(
    RingBuffer_t* ins, Int32_t max_data_num, Int32_t type_bytes, void* buff) {
  ins->max_data_num_ = max_data_num;
  ins->type_bytes_ = type_bytes;
  ins->write_position_ = 0;
  ins->read_position_ = 0;
  ins->data_count_ = 0;
  ins->data_buff_ = buff;
}

/*
 * @brief 清除内部数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_Clear(RingBuffer_t* ins) {
  ins->write_position_ = 0;
  ins->read_position_ = 0;
  ins->data_count_ = 0;
}

/*
 * @brief 判断环形数据缓冲区是否为空
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBuffer_IsEmpty(const RingBuffer_t* ins) {
  return (ins->data_count_ < 1);
}

/*
 * @brief 判断环形数据缓冲区是否已经存满了
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBuffer_IsFull(const RingBuffer_t* ins) {
  return (ins->data_count_ >= ins->max_data_num_);
}

/*
 * @brief 获取环形数据缓冲区内已有数据的数量
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_RingBuffer_GetSize(const RingBuffer_t* ins) {
  return (ins->data_count_);
}

/*
 * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则不添加）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBuffer_PushBack(RingBuffer_t* ins, const void* data) {
  Bool_t ret = True_t;
  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    phoenix_com_memcpy(
          &(ins->data_buff_[ins->write_position_*ins->type_bytes_]),
          data, ins->type_bytes_);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    ret = False_t;
  } else {
    phoenix_com_memcpy(
          &(ins->data_buff_[ins->write_position_*ins->type_bytes_]),
          data, ins->type_bytes_);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  }

  return (ret);
}

/*
 * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则不分配）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBuffer_Allocate(RingBuffer_t* ins) {
  Uint8_t* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    data = &(ins->data_buff_[ins->write_position_*ins->type_bytes_]);
    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    data = Null_t;
  } else {
    data = &(ins->data_buff_[ins->write_position_*ins->type_bytes_]);
    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  }

  return (data);
}

/*
 * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则覆盖之前的数据）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBuffer_PushBackOverride(
    RingBuffer_t* ins, const void* data) {
  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    phoenix_com_memcpy(
          &(ins->data_buff_[ins->write_position_*ins->type_bytes_]),
          data, ins->type_bytes_);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
    // printf("ins->data_count_=%d\n", ins->data_count_);
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    phoenix_com_memcpy(
          &(ins->data_buff_[ins->write_position_*ins->type_bytes_]),
          data, ins->type_bytes_);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ++ins->read_position_;
    if (ins->read_position_ >= ins->max_data_num_) {
      ins->read_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  } else {
    phoenix_com_memcpy(
          &(ins->data_buff_[ins->write_position_*ins->type_bytes_]),
          data, ins->type_bytes_);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  }

  return True_t;
}

/*
 * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则覆盖之前的数据）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBuffer_AllocateOverride(RingBuffer_t* ins) {
  void* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    data = &(ins->data_buff_[ins->write_position_*ins->type_bytes_]);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    data = &(ins->data_buff_[ins->write_position_*ins->type_bytes_]);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ++ins->read_position_;
    if (ins->read_position_ >= ins->max_data_num_) {
      ins->read_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  } else {
    data = &(ins->data_buff_[ins->write_position_*ins->type_bytes_]);

    ++ins->write_position_;
    if (ins->write_position_ >= ins->max_data_num_) {
      ins->write_position_ -= ins->max_data_num_;
    }
    ins->data_count_++;
    if (ins->data_count_ > ins->max_data_num_) {
      ins->data_count_ = ins->max_data_num_;
    }
  }

  return (data);
}

/*
 * @brief 获取环形数据缓冲区头部的数据，并从缓冲区中移除此数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
const void* Phoenix_Com_RingBuffer_PopFront(RingBuffer_t* ins) {
  void* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    data = &(ins->data_buff_[ins->read_position_*ins->type_bytes_]);

    ++ins->read_position_;
    if (ins->read_position_ >= ins->max_data_num_) {
      ins->read_position_ -= ins->max_data_num_;
    }
    ins->data_count_--;
    if (ins->data_count_ < 0) {
      ins->data_count_ = 0;
    }
  } else {
    data = &(ins->data_buff_[ins->read_position_*ins->type_bytes_]);

    ++ins->read_position_;
    if (ins->read_position_ >= ins->max_data_num_) {
      ins->read_position_ -= ins->max_data_num_;
    }
    ins->data_count_--;
    if (ins->data_count_ < 0) {
      ins->data_count_ = 0;
    }
  }

  return (data);
}

/*
 * @brief 从环形数据缓冲区头部开始移除指定数量的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_PopFromFront(RingBuffer_t* ins, Int32_t num) {
  if (num < 1 || Phoenix_Com_RingBuffer_IsEmpty(ins)) {
    return;
  }
  if (num >= ins->data_count_) {
    ins->read_position_ = ins->write_position_;
    ins->data_count_ = 0;
    return;
  }

  Int32_t index = Phoenix_Com_RingBuffer_BoundIndex(
        ins, ins->read_position_, num);
  if (index == ins->write_position_) {
    ins->read_position_ = ins->write_position_;
    ins->data_count_ = 0;
  } else {
    ins->read_position_ = index;
    ins->data_count_ -= num;
  }
}

/*
 * @brief 获取环形数据缓冲区尾部的数据，并从缓冲区中移除此数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
const void* Phoenix_Com_RingBuffer_PopBack(RingBuffer_t* ins) {
  void* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    Int32_t read_pose = ins->write_position_ - 1;
    if (read_pose < 0) {
      read_pose += ins->max_data_num_;
    }
    data = &(ins->data_buff_[read_pose*ins->type_bytes_]);
    ins->write_position_ = read_pose;

    ins->data_count_--;
    if (ins->data_count_ < 0) {
      ins->data_count_ = 0;
    }
  } else {
    Int32_t read_pose = ins->write_position_ - 1;
    if (read_pose < 0) {
      read_pose += ins->max_data_num_;
    }
    data = &(ins->data_buff_[read_pose*ins->type_bytes_]);
    ins->write_position_ = read_pose;

    ins->data_count_--;
    if (ins->data_count_ < 0) {
      ins->data_count_ = 0;
    }
  }

  return (data);
}

/*
 * @brief 从环形数据缓冲区尾部开始移除指定数量的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_PopFromBack(RingBuffer_t* ins, Int32_t num) {
  if (num < 1 || Phoenix_Com_RingBuffer_IsEmpty(ins)) {
    return;
  }
  if (num >= ins->data_count_) {
    ins->write_position_ = ins->read_position_;
    ins->data_count_ = 0;
    return;
  }

  Int32_t index = Phoenix_Com_RingBuffer_BoundIndex(
        ins, ins->write_position_, -num);
  if (index == ins->read_position_) {
    ins->write_position_ = ins->read_position_;
    ins->data_count_ = 0;
  } else {
    ins->write_position_ = index;
    ins->data_count_ -= num;
  }
}

/*
 * @brief 获取环形数据缓冲区头部的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBuffer_GetFront(RingBuffer_t* ins) {
  void* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    data = &(ins->data_buff_[ins->read_position_*ins->type_bytes_]);
  } else {
    data = &(ins->data_buff_[ins->read_position_*ins->type_bytes_]);
  }

  return (data);
}

/*
 * @brief 获取环形数据缓冲区尾部的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBuffer_GetBack(RingBuffer_t* ins) {
  void* data = Null_t;

  if ((ins->write_position_ == ins->read_position_) &&
      (ins->data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((ins->write_position_ == ins->read_position_) &&
             (ins->data_count_ >= ins->max_data_num_)) {
    // buffer is full
    Int32_t read_pose = ins->write_position_ - 1;
    if (read_pose < 0) {
      read_pose += ins->max_data_num_;
    }
    data = &(ins->data_buff_[read_pose*ins->type_bytes_]);
  } else {
    Int32_t read_pose = ins->write_position_ - 1;
    if (read_pose < 0) {
      read_pose = ins->max_data_num_ -1;
    }
    data = &(ins->data_buff_[read_pose*ins->type_bytes_]);
  }

  return (data);
}

/*
 * @brief 获取数据索引所指向的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBuffer_GetDataByIdx(RingBuffer_t* ins, Int32_t index) {
  if (index < 0 || index >= ins->data_count_) {
    // LOG_ERR_C("Invalid index (%d).", index);
    return Null_t;
  }

  Int32_t real_index = Phoenix_Com_RingBuffer_BoundIndex(
        ins, ins->read_position_, index);

  return &(ins->data_buff_[real_index*ins->type_bytes_]);
}

/*
 * @brief 返回环形数据缓冲区内保存的数据的起始位置的迭代器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_SetItToBegin(
    RingBuffer_t* ins, RingBufferIterator_t* it) {
  it->ring_buff_ = ins;
  it->ring_buff_index_ = ins->read_position_;
  it->is_ring_buff_full_ =
      ins->data_count_>= ins->max_data_num_ ? True_t : False_t;
}

/*
 * @brief 返回环形数据缓冲区内保存的数据的结束位置的迭代器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBuffer_SetItToEnd(
    RingBuffer_t* ins, RingBufferIterator_t* it) {
  it->ring_buff_ = ins;
  it->ring_buff_index_ = ins->write_position_;
  it->is_ring_buff_full_ =
      ins->data_count_>= ins->max_data_num_ ? True_t : False_t;
}


/*
 * @brief 判断当前迭代器是否与另一个迭代器相等
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBufferIterator_IsEqual(
    const RingBufferIterator_t* ins, const RingBufferIterator_t* rv) {
  return ((ins->ring_buff_index_ == rv->ring_buff_index_) &&
          (False_t == ins->is_ring_buff_full_));
}

/*
 * @brief 判断当前迭代器是否与另一个迭代器不相等
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Bool_t Phoenix_Com_RingBufferIterator_IsNotEqual(
    const RingBufferIterator_t* ins, const RingBufferIterator_t* rv) {
  return (!Phoenix_Com_RingBufferIterator_IsEqual(ins, rv));
}

/*
 * @brief 迭代器自增
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBufferIterator_Increase(RingBufferIterator_t* ins) {
  ++ins->ring_buff_index_;
  ins->is_ring_buff_full_ = False_t;
  if (ins->ring_buff_index_ >= ins->ring_buff_->max_data_num_) {
    ins->ring_buff_index_ -= ins->ring_buff_->max_data_num_;
  }
}

/*
 * @brief 偏移迭代器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_RingBufferIterator_Move(
    RingBufferIterator_t* ins, Int32_t offset) {
  offset %= ins->ring_buff_->max_data_num_;
  if (0 == offset || Phoenix_Com_RingBuffer_IsEmpty(ins->ring_buff_)) {
    return;
  }

  if (ins->ring_buff_->write_position_ == ins->ring_buff_->read_position_) {
    if (ins->ring_buff_index_ == ins->ring_buff_->write_position_) {
      if (offset > 0 && !ins->is_ring_buff_full_) {
        return;
      }
      if (offset < 0 && ins->is_ring_buff_full_) {
        return;
      }
    }
  }

  Int32_t index = Phoenix_Com_RingBuffer_BoundIndex(
        ins->ring_buff_, ins->ring_buff_index_, offset);
  if (offset < 0 && index == ins->ring_buff_->read_position_) {
    ins->is_ring_buff_full_ = Phoenix_Com_RingBuffer_IsFull(ins->ring_buff_);
  } else {
    ins->is_ring_buff_full_ = True_t;
  }

  ins->ring_buff_index_ = index;
}

/*
 * @brief 返回当前迭代器所指向的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void* Phoenix_Com_RingBufferIterator_GetCurrent(RingBufferIterator_t* ins) {
  return &(ins->ring_buff_->data_buff_[ins->ring_buff_index_*ins->ring_buff_->type_bytes_]);
}

