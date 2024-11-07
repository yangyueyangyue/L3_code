/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       ring_buffer_c.h
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

#ifndef PHOENIX_COMMON_CONTAINER_RING_BUFFER_C_H_
#define PHOENIX_COMMON_CONTAINER_RING_BUFFER_C_H_


#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct RingBuffer_t
 * @brief 定义了Ring Buffer的存储结构
 */
typedef struct _RingBuffer_t RingBuffer_t;
struct _RingBuffer_t {
  // 最大数据个数
  Int32_t max_data_num_;
  // 数据类型的字节数
  Int32_t type_bytes_;
  // 写入位置的索引
  Int32_t write_position_;
  // 读取位置的索引
  Int32_t read_position_;
  // 已保存的数据的数量
  Int32_t data_count_;
  // 用来保存数据
  Uint8_t* data_buff_;
};

/**
 * @class RingBufferIterator_t
 * @brief 环形数据缓冲区的迭代器
 */
typedef struct _RingBufferIterator_t RingBufferIterator_t;
struct _RingBufferIterator_t {
  // 指向环形数据缓冲区的指针
  RingBuffer_t* ring_buff_;
  // 指向环形数据缓冲区的索引
  Int32_t ring_buff_index_;
  // 环形缓冲区是否已满
  Bool_t is_ring_buff_full_;
};


/**
 * @brief 初始化RingBuffer实例
 * @param[in] ins RingBuffer实例
 * @param[in] max_data_num 最大数据个数
 * @param[in] type_bytes 数据类型的字节数
 * @param[in] buff 数据缓存区（必须有能足够容纳max_data_num*type_bytes字节数的空间）
 */
void Phoenix_Com_RingBuffer_Init(
    RingBuffer_t* ins, Int32_t max_data_num, Int32_t type_bytes, void* buff);

/**
 * @brief 清除内部数据
 * @param[in] ins RingBuffer实例
 */
void Phoenix_Com_RingBuffer_Clear(RingBuffer_t* ins);

/**
 * @brief 判断环形数据缓冲区是否为空
 * @param[in] ins RingBuffer实例
 * @return True_t - 空的, False_t - 不是空的
 */
Bool_t Phoenix_Com_RingBuffer_IsEmpty(const RingBuffer_t* ins);

/**
 * @brief 判断环形数据缓冲区是否已经存满了
 * @param[in] ins RingBuffer实例
 * @return True_t - 满的, False_t - 不是满的
 */
Bool_t Phoenix_Com_RingBuffer_IsFull(const RingBuffer_t* ins);

/**
 * @brief 获取环形数据缓冲区内已有数据的数量
 * @param[in] ins RingBuffer实例
 * @return 环形数据缓冲区内已有数据的数量
 */
Int32_t Phoenix_Com_RingBuffer_GetSize(const RingBuffer_t* ins);

/**
 * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则不添加）
 * @param[in] ins RingBuffer实例
 * @param[in] data 待添加的数据
 * @return True_t~添加成功, False_t~添加失败
 */
Bool_t Phoenix_Com_RingBuffer_PushBack(RingBuffer_t* ins, const void* data);

/**
 * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则不分配）
 * @param[in] ins RingBuffer实例
 * @return 分配的空间的地址（Null_t-分配失败)
 */
void* Phoenix_Com_RingBuffer_Allocate(RingBuffer_t* ins);

/**
 * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则覆盖之前的数据）
 * @param[in] ins RingBuffer实例
 * @param[in] data 待添加的数据
 * @return True_t-添加成功, False_t-添加失败
 */
Bool_t Phoenix_Com_RingBuffer_PushBackOverride(
    RingBuffer_t* ins, const void* data);

/**
 * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则覆盖之前的数据）
 * @param[in] ins RingBuffer实例
 * @return 分配的空间的地址
 */
void* Phoenix_Com_RingBuffer_AllocateOverride(RingBuffer_t* ins);

/**
 * @brief 获取环形数据缓冲区头部的数据，并从缓冲区中移除此数据
 * @param[in] ins RingBuffer实例
 * @return 返回的数据, (Null_t-缓冲区内无数据)
 */
const void* Phoenix_Com_RingBuffer_PopFront(RingBuffer_t* ins);

/**
 * @brief 从环形数据缓冲区头部开始移除指定数量的数据
 * @param[in] ins RingBuffer实例
 * @param[in] num 要移除的数据的数量
 */
void Phoenix_Com_RingBuffer_PopFromFront(RingBuffer_t* ins, Int32_t num);

/**
 * @brief 获取环形数据缓冲区尾部的数据，并从缓冲区中移除此数据
 * @param[in] ins RingBuffer实例
 * @return 返回的数据, (Null_t-缓冲区内无数据)
 */
const void* Phoenix_Com_RingBuffer_PopBack(RingBuffer_t* ins);

/**
 * @brief 从环形数据缓冲区尾部开始移除指定数量的数据
 * @param[in] ins RingBuffer实例
 * @param[in] num 要移除的数据的数量
 */
void Phoenix_Com_RingBuffer_PopFromBack(RingBuffer_t* ins, Int32_t num);

/**
 * @brief 获取环形数据缓冲区头部的数据
 * @param[in] ins RingBuffer实例
 * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
 */
void* Phoenix_Com_RingBuffer_GetFront(RingBuffer_t* ins);

/**
 * @brief 获取环形数据缓冲区尾部的数据
 * @param[in] ins RingBuffer实例
 * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
 */
void* Phoenix_Com_RingBuffer_GetBack(RingBuffer_t* ins);

/**
 * @brief 获取数据索引所指向的数据
 * @param[in] ins RingBuffer实例
 * @param[in] index 数据索引
 * @return 数据索引所指向的数据
 */
void* Phoenix_Com_RingBuffer_GetDataByIdx(RingBuffer_t* ins, Int32_t index);

/**
 * @brief 返回环形数据缓冲区内保存的数据的起始位置的迭代器
 * @param[in] ins RingBuffer实例
 * @param[out] it RingBufferIterator实例
 */
void Phoenix_Com_RingBuffer_SetItToBegin(
    RingBuffer_t* ins, RingBufferIterator_t* it);

/**
 * @brief 返回环形数据缓冲区内保存的数据的结束位置的迭代器
 * @param[in] ins RingBuffer实例
 * @param[out] it RingBufferIterator实例
 */
void Phoenix_Com_RingBuffer_SetItToEnd(
    RingBuffer_t* ins, RingBufferIterator_t* it);


/**
 * @brief 判断当前迭代器是否与另一个迭代器相等
 * @param[in]  ins 当前迭代器
 * @param[in]  rv 另一个迭代器
 * @return True_t-相等, False_t-不相等
 */
Bool_t Phoenix_Com_RingBufferIterator_IsEqual(
    const RingBufferIterator_t* ins, const RingBufferIterator_t* rv);

/**
 * @brief 判断当前迭代器是否与另一个迭代器不相等
 * @param[in]  ins 当前迭代器
 * @param[in]  rv 另一个迭代器
 * @return True_t-不相等, False_t-相等
 */
Bool_t Phoenix_Com_RingBufferIterator_IsNotEqual(
    const RingBufferIterator_t* ins, const RingBufferIterator_t* rv);

/**
 * @brief 迭代器自增
 * @param[in]  ins 当前迭代器
 */
void Phoenix_Com_RingBufferIterator_Increase(RingBufferIterator_t* ins);

/**
 * @brief 偏移迭代器
 * @param[in]  ins 当前迭代器
 * @param[in] offset 偏移量(+ ~ forward, - ~ backward)
 */
void Phoenix_Com_RingBufferIterator_Move(
    RingBufferIterator_t* ins, Int32_t offset);

/**
 * @brief 返回当前迭代器所指向的数据
 * @param[in]  ins 当前迭代器
 * @return 当前迭代器所指向的数据
 */
void* Phoenix_Com_RingBufferIterator_GetCurrent(RingBufferIterator_t* ins);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_CONTAINER_RING_BUFFER_C_H_

