/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_serialization.h
 * @brief      通讯用的数据序列化
 * @details    实现了基通讯用的数据序列化的操作函数
 *
 * @author     yaoyangz
 * @date       2023.03.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DATA_SERIAL_SERIAL_MSG_SCENESTORY_H_
#define PHOENIX_DATA_SERIAL_SERIAL_MSG_SCENESTORY_H_


#include "ad_msg.h"
#include "utils/macros.h"
#include "utils/com_utils.h"


namespace phoenix {
namespace data_serial {


/**
 * @brief 序列化 SceneStoryControlLine 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStoryControlLineArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryControlLine * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStoryControlLine 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryControlLineArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryControlLine* p, Int32_t elements);


/**
 * @brief 序列化 SceneStoryArea 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStorAreayArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryArea * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStoryArea 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryAreaArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryArea* p, Int32_t elements);

/**
 * @brief 序列化 SceneStoryCondition 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStoryConditionArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryCondition * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStoryCondition 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryConditionArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryCondition* p, Int32_t elements);


/**
 * @brief 序列化 SceneStoryAction 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStoryActionArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryAction * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStoryAction 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryActionArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryAction* p, Int32_t elements);


/**
 * @brief 序列化 SceneStory 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStoryArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStory * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStory 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStory* p, Int32_t elements);


/**
 * @brief 序列化 SceneStoryList 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeSceneStoryListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryList * p, Int32_t elements);

/**
 * @brief 反序列化 SceneStoryList 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeSceneStoryListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryList* p, Int32_t elements);



}  // namespace data_serial
}  // namespace phoenix


#endif // PHOENIX_DATA_SERIAL_SERIAL_VEC2D_H_
