/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_serialization.h
 * @brief      通讯用的数据序列化
 * @details    实现了基通讯用的数据序列化的操作函数
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
#ifndef PHOENIX_DATA_SERIAL_SERIAL_DRIVING_MAP_H_
#define PHOENIX_DATA_SERIAL_SERIAL_DRIVING_MAP_H_


#include "ad_msg.h"
#include "utils/macros.h"
#include "utils/com_utils.h"
#include "driving_map.h"


namespace phoenix {
namespace data_serial {


/**
 * @brief 序列化 LaneInfo 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeLaneInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::LaneInfo* p, Int32_t elements);

/**
 * @brief 反序列化 LaneInfo 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeLaneInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::LaneInfo* p, Int32_t elements);


Int32_t EncodeMapTrafficLightArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::MapTrafficLight* p, Int32_t elements);

Int32_t DecodeMapTrafficLightArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::MapTrafficLight* p, Int32_t elements);


/**
 * @brief 序列化 MapInfo 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeMapInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::MapInfo* p, Int32_t elements);

/**
 * @brief 反序列化 MapInfo 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeMapInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::MapInfo* p, Int32_t elements);


/**
 * @brief 序列化 CollisionRiskTestResult 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeCollisionRiskTestResultObjInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::CollisionTestResult::ObjInfo* p, Int32_t elements);

/**
 * @brief 反序列化 CollisionRiskTestResult 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeCollisionRiskTestResultObjInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::CollisionTestResult::ObjInfo* p, Int32_t elements);


/**
 * @brief 序列化 DrivingMapInfo::ReferenceLineInfo 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeReferenceLineInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo::ReferenceLineInfo* p, Int32_t elements);

/**
 * @brief 反序列化 DrivingMapInfo::ReferenceLineInfo 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeReferenceLineInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo::ReferenceLineInfo* p, Int32_t elements);


/**
 * @brief 序列化 DrivingMapInfo::ObstacleInfo 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeObstacleInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo::ObstacleInfo* p, Int32_t elements);

/**
 * @brief 反序列化 DrivingMapInfo::ObstacleInfo 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeObstacleInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo::ObstacleInfo* p, Int32_t elements);

/**
 * @brief 序列化 RoadBoundary 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeRoadBoundaryArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::RoadBoundary* p, Int32_t elements);

/**
 * @brief 反序列化 RoadBoundary 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeRoadBoundaryArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::RoadBoundary* p, Int32_t elements);


/**
 * @brief 序列化 DrivingMapInfo 数组
 * @param buf[in] 保存序列化后的数据的内存首地址
 * @param offset[in] 保存序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 保存序列化后的数据的内存的最大尺寸
 * @param p[in] 需要序列化的数组的起始地址
 * @param elements[in] 需要序列化的数组元素的个数
 * @return  序列化后的数据的尺寸, -1 表示序列化失败
 */
Int32_t EncodeDrivingMapInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo* p, Int32_t elements);

/**
 * @brief 反序列化 DrivingMapInfo 数组
 * @param buf[in] 序列化后的数据的内存首地址
 * @param offset[in] 序列化后的数据的内存首地址的偏移量
 * @param maxlen[in] 序列化后的数据的内存的最大尺寸
 * @param p[in] 保存反序列化后的数组的起始地址
 * @param elements[in] 需要反序列化的数组元素的个数
 * @return  反序列化后的数据的尺寸, -1 表示反序列化失败
 */
Int32_t DecodeDrivingMapInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo* p, Int32_t elements);


}  // namespace data_serial
}  // namespace phoenix


#endif // PHOENIX_DATA_SERIAL_SERIAL_DRIVING_MAP_H_
